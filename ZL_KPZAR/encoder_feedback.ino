/****************************************************************************
  编码器测速 + 防后溜用实时速度反馈

  选用当前工程中空闲的 4 个引脚：
  左编码器：A1(A相), A2(B相)
  右编码器：A4(A相), A5(B相)

  当前已占用引脚：
  0/1  -> Serial 总线马达
  3/5/6/7/8/9 -> 6路舵机
  11/12/A0/A3 -> PS2 手柄

  所以这里使用：A1 A2 A4 A5
****************************************************************************/

#include <Arduino.h>

// ================= 编码器接线定义 =================
#define ENC_LEFT_A   A1
#define ENC_LEFT_B   A2
#define ENC_RIGHT_A  A4
#define ENC_RIGHT_B  A5

// 如果你发现“手推小车前进时，测速却显示为负方向”，
// 只需要把对应的符号从 1 改成 -1。
#define ENC_LEFT_SIGN   1
#define ENC_RIGHT_SIGN  1

// ================= 速度滤波参数 =================
const unsigned long ENCODER_SPEED_UPDATE_MS = 20;  // 50Hz 更新速度
const unsigned long ENCODER_DECODE_ALIVE_TIMEOUT_MS = 300;

volatile long g_left_encoder_ticks  = 0;
volatile long g_right_encoder_ticks = 0;

volatile uint8_t g_encoder_prev_left_state  = 0;
volatile uint8_t g_encoder_prev_right_state = 0;
volatile bool g_encoder_transition_seen = false;

long g_left_speed_raw  = 0;   // 每20ms的脉冲增量
long g_right_speed_raw = 0;   // 每20ms的脉冲增量
int  g_vehicle_speed   = 0;   // 左右平均后的滤波速度，正=前进，负=后退
volatile bool g_encoder_decode_alive = false;
unsigned long g_last_encoder_transition_ms = 0;

static const int8_t QUAD_TABLE[16] = {
  0, -1,  1,  0,
  1,  0,  0, -1,
 -1,  0,  0,  1,
  0,  1, -1,  0
};

static inline int8_t encoder_get_quad_delta(uint8_t prev_state, uint8_t curr_state) {
  return QUAD_TABLE[(prev_state << 2) | curr_state];
}

void setup_encoder_feedback(void) {
  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);

  // 初始化当前AB状态
  g_encoder_prev_left_state =
      ((digitalRead(ENC_LEFT_A) ? 1 : 0) << 1) |
       (digitalRead(ENC_LEFT_B) ? 1 : 0);

  g_encoder_prev_right_state =
      ((digitalRead(ENC_RIGHT_A) ? 1 : 0) << 1) |
       (digitalRead(ENC_RIGHT_B) ? 1 : 0);

  // A1/A2/A4/A5 都在 PORTC，上升沿/下降沿变化都触发 PCINT1_vect
  PCICR  |= (1 << PCIE1);  // 开启 PORTC 的 Pin Change Interrupt
  PCMSK1 |= (1 << PCINT9)  // A1
         |  (1 << PCINT10) // A2
         |  (1 << PCINT12) // A4
         |  (1 << PCINT13);// A5
}

ISR(PCINT1_vect) {
  uint8_t portc_now = PINC;

  uint8_t left_state =
      (((portc_now >> 1) & 0x01) << 1) |
       ((portc_now >> 2) & 0x01);

  uint8_t right_state =
      (((portc_now >> 4) & 0x01) << 1) |
       ((portc_now >> 5) & 0x01);

  int8_t left_delta = encoder_get_quad_delta(g_encoder_prev_left_state, left_state);
  int8_t right_delta = encoder_get_quad_delta(g_encoder_prev_right_state, right_state);

  if (left_delta != 0) {
    g_left_encoder_ticks += (long)(left_delta * ENC_LEFT_SIGN);
    g_encoder_transition_seen = true;
    g_encoder_prev_left_state = left_state;
  } else {
    g_encoder_prev_left_state = left_state;
  }

  if (right_delta != 0) {
    g_right_encoder_ticks += (long)(right_delta * ENC_RIGHT_SIGN);
    g_encoder_transition_seen = true;
    g_encoder_prev_right_state = right_state;
  } else {
    g_encoder_prev_right_state = right_state;
  }
}

void loop_encoder_feedback(void) {
  static unsigned long last_update_ms = 0;
  static long last_left_ticks = 0;
  static long last_right_ticks = 0;

  if (millis() - last_update_ms < ENCODER_SPEED_UPDATE_MS) {
    return;
  }
  last_update_ms = millis();

  long left_ticks_now;
  long right_ticks_now;

  noInterrupts();
  left_ticks_now = g_left_encoder_ticks;
  right_ticks_now = g_right_encoder_ticks;
  interrupts();

  long dl = left_ticks_now - last_left_ticks;
  long dr = right_ticks_now - last_right_ticks;

  last_left_ticks = left_ticks_now;
  last_right_ticks = right_ticks_now;

  g_left_speed_raw = dl;
  g_right_speed_raw = dr;

  int instant_vehicle_speed = (int)((dl + dr) / 2L);

  // 一阶低通滤波，减小抖动
  g_vehicle_speed = (g_vehicle_speed * 3 + instant_vehicle_speed) / 4;

  bool seen_transition;
  noInterrupts();
  seen_transition = g_encoder_transition_seen;
  g_encoder_transition_seen = false;
  interrupts();
  if (seen_transition) {
    g_last_encoder_transition_ms = last_update_ms;
  }

  bool alive_now = (last_update_ms - g_last_encoder_transition_ms) <= ENCODER_DECODE_ALIVE_TIMEOUT_MS;
  g_encoder_decode_alive = alive_now;
}

bool encoder_decode_is_alive(void) {
  bool alive;
  noInterrupts();
  alive = g_encoder_decode_alive;
  interrupts();
  return alive;
}
