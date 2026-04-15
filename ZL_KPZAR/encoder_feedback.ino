/****************************************************************************
  JGB520 编码器测速（按手册口径改写）
  方案：AB 两路“上升沿计数 + 相位判向”
  手册口径：AB 合计 780 脉冲/圈，建议每 20ms 读取一次

  当前接线建议：
  左编码器：A -> A1, B -> A2
  右编码器：A -> A4, B -> A5

  说明：
  1. 保留了 g_left_encoder_ticks / g_right_encoder_ticks / g_vehicle_speed
     这些全局变量名，这样 loopkey 里的主动驻车逻辑可以继续直接用。
  2. 新增了 mm/s 与 rpm 变量，后续如果你要调参或做显示会更方便。
****************************************************************************/

#include <Arduino.h>

// ================= 编码器接线定义 =================
#define ENC_LEFT_A   A1
#define ENC_LEFT_B   A2
#define ENC_RIGHT_A  A4
#define ENC_RIGHT_B  A5

// 如果你发现“手推小车前进时，速度却显示为负”，就把对应符号改成 -1
#define ENC_LEFT_SIGN   1
#define ENC_RIGHT_SIGN  1

// ================= 手册参数 =================
const unsigned long ENCODER_SPEED_UPDATE_MS = 20;   // 手册建议 20ms 读取一次
const long ENCODER_PULSES_PER_REV = 780;            // 手册：AB 合计一圈 780 脉冲

// 这里改成你车轮的实际直径（单位：mm）
const float WHEEL_DIAMETER_MM = 65.0f;

// ================= 全局变量（供其他模块使用） =================
volatile long g_left_encoder_ticks  = 0;   // 左侧累计脉冲（带方向）
volatile long g_right_encoder_ticks = 0;   // 右侧累计脉冲（带方向）

long g_left_speed_raw  = 0;                // 左侧每20ms脉冲数（带方向）
long g_right_speed_raw = 0;                // 右侧每20ms脉冲数（带方向）
int  g_vehicle_speed   = 0;                // 平均每20ms脉冲数（带方向）——给驻车PD继续用

float g_left_speed_mm_s    = 0.0f;         // 左侧线速度 mm/s
float g_right_speed_mm_s   = 0.0f;         // 右侧线速度 mm/s
float g_vehicle_speed_mm_s = 0.0f;         // 平均线速度 mm/s

float g_left_rpm  = 0.0f;                  // 左侧轮轴转速 rpm
float g_right_rpm = 0.0f;                  // 右侧轮轴转速 rpm

// ================= 记录上一次电平，用于检测上升沿 =================
volatile uint8_t g_left_prev_a  = 0;
volatile uint8_t g_left_prev_b  = 0;
volatile uint8_t g_right_prev_a = 0;
volatile uint8_t g_right_prev_b = 0;

void setup_encoder_feedback(void) {
  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);

  g_left_prev_a  = digitalRead(ENC_LEFT_A) ? 1 : 0;
  g_left_prev_b  = digitalRead(ENC_LEFT_B) ? 1 : 0;
  g_right_prev_a = digitalRead(ENC_RIGHT_A) ? 1 : 0;
  g_right_prev_b = digitalRead(ENC_RIGHT_B) ? 1 : 0;

  // A1/A2/A4/A5 都在 PORTC，开启 Pin Change Interrupt
  PCICR  |= (1 << PCIE1);
  PCMSK1 |= (1 << PCINT9)   // A1
         |  (1 << PCINT10)  // A2
         |  (1 << PCINT12)  // A4
         |  (1 << PCINT13); // A5
}

ISR(PCINT1_vect) {
  uint8_t portc_now = PINC;

  uint8_t left_a  = (portc_now >> 1) & 0x01;  // A1
  uint8_t left_b  = (portc_now >> 2) & 0x01;  // A2
  uint8_t right_a = (portc_now >> 4) & 0x01;  // A4
  uint8_t right_b = (portc_now >> 5) & 0x01;  // A5

  // ===== 左侧：只在上升沿计数，另一相判方向 =====
  if (!g_left_prev_a && left_a) {
    // A 上升沿
    if (left_b) {
      g_left_encoder_ticks -= ENC_LEFT_SIGN;
    } else {
      g_left_encoder_ticks += ENC_LEFT_SIGN;
    }
  }

  if (!g_left_prev_b && left_b) {
    // B 上升沿
    if (left_a) {
      g_left_encoder_ticks += ENC_LEFT_SIGN;
    } else {
      g_left_encoder_ticks -= ENC_LEFT_SIGN;
    }
  }

  // ===== 右侧 =====
  if (!g_right_prev_a && right_a) {
    if (right_b) {
      g_right_encoder_ticks -= ENC_RIGHT_SIGN;
    } else {
      g_right_encoder_ticks += ENC_RIGHT_SIGN;
    }
  }

  if (!g_right_prev_b && right_b) {
    if (right_a) {
      g_right_encoder_ticks += ENC_RIGHT_SIGN;
    } else {
      g_right_encoder_ticks -= ENC_RIGHT_SIGN;
    }
  }

  g_left_prev_a  = left_a;
  g_left_prev_b  = left_b;
  g_right_prev_a = right_a;
  g_right_prev_b = right_b;
}

void loop_encoder_feedback(void) {
  static unsigned long last_update_ms = 0;
  static long last_left_ticks = 0;
  static long last_right_ticks = 0;

  unsigned long now = millis();
  if (now - last_update_ms < ENCODER_SPEED_UPDATE_MS) {
    return;
  }
  last_update_ms = now;

  long left_ticks_now;
  long right_ticks_now;

  noInterrupts();
  left_ticks_now  = g_left_encoder_ticks;
  right_ticks_now = g_right_encoder_ticks;
  interrupts();

  long dl = left_ticks_now  - last_left_ticks;
  long dr = right_ticks_now - last_right_ticks;

  last_left_ticks  = left_ticks_now;
  last_right_ticks = right_ticks_now;

  g_left_speed_raw  = dl;
  g_right_speed_raw = dr;

  // 这个变量保留给你现有的主动驻车 PD 继续使用
  int instant_vehicle_speed = (int)((dl + dr) / 2L);
  g_vehicle_speed = (g_vehicle_speed * 3 + instant_vehicle_speed) / 4;

  // ===== 按手册公式换算线速度 =====
  // v = PI * D * M * 50 / 780
  const float mm_per_pulse = PI * WHEEL_DIAMETER_MM / (float)ENCODER_PULSES_PER_REV;
  const float pulse_to_mm_s = mm_per_pulse * (1000.0f / ENCODER_SPEED_UPDATE_MS);

  g_left_speed_mm_s    = dl * pulse_to_mm_s;
  g_right_speed_mm_s   = dr * pulse_to_mm_s;
  g_vehicle_speed_mm_s = ((g_left_speed_mm_s + g_right_speed_mm_s) * 0.5f);

  // ===== 换算 rpm =====
  const float pulse_to_rpm = (1000.0f / ENCODER_SPEED_UPDATE_MS) * 60.0f / (float)ENCODER_PULSES_PER_REV;
  g_left_rpm  = dl * pulse_to_rpm;
  g_right_rpm = dr * pulse_to_rpm;
}