/****************************************************************************
  编码器检测工具函数（A/B 相位）
  用法：
  1) 先让电机以固定方向转动；
  2) 调用 encoder_check_encode_decode(pinA, pinB, 采样时长, 最小跳变数)；
  3) 若返回 true，说明在当前方向下具备有效编码与解码能力。
  建议再反向转动重复一次，确认双向都正常。
****************************************************************************/

const unsigned long ENCODER_SAMPLE_INTERVAL_US = 100;
const unsigned long ENCODER_VALID_STEPS_PER_INVALID_STEP = 2;  // 每 1 次非法跳变至少需要 2 次有效跳变

// 运行期监控配置：请按实际接线修改为编码器 A/B 引脚。
// 设为 -1 表示禁用串口输出。
const int ENCODER_MONITOR_PIN_A = -1;
const int ENCODER_MONITOR_PIN_B = -1;
const unsigned long ENCODER_MONITOR_SAMPLE_MS = 30;
const unsigned long ENCODER_MONITOR_PRINT_INTERVAL_MS = 200;

static int encoder_read_state(uint8_t pin_a, uint8_t pin_b) {
  return (digitalRead(pin_a) << 1) | digitalRead(pin_b);
}

// 返回 +1/-1/0（0 表示非法跳变或无跳变）
static int encoder_quadrature_delta(int prev_state, int curr_state) {
  int key = (prev_state << 2) | curr_state;
  switch (key) {
    case 0b0001:
    case 0b0111:
    case 0b1110:
    case 0b1000:
      return 1;
    case 0b0010:
    case 0b1011:
    case 0b1101:
    case 0b0100:
      return -1;
    default:
      return 0;
  }
}

static void encoder_detect_sample(uint8_t pin_a, uint8_t pin_b,
                                  unsigned long sample_ms,
                                  long &ticks,
                                  unsigned long &transitions,
                                  unsigned long &valid_steps,
                                  unsigned long &invalid_steps,
                                  int &direction,
                                  bool &direction_stable) {
  ticks = 0;
  transitions = 0;
  valid_steps = 0;
  invalid_steps = 0;
  direction = 0;
  direction_stable = false;

  pinMode(pin_a, INPUT_PULLUP);
  pinMode(pin_b, INPUT_PULLUP);

  int prev_state = encoder_read_state(pin_a, pin_b);
  unsigned long t0 = millis();
  long positive_steps = 0;
  long negative_steps = 0;

  while (millis() - t0 < sample_ms) {
    int curr_state = encoder_read_state(pin_a, pin_b);
    if (curr_state == prev_state) {
      delayMicroseconds(ENCODER_SAMPLE_INTERVAL_US);
      continue;
    }

    transitions++;
    int d = encoder_quadrature_delta(prev_state, curr_state);
    if (d > 0) {
      valid_steps++;
      ticks++;
      positive_steps++;
    } else if (d < 0) {
      valid_steps++;
      ticks--;
      negative_steps++;
    } else {
      invalid_steps++;
    }
    prev_state = curr_state;
    delayMicroseconds(ENCODER_SAMPLE_INTERVAL_US);
  }

  if (positive_steps > 0 && negative_steps == 0) {
    direction = 1;
    direction_stable = true;
  } else if (negative_steps > 0 && positive_steps == 0) {
    direction = -1;
    direction_stable = true;
  } else {
    direction = 0;
    direction_stable = false;
  }
}

// 综合判定：是否具备“编码 + 解码”能力（单次采样）
bool encoder_check_encode_decode(uint8_t pin_a, uint8_t pin_b,
                                 unsigned long sample_ms = 500,
                                 unsigned long min_transitions = 6) {
  long ticks;
  unsigned long transitions;
  unsigned long valid_steps;
  unsigned long invalid_steps;
  int direction;
  bool direction_stable;

  encoder_detect_sample(pin_a, pin_b, sample_ms, ticks, transitions,
                        valid_steps, invalid_steps, direction,
                        direction_stable);

  bool has_signal = (transitions > 0);
  bool decode_ok = (valid_steps > 0) &&
                   (valid_steps >=
                    (invalid_steps * ENCODER_VALID_STEPS_PER_INVALID_STEP));
  bool result = has_signal && decode_ok && direction_stable &&
                (transitions >= min_transitions);

  if (result) {
    Serial.print("encoder_check_encode_decode=true");
  }
  else Serial.print("encoder_check_encode_decode=false");
  return result;
}

void encoder_print_during_drive(int throttle_cmd) {
  if (throttle_cmd == 0) return;
  if (ENCODER_MONITOR_PIN_A < 0 || ENCODER_MONITOR_PIN_B < 0) return;

  static unsigned long last_print_ms = 0;
  unsigned long now = millis();
  if (now - last_print_ms < ENCODER_MONITOR_PRINT_INTERVAL_MS) return;
  last_print_ms = now;

  long ticks = 0;
  unsigned long transitions = 0;
  unsigned long valid_steps = 0;
  unsigned long invalid_steps = 0;
  int direction = 0;
  bool direction_stable = false;

  encoder_detect_sample((uint8_t)ENCODER_MONITOR_PIN_A,
                        (uint8_t)ENCODER_MONITOR_PIN_B,
                        ENCODER_MONITOR_SAMPLE_MS,
                        ticks, transitions, valid_steps, invalid_steps,
                        direction, direction_stable);

  Serial.print("ENC throttle=");
  Serial.print(throttle_cmd);
  Serial.print(", ticks=");
  Serial.print(ticks);
  Serial.print(", trans=");
  Serial.print(transitions);
  Serial.print(", valid=");
  Serial.print(valid_steps);
  Serial.print(", invalid=");
  Serial.print(invalid_steps);
  Serial.print(", dir=");
  Serial.print(direction);
  Serial.print(", stable=");
  Serial.println(direction_stable ? 1 : 0);
}
