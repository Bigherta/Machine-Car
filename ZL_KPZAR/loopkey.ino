const int MOTOR_SPEED_MIN = -1000;
const int MOTOR_SPEED_MAX = 1000;
const int JOYSTICK_DEADZONE = 5;
const int JOYSTICK_MAX_ABS = 128;
const int MOTOR_SPEED_MIN_EFFECTIVE = 80;
const int MOTOR_SLEW_ACCEL_STEP = 20;
const int MOTOR_SLEW_DECEL_STEP = 50;
const int STEERING_GAIN_NUM = 8;
const int STEERING_GAIN_DEN = 10;
#define ENABLE_LOOPKEY_DEBUG_PRINT 0

static int clamp_motor_speed(int speed) {
  if (speed < MOTOR_SPEED_MIN) return MOTOR_SPEED_MIN;
  if (speed > MOTOR_SPEED_MAX) return MOTOR_SPEED_MAX;
  return speed;
}

static int apply_deadzone(int value) {
  if (abs(value) <= JOYSTICK_DEADZONE) return 0;
  return value;
}

static int map_axis_to_speed(int axis) {
  int value = apply_deadzone(axis);
  if (value == 0) return 0;

  int sign = value > 0 ? 1 : -1;
  int magnitude = abs(value) - JOYSTICK_DEADZONE;
  const int active_range = JOYSTICK_MAX_ABS - JOYSTICK_DEADZONE;
  if (magnitude > active_range) magnitude = active_range;

  // 二次曲线：偏移越大，增幅越大
  long quadratic_value = (long)magnitude * (long)magnitude / active_range;
  long scaled = quadratic_value * MOTOR_SPEED_MAX / active_range;
  int speed = (int)scaled;

  // speed 在此处始终为非负量，方向在返回时由 sign 统一施加
  if (speed != 0 && speed < MOTOR_SPEED_MIN_EFFECTIVE) {
    speed = MOTOR_SPEED_MIN_EFFECTIVE;
  }
  return sign * speed;
}

static bool is_same_nonzero_direction(int current, int target) {
  if (target == 0 || current == 0) return false;
  return (current > 0 && target > 0) || (current < 0 && target < 0);
}

static int approach_speed(int current, int target) {
  int delta = target - current;
  if (delta == 0) return current;

  int step = MOTOR_SLEW_DECEL_STEP;
  bool same_direction = is_same_nonzero_direction(current, target);
  if (same_direction && abs(target) > abs(current)) {
    step = MOTOR_SLEW_ACCEL_STEP;
  }

  if (delta > step) return current + step;
  if (delta < -step) return current - step;
  return target;
}

void loop_key(void) {
  int throttle = map_axis_to_speed(PS2_LEFT_Y);
  int steering = map_axis_to_speed(PS2_LEFT_X);
  steering = steering * STEERING_GAIN_NUM / STEERING_GAIN_DEN;

  int target_motor1_speed = clamp_motor_speed(throttle + steering);
  int target_motor2_speed = clamp_motor_speed(throttle - steering);

  motor1_speed = approach_speed(motor1_speed, target_motor1_speed);
  motor2_speed = approach_speed(motor2_speed, target_motor2_speed);
  motor1_SetSpeed(motor1_speed);
  motor2_SetSpeed(motor2_speed);

#if ENABLE_LOOPKEY_DEBUG_PRINT
    Serial.print("LX=");
    Serial.print(PS2_LEFT_X);
    Serial.print("  LY=");
    Serial.print(PS2_LEFT_Y);
    Serial.print("  M1=");
    Serial.print(motor1_speed);
    Serial.print("  M2=");
    Serial.println(motor2_speed);
#endif
}
