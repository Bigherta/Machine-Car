const int MOTOR_SLEW_ACCEL_STEP = 20;
const int MOTOR_SLEW_DECEL_STEP = 50;
const int DIRECT_MOTOR_TEST_SPEED = 120;
const unsigned long DIRECT_MOTOR_TEST_STAGE_MS = 2000;

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
  if (KEY2 == LOW) {
    motor1_speed = 0;
    motor2_speed = 0;
    motor1_SetSpeed(0);
    motor2_SetSpeed(0);
    return;
  }

  unsigned long motor_test_phase = (millis() / DIRECT_MOTOR_TEST_STAGE_MS) % 4;
  int target_motor1_speed = 0;
  int target_motor2_speed = 0;
  if (motor_test_phase == 0) {
    target_motor1_speed = DIRECT_MOTOR_TEST_SPEED;
    target_motor2_speed = DIRECT_MOTOR_TEST_SPEED;
  } else if (motor_test_phase == 2) {
    target_motor1_speed = -DIRECT_MOTOR_TEST_SPEED;
    target_motor2_speed = -DIRECT_MOTOR_TEST_SPEED;
  }

  motor1_speed = approach_speed(motor1_speed, target_motor1_speed);
  motor2_speed = approach_speed(motor2_speed, target_motor2_speed);
  motor1_SetSpeed(motor1_speed);
  motor2_SetSpeed(motor2_speed);
}
