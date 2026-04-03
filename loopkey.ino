const int MOTOR_SPEED_MIN = -1000;
const int MOTOR_SPEED_MAX = 1000;
const int JOYSTICK_DEADZONE = 5;
const int JOYSTICK_SCALE_DIV = 10;

int clamp_motor_speed(int speed) {
  if (speed < MOTOR_SPEED_MIN) return MOTOR_SPEED_MIN;
  if (speed > MOTOR_SPEED_MAX) return MOTOR_SPEED_MAX;
  return speed;
}

void loop_key(void) {
  int left_key_x = PS2_LEFT_X;
  int left_key_y = PS2_LEFT_Y;
  if (std::abs(left_key_x) <= JOYSTICK_DEADZONE) left_key_x = 0;
  if (std::abs(left_key_y) <= JOYSTICK_DEADZONE) left_key_y = 0;
  // 情况一：在原点，重置电机
  if (left_key_x == 0 && left_key_y == 0) {
    setup_motor();
    return;
  }
  // 情况二：纵向为主（|x| < |y|），直行加减速
  if (std::abs(left_key_x) < std::abs(left_key_y)) {
    int extent_of_accelerate = std::abs(left_key_y) / JOYSTICK_SCALE_DIV;
    // 向上推杆：前进加速
    if (left_key_y > 0) {
      motor1_speed += extent_of_accelerate;
      motor1_speed = clamp_motor_speed(motor1_speed);
      motor1_SetSpeed(motor1_speed);
      motor2_speed += extent_of_accelerate;
      motor2_speed = clamp_motor_speed(motor2_speed);
      motor2_SetSpeed(motor2_speed);
    }
    // 向下推杆：后退加速
    if (left_key_y < 0) {
      motor1_speed -= extent_of_accelerate;
      motor1_speed = clamp_motor_speed(motor1_speed);
      motor1_SetSpeed(motor1_speed);
      motor2_speed -= extent_of_accelerate;
      motor2_speed = clamp_motor_speed(motor2_speed);
      motor2_SetSpeed(motor2_speed);
    }
  }
  // 情况三：横向为主（|x| >= |y|），转向
  if (std::abs(left_key_x) >= std::abs(left_key_y)) {
    int extent_of_turn = std::abs(left_key_x) / JOYSTICK_SCALE_DIV;
    // 向左推杆：左转（motor1减速，motor2加速）
    if (left_key_x < 0) {
      motor1_speed -= extent_of_turn;
      motor1_speed = clamp_motor_speed(motor1_speed);
      motor1_SetSpeed(motor1_speed);
      motor2_speed += extent_of_turn;
      motor2_speed = clamp_motor_speed(motor2_speed);
      motor2_SetSpeed(motor2_speed);
    }
    // 向右推杆：右转（motor1加速，motor2减速）
    if (left_key_x > 0) {
      motor1_speed += extent_of_turn;
      motor1_speed = clamp_motor_speed(motor1_speed);
      motor1_SetSpeed(motor1_speed);
      motor2_speed -= extent_of_turn;
      motor2_speed = clamp_motor_speed(motor2_speed);
      motor2_SetSpeed(motor2_speed);
    }
  }
}
