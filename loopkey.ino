void loop_key(void) {
  int left_key_x = PS2_LEFT_X;
  int left_key_y = PS2_LEFT_Y;
  // 情况一：在原点，重置电机
  if (left_key_x == origin_left_x && left_key_y == origin_left_y) {
    setup_motor();
    return;
  }
  // 情况二：纵向为主（|x| < |y|），直行加减速
  if (std::abs(left_key_x) < std::abs(left_key_y)) {
    int extent_of_accelerate = std::abs(left_key_y) / 10;
    // 向上推杆：前进加速
    if (left_key_y > 0) {
      motor1_speed += extent_of_accelerate;
      if (motor1_speed > 1000) motor1_speed = 1000;
      motor1_SetSpeed(motor1_speed);
      motor2_speed += extent_of_accelerate;
      if (motor2_speed > 1000) motor2_speed = 1000;
      motor2_SetSpeed(motor2_speed);
    }
    // 向下推杆：后退加速
    if (left_key_y < 0) {
      motor1_speed -= extent_of_accelerate;
      if (motor1_speed < -1000) motor1_speed = -1000;
      motor1_SetSpeed(motor1_speed);
      motor2_speed -= extent_of_accelerate;
      if (motor2_speed < -1000) motor2_speed = -1000;
      motor2_SetSpeed(motor2_speed);
    }
  }
  // 情况三：横向为主（|x| >= |y|），转向
  if (std::abs(left_key_x) >= std::abs(left_key_y)) {
    int extent_of_turn = std::abs(left_key_x) / 10;
    // 向左推杆：左转（motor1减速，motor2加速）
    if (left_key_x < 0) {
      motor1_speed -= extent_of_turn;
      if (motor1_speed < -1000) motor1_speed = -1000;
      motor1_SetSpeed(motor1_speed);
      motor2_speed += extent_of_turn;
      if (motor2_speed > 1000) motor2_speed = 1000;
      motor2_SetSpeed(motor2_speed);
    }
    // 向右推杆：右转（motor1加速，motor2减速）
    if (left_key_x > 0) {
      motor1_speed += extent_of_turn;
      if (motor1_speed > 1000) motor1_speed = 1000;
      motor1_SetSpeed(motor1_speed);
      motor2_speed -= extent_of_turn;
      if (motor2_speed < -1000) motor2_speed = -1000;
      motor2_SetSpeed(motor2_speed);
    }
  }
}