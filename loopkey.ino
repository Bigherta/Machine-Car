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
    // 向上推杆：前进
    if (left_key_y > 0) {
      if (press_time_up == 0) {
        press_time_up = millis();
      }

      if (!long_press_active_up && (millis() - press_time_up >= 500)) {
        long_press_active_up = true;
        // 长按执行代码（如持续加速）
      }
    } else if (press_time_up > 0) {
      if (!long_press_active_up && (millis() - press_time_up < 500)) {
        int extent_of_accelerate = std::abs(left_key_y) / 10;
        motor1_speed += extent_of_accelerate;
        if (motor1_speed > 1000) motor1_speed = 1000;
        motor1_SetSpeed(motor1_speed);

        motor2_speed += extent_of_accelerate;
        if (motor2_speed > 1000) motor2_speed = 1000;
        motor2_SetSpeed(motor2_speed);
      }

      press_time_up = 0;
      long_press_active_up = false;
    }

    // 向下推杆：后退
    if (left_key_y < 0) {
      if (press_time_down == 0) {
        press_time_down = millis();
      }

      if (!long_press_active_down && (millis() - press_time_down >= 500)) {
        long_press_active_down = true;
        // 长按执行代码
      }
    } else if (press_time_down > 0) {
      if (!long_press_active_down && (millis() - press_time_down < 500)) {
        int extent_of_decelerate = std::abs(left_key_y) / 10;
        motor1_speed -= extent_of_decelerate;
        if (motor1_speed < -1000) motor1_speed = -1000;
        motor1_SetSpeed(motor1_speed);

        motor2_speed -= extent_of_decelerate;
        if (motor2_speed < -1000) motor2_speed = -1000;
        motor2_SetSpeed(motor2_speed);
      }

      press_time_down = 0;
      long_press_active_down = false;
    }
  }

  // 情况三：横向为主（|x| >= |y|），转向
  if (std::abs(left_key_x) >= std::abs(left_key_y)) {
    int extent_of_turn = std::abs(left_key_x) / 10;

    // 向左推杆：左转（motor1减速，motor2加速）
    if (left_key_x < 0) {
      if (press_time_left == 0) {
        press_time_left = millis();
      }

      if (!long_press_active_left && (millis() - press_time_left >= 500)) {
        long_press_active_left = true;
        // 长按执行代码
      }
    } else if (press_time_left > 0) {
      if (!long_press_active_left && (millis() - press_time_left < 500)) {
        motor1_speed -= extent_of_turn;
        if (motor1_speed < -1000) motor1_speed = -1000;
        motor1_SetSpeed(motor1_speed);

        motor2_speed += extent_of_turn;
        if (motor2_speed > 1000) motor2_speed = 1000;
        motor2_SetSpeed(motor2_speed);
      }

      press_time_left = 0;
      long_press_active_left = false;
    }

    // 向右推杆：右转（motor1加速，motor2减速）
    if (left_key_x > 0) {
      if (press_time_right == 0) {
        press_time_right = millis();
      }

      if (!long_press_active_right && (millis() - press_time_right >= 500)) {
        long_press_active_right = true;
        // 长按执行代码
      }
    } else if (press_time_right > 0) {
      if (!long_press_active_right && (millis() - press_time_right < 500)) {
        motor1_speed += extent_of_turn;
        if (motor1_speed > 1000) motor1_speed = 1000;
        motor1_SetSpeed(motor1_speed);

        motor2_speed -= extent_of_turn;
        if (motor2_speed < -1000) motor2_speed = -1000;
        motor2_SetSpeed(motor2_speed);
      }

      press_time_right = 0;
      long_press_active_right = false;
    }
  }
}