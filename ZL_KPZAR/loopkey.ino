const int MOTOR_SPEED_MIN = -800;
const int MOTOR_SPEED_MAX = 800;

const int JOYSTICK_DEADZONE = 8;
const int JOYSTICK_MAX_ABS = 128;

const int THROTTLE_MIN_EFFECTIVE = 100;
const int STRAFE_MIN_EFFECTIVE   = 100;
const int ROTATE_MIN_EFFECTIVE   = 220;

const int MOTOR_SLEW_ACCEL_STEP = 30;
const int MOTOR_SLEW_DECEL_STEP = 70;

const int THROTTLE_GAIN_NUM = 10;
const int THROTTLE_GAIN_DEN = 10;

const int STRAFE_GAIN_NUM = 10;
const int STRAFE_GAIN_DEN = 10;

const int ROTATE_GAIN_NUM = 15;
const int ROTATE_GAIN_DEN = 10;

const int ROTATE_PRIORITY_THRESHOLD    = 140;
const int TRANSLATION_IGNORE_THRESHOLD = 120;

// ================= 挡位设置 =================
// 0 = 低速挡，1 = 中速挡，2 = 高速挡
static int g_speed_gear = 1;

const int LOW_GEAR_PERCENT  = 20;
const int MID_GEAR_PERCENT  = 40;
const int HIGH_GEAR_PERCENT = 100;

// 右摇杆Y轴三挡逻辑
const int GEAR_HIGH_THRESHOLD = 55;
const int GEAR_LOW_THRESHOLD  = -55;

// 如果你发现右摇杆Y方向和挡位逻辑反了，就把 1 改成 -1
#define GEAR_AXIS_SIGN 1

// ================= 两块 ZMotor 的方向分离补偿 =================
// 000 / 001 = 前面那块 ZMotor
// 008 / 009 = 后面那块 ZMotor

// 前面那块 ZMotor：前进补偿
const int FRONT_ZMOTOR_FWD_GAIN_NUM = 14;
const int FRONT_ZMOTOR_FWD_GAIN_DEN = 10;

// 前面那块 ZMotor：后退补偿
const int FRONT_ZMOTOR_REV_GAIN_NUM = 14;
const int FRONT_ZMOTOR_REV_GAIN_DEN = 10;

// 后面那块 ZMotor：前进补偿
const int REAR_ZMOTOR_FWD_GAIN_NUM  = 10;
const int REAR_ZMOTOR_FWD_GAIN_DEN  = 10;

// 后面那块 ZMotor：后退补偿
const int REAR_ZMOTOR_REV_GAIN_NUM  = 11;
const int REAR_ZMOTOR_REV_GAIN_DEN  = 10;

// 方向符号（按你当前已经调通的状态保留）
#define THROTTLE_SIGN  1
#define ROTATE_SIGN    -1
#define STRAFE_SIGN    1

// ================= 驻车抗推参数 =================
// g_vehicle_speed 是“每20ms平均编码器增量”的滤波值
extern int g_vehicle_speed;

const int PARK_ENTER_CMD_DZ = 25;
const int PARK_EXIT_CMD_DZ = 45;
const unsigned long PARK_ENTER_HOLD_MS = 350;

const int PARK_SPEED_DEADZONE = 1;
const int PARK_HOLD_KP = 32;
const int PARK_MIN_HOLD_TORQUE = 70;
const int PARK_MAX_TORQUE = 320;
const unsigned long PARK_TORQUE_DERATE_DELAY_MS = 3500;
const int PARK_MAX_TORQUE_DERATED = 220;

const int PARK_SLEW_ACCEL_STEP = 80;
const int PARK_SLEW_DECEL_STEP = 120;

bool g_parking_mode_active = false;
static unsigned long g_park_idle_start_ms = 0;
static unsigned long g_park_mode_start_ms = 0;

static int current_lf = 0;
static int current_rf = 0;
static int current_lr = 0;
static int current_rr = 0;

static int loopkey_clamp_motor_speed(int speed) {
  if (speed < MOTOR_SPEED_MIN) return MOTOR_SPEED_MIN;
  if (speed > MOTOR_SPEED_MAX) return MOTOR_SPEED_MAX;
  return speed;
}

static int loopkey_apply_deadzone(int value) {
  if (abs(value) <= JOYSTICK_DEADZONE) return 0;
  return value;
}

static int loopkey_apply_gear_scale(int speed) {
  int percent;

  if (g_speed_gear == 0) {
    percent = LOW_GEAR_PERCENT;
  } else if (g_speed_gear == 2) {
    percent = HIGH_GEAR_PERCENT;
  } else {
    percent = MID_GEAR_PERCENT;
  }

  return ((long)speed * percent) / 100;
}

// 前面那块 ZMotor 的方向分离补偿
static int loopkey_apply_front_zmotor_scale(int speed) {
  if (speed >= 0) {
    return ((long)speed * FRONT_ZMOTOR_FWD_GAIN_NUM) / FRONT_ZMOTOR_FWD_GAIN_DEN;
  } else {
    return ((long)speed * FRONT_ZMOTOR_REV_GAIN_NUM) / FRONT_ZMOTOR_REV_GAIN_DEN;
  }
}

// 后面那块 ZMotor 的方向分离补偿
static int loopkey_apply_rear_zmotor_scale(int speed) {
  if (speed >= 0) {
    return ((long)speed * REAR_ZMOTOR_FWD_GAIN_NUM) / REAR_ZMOTOR_FWD_GAIN_DEN;
  } else {
    return ((long)speed * REAR_ZMOTOR_REV_GAIN_NUM) / REAR_ZMOTOR_REV_GAIN_DEN;
  }
}

static int loopkey_map_axis_to_speed_with_min(int axis, int min_effective) {
  int value = loopkey_apply_deadzone(axis);
  if (value == 0) return 0;

  int sign = value > 0 ? 1 : -1;
  int magnitude = abs(value) - JOYSTICK_DEADZONE;

  const int active_range = JOYSTICK_MAX_ABS - JOYSTICK_DEADZONE;
  if (magnitude > active_range) magnitude = active_range;

  long quadratic_value = (long)magnitude * (long)magnitude / active_range;
  long scaled = quadratic_value * MOTOR_SPEED_MAX / active_range;
  int speed = (int)scaled;

  if (speed != 0 && speed < min_effective) {
    speed = min_effective;
  }

  return sign * speed;
}

static int loopkey_approach_speed(int current, int target) {
  int delta = target - current;
  if (delta == 0) return current;

  int step = MOTOR_SLEW_DECEL_STEP;

  bool same_direction = false;
  if (current != 0 && target != 0) {
    same_direction = ((current > 0 && target > 0) || (current < 0 && target < 0));
  }

  if (same_direction && abs(target) > abs(current)) {
    step = MOTOR_SLEW_ACCEL_STEP;
  }

  if (delta > step)  return current + step;
  if (delta < -step) return current - step;
  return target;
}

static int loopkey_approach_speed_with_steps(int current, int target, int accel_step, int decel_step) {
  int delta = target - current;
  if (delta == 0) return current;

  int step = decel_step;

  bool same_direction = false;
  if (current != 0 && target != 0) {
    same_direction = ((current > 0 && target > 0) || (current < 0 && target < 0));
  }

  if (same_direction && abs(target) > abs(current)) {
    step = accel_step;
  }

  if (delta > step)  return current + step;
  if (delta < -step) return current - step;
  return target;
}

static bool loopkey_elapsed_ms(unsigned long start_ms, unsigned long duration_ms) {
  if (start_ms == 0) return false;
  return (unsigned long)(millis() - start_ms) >= duration_ms;
}

static void loopkey_update_gear_by_right_y(void) {
  int gear_axis = GEAR_AXIS_SIGN * loopkey_apply_deadzone(PS2_RIGHT_Y);

  if (gear_axis >= GEAR_HIGH_THRESHOLD) {
    g_speed_gear = 2;
  } else if (gear_axis <= GEAR_LOW_THRESHOLD) {
    g_speed_gear = 0;
  } else {
    g_speed_gear = 1;
  }
}

static bool loopkey_drive_command_near_zero(int left_x, int left_y, int right_x) {
  return (abs(left_x) < PARK_ENTER_CMD_DZ) &&
         (abs(left_y) < PARK_ENTER_CMD_DZ) &&
         (abs(right_x) < PARK_ENTER_CMD_DZ);
}

static bool loopkey_drive_command_active(int left_x, int left_y, int right_x) {
  return (abs(left_x) > PARK_EXIT_CMD_DZ) ||
         (abs(left_y) > PARK_EXIT_CMD_DZ) ||
         (abs(right_x) > PARK_EXIT_CMD_DZ);
}

static void loopkey_update_parking_mode(int left_x, int left_y, int right_x) {
  unsigned long now = millis();

  if (g_parking_mode_active) {
    if (loopkey_drive_command_active(left_x, left_y, right_x)) {
      g_parking_mode_active = false;
      g_park_idle_start_ms = 0;
    }
    return;
  }

  if (!loopkey_drive_command_near_zero(left_x, left_y, right_x)) {
    g_park_idle_start_ms = 0;
    return;
  }

  if (g_park_idle_start_ms == 0) {
    g_park_idle_start_ms = now;
    return;
  }

  if (loopkey_elapsed_ms(g_park_idle_start_ms, PARK_ENTER_HOLD_MS)) {
    g_parking_mode_active = true;
    g_park_mode_start_ms = now;
    g_park_idle_start_ms = 0;
  }
}

static int loopkey_compute_park_hold_torque(void) {
  int speed = g_vehicle_speed;
  if (abs(speed) <= PARK_SPEED_DEADZONE) {
    return 0;
  }

  int torque = -speed * PARK_HOLD_KP;
  int abs_torque = abs(torque);

  if (abs_torque < PARK_MIN_HOLD_TORQUE) {
    torque = speed > 0 ? -PARK_MIN_HOLD_TORQUE : PARK_MIN_HOLD_TORQUE;
  }

  int max_torque = PARK_MAX_TORQUE;
  if (loopkey_elapsed_ms(g_park_mode_start_ms, PARK_TORQUE_DERATE_DELAY_MS)) {
    max_torque = PARK_MAX_TORQUE_DERATED;
  }

  if (torque > max_torque) {
    torque = max_torque;
  } else if (torque < -max_torque) {
    torque = -max_torque;
  }

  return torque;
}

void loop_key(void) {
  int cmd_left_x = loopkey_apply_deadzone(PS2_LEFT_X);
  int cmd_left_y = loopkey_apply_deadzone(PS2_LEFT_Y);
  int cmd_right_x = loopkey_apply_deadzone(PS2_RIGHT_X);

  loopkey_update_parking_mode(cmd_left_x, cmd_left_y, cmd_right_x);

  // ===== 按右摇杆Y选择挡位 =====
  loopkey_update_gear_by_right_y();

  // 左摇杆Y：前后
  int throttle = THROTTLE_SIGN * loopkey_map_axis_to_speed_with_min(PS2_LEFT_Y, THROTTLE_MIN_EFFECTIVE);

  // 右摇杆X：原地旋转
  int rotate = ROTATE_SIGN * loopkey_map_axis_to_speed_with_min(PS2_RIGHT_X, ROTATE_MIN_EFFECTIVE);

  // 左摇杆X：左右平移
  int strafe = STRAFE_SIGN * loopkey_map_axis_to_speed_with_min(PS2_LEFT_X, STRAFE_MIN_EFFECTIVE);

  throttle = throttle * THROTTLE_GAIN_NUM / THROTTLE_GAIN_DEN;
  rotate   = rotate   * ROTATE_GAIN_NUM   / ROTATE_GAIN_DEN;
  strafe   = strafe   * STRAFE_GAIN_NUM   / STRAFE_GAIN_DEN;

  int target_lf, target_rf, target_lr, target_rr;

  // ===== 旋转优先模式 =====
  if (abs(rotate) >= ROTATE_PRIORITY_THRESHOLD &&
      abs(throttle) < TRANSLATION_IGNORE_THRESHOLD &&
      abs(strafe) < TRANSLATION_IGNORE_THRESHOLD) {

    // 保留你已经验证可用的旋转方向
    target_lf = loopkey_clamp_motor_speed(-rotate);
    target_rf = loopkey_clamp_motor_speed(-rotate);
    target_lr = loopkey_clamp_motor_speed( rotate);
    target_rr = loopkey_clamp_motor_speed( rotate);

  } else {
    // 正常混控
    target_lf = loopkey_clamp_motor_speed(throttle + strafe + rotate);
    target_rf = loopkey_clamp_motor_speed(throttle - strafe - rotate);
    target_lr = loopkey_clamp_motor_speed(throttle - strafe + rotate);
    target_rr = loopkey_clamp_motor_speed(throttle + strafe - rotate);
  }

  // ===== 挡位缩放 =====
  target_lf = loopkey_clamp_motor_speed(loopkey_apply_gear_scale(target_lf));
  target_rf = loopkey_clamp_motor_speed(loopkey_apply_gear_scale(target_rf));
  target_lr = loopkey_clamp_motor_speed(loopkey_apply_gear_scale(target_lr));
  target_rr = loopkey_clamp_motor_speed(loopkey_apply_gear_scale(target_rr));

  // ===== ZMotor 分组补偿（按方向分开） =====
  // 前面那块 ZMotor：000 / 001
  target_lf = loopkey_clamp_motor_speed(loopkey_apply_front_zmotor_scale(target_lf));
  target_rf = loopkey_clamp_motor_speed(loopkey_apply_front_zmotor_scale(target_rf));

  // 后面那块 ZMotor：008 / 009
  target_lr = loopkey_clamp_motor_speed(loopkey_apply_rear_zmotor_scale(target_lr));
  target_rr = loopkey_clamp_motor_speed(loopkey_apply_rear_zmotor_scale(target_rr));

  // ===== 驻车抗推：保持零速，覆盖常规摇杆混控输出 =====
  if (g_parking_mode_active) {
    int park_torque = loopkey_compute_park_hold_torque();
    target_lf = loopkey_clamp_motor_speed(park_torque);
    target_rf = loopkey_clamp_motor_speed(park_torque);
    target_lr = loopkey_clamp_motor_speed(park_torque);
    target_rr = loopkey_clamp_motor_speed(park_torque);
  }

  // ===== 平滑过渡 =====
  if (g_parking_mode_active) {
    current_lf = loopkey_approach_speed_with_steps(current_lf, target_lf, PARK_SLEW_ACCEL_STEP, PARK_SLEW_DECEL_STEP);
    current_rf = loopkey_approach_speed_with_steps(current_rf, target_rf, PARK_SLEW_ACCEL_STEP, PARK_SLEW_DECEL_STEP);
    current_lr = loopkey_approach_speed_with_steps(current_lr, target_lr, PARK_SLEW_ACCEL_STEP, PARK_SLEW_DECEL_STEP);
    current_rr = loopkey_approach_speed_with_steps(current_rr, target_rr, PARK_SLEW_ACCEL_STEP, PARK_SLEW_DECEL_STEP);
  } else {
    current_lf = loopkey_approach_speed(current_lf, target_lf);
    current_rf = loopkey_approach_speed(current_rf, target_rf);
    current_lr = loopkey_approach_speed(current_lr, target_lr);
    current_rr = loopkey_approach_speed(current_rr, target_rr);
  }

  // ===== 输出到四个轮子 =====
  motor_set_wheels(current_lf, current_rf, current_lr, current_rr);

  delay(10);
}
