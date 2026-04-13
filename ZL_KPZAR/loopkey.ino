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

// ================= 主动抱死（驻车）参数 =================
// 编码器反馈变量
extern int g_vehicle_speed;
extern volatile long g_left_encoder_ticks;
extern volatile long g_right_encoder_ticks;

// 摇杆所有轴都在这个范围内才算”松杆”
const int BRAKE_CMD_DEADZONE = 80;

// PD 驻车控制参数（编码器脉冲 → 电机速度指令）
//   KP 越大 → 位置保持越硬（偏移恢复越快），但太大会振荡
//   KD 越大 → 阻尼越强（减速越猛），抑制振荡
//   MAX_TORQUE 限制最大输出，防止电机过载
const int BRAKE_POSITION_KP = 50;
const int BRAKE_SPEED_KD    = 30;
const int BRAKE_MAX_TORQUE  = 500;

static bool g_brake_holding    = false;
static long g_brake_hold_left  = 0;
static long g_brake_hold_right = 0;

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


void loop_key(void) {
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

  // ===== 主动抱死：松杆时用 PD 控制锁住当前位置 =====
  bool joystick_released =
      (abs(throttle) < BRAKE_CMD_DEADZONE) &&
      (abs(strafe)   < BRAKE_CMD_DEADZONE) &&
      (abs(rotate)   < BRAKE_CMD_DEADZONE);

  if (joystick_released) {
    if (!g_brake_holding) {
      // 刚松杆：记录当前编码器位置作为锁定点
      noInterrupts();
      g_brake_hold_left  = g_left_encoder_ticks;
      g_brake_hold_right = g_right_encoder_ticks;
      interrupts();
      g_brake_holding = true;
    }

    // 读取当前编码器位置
    long left_now, right_now;
    noInterrupts();
    left_now  = g_left_encoder_ticks;
    right_now = g_right_encoder_ticks;
    interrupts();

    // 位置误差（正 = 相对锁定点向前偏移了）
    long pos_err = ((left_now - g_brake_hold_left)
                  + (right_now - g_brake_hold_right)) / 2;

    // PD 控制：扭矩 = -Kp × 位置误差 - Kd × 当前速度
    //   位置偏了 → P 项把车拉回来
    //   还在动   → D 项提供阻尼，防止振荡
    int torque = (int)(-(long)BRAKE_POSITION_KP * pos_err
                       - (long)BRAKE_SPEED_KD * (long)g_vehicle_speed);

    if (torque >  BRAKE_MAX_TORQUE) torque =  BRAKE_MAX_TORQUE;
    if (torque < -BRAKE_MAX_TORQUE) torque = -BRAKE_MAX_TORQUE;

    target_lf = loopkey_clamp_motor_speed(torque);
    target_rf = loopkey_clamp_motor_speed(torque);
    target_lr = loopkey_clamp_motor_speed(torque);
    target_rr = loopkey_clamp_motor_speed(torque);
  } else {
    g_brake_holding = false;
  }

  // ===== 平滑过渡 =====
  current_lf = loopkey_approach_speed(current_lf, target_lf);
  current_rf = loopkey_approach_speed(current_rf, target_rf);
  current_lr = loopkey_approach_speed(current_lr, target_lr);
  current_rr = loopkey_approach_speed(current_rr, target_rr);

  // ===== 输出到四个轮子 =====
  motor_set_wheels(current_lf, current_rf, current_lr, current_rr);

  delay(10);
}
