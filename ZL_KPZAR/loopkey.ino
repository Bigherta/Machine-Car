const int MOTOR_SPEED_MIN = -300;
const int MOTOR_SPEED_MAX = 300;   // 高速挡的基础最大速度，可改 500 / 700

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
static int g_speed_gear = 1;   // 初始中速挡

// 各挡位速度百分比
const int LOW_GEAR_PERCENT  = 35;
const int MID_GEAR_PERCENT  = 65;
const int HIGH_GEAR_PERCENT = 100;

// 右摇杆Y轴阈值（注意：这里使用的是 PS2_RIGHT_Y 偏移值，中心是 0）
// 大于这个值 -> 升高一档（最多到 2）
// 小于负这个值 -> 降低一档（最低到 0）
// 中间区域 -> 解锁下一次换挡（防止持续推杆连跳）
const int GEAR_HIGH_THRESHOLD = 55;
const int GEAR_LOW_THRESHOLD  = -55;

// 如果你发现“向上推右摇杆”实际进了低速挡，
// 就把这个 1 改成 -1
#define GEAR_AXIS_SIGN 1

// 方向符号（保留你当前已经调通的状态）
#define THROTTLE_SIGN  -1
#define ROTATE_SIGN    -1
#define STRAFE_SIGN    -1

static int current_lf = 0;
static int current_rf = 0;
static int current_lr = 0;
static int current_rr = 0;
static bool g_gear_shift_armed = true;

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

  // 回到中间区后解锁一次换挡
  if (gear_axis < GEAR_HIGH_THRESHOLD && gear_axis > GEAR_LOW_THRESHOLD) {
    g_gear_shift_armed = true;
    return;
  }

  // 未解锁时忽略，避免持续推杆连跳
  if (!g_gear_shift_armed) return;

  if (gear_axis >= GEAR_HIGH_THRESHOLD && g_speed_gear < 2) {
    g_speed_gear++;      // 升高一档
    g_gear_shift_armed = false;
  } else if (gear_axis <= GEAR_LOW_THRESHOLD && g_speed_gear > 0) {
    g_speed_gear--;      // 降低一档
    g_gear_shift_armed = false;
  }
}

void loop_key(void) {
  // ===== 根据右摇杆Y轴位置决定挡位 =====
  loopkey_update_gear_by_right_y();

  // ===== 三个运动分量 =====
  // 左摇杆Y：前后
  int throttle = THROTTLE_SIGN * loopkey_map_axis_to_speed_with_min(PS2_LEFT_Y, THROTTLE_MIN_EFFECTIVE);

  // 右摇杆X：原地旋转
  int rotate = ROTATE_SIGN * loopkey_map_axis_to_speed_with_min(PS2_RIGHT_X, ROTATE_MIN_EFFECTIVE);

  // 左摇杆X：左右平移
  int strafe = STRAFE_SIGN * loopkey_map_axis_to_speed_with_min(PS2_LEFT_X, STRAFE_MIN_EFFECTIVE);

  // ===== 增益 =====
  throttle = throttle * THROTTLE_GAIN_NUM / THROTTLE_GAIN_DEN;
  rotate   = rotate   * ROTATE_GAIN_NUM   / ROTATE_GAIN_DEN;
  strafe   = strafe   * STRAFE_GAIN_NUM   / STRAFE_GAIN_DEN;

  int target_lf, target_rf, target_lr, target_rr;

  // ===== 旋转优先模式 =====
  if (abs(rotate) >= ROTATE_PRIORITY_THRESHOLD &&
      abs(throttle) < TRANSLATION_IGNORE_THRESHOLD &&
      abs(strafe) < TRANSLATION_IGNORE_THRESHOLD) {

    // 保留你当前已经验证可用的旋转方向
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

  // ===== 平滑过渡 =====
  current_lf = loopkey_approach_speed(current_lf, target_lf);
  current_rf = loopkey_approach_speed(current_rf, target_rf);
  current_lr = loopkey_approach_speed(current_lr, target_lr);
  current_rr = loopkey_approach_speed(current_rr, target_rr);

  // ===== 输出到四个轮子 =====
  motor_set_wheels(current_lf, current_rf, current_lr, current_rr);

  delay(10);
}
