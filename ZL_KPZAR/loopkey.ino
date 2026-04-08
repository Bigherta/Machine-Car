const int MOTOR_SPEED_MIN = -1000;
const int MOTOR_SPEED_MAX = 1000;
const int JOYSTICK_DEADZONE = 5;
const int JOYSTICK_MAX_ABS = 128;
const int MOTOR_SPEED_MIN_EFFECTIVE = 80;
const int MOTOR_SLEW_ACCEL_STEP = 20;
const int MOTOR_SLEW_DECEL_STEP = 50;
const int VY_GAIN_NUM = 10;
const int VY_GAIN_DEN = 10;
const int WZ_GAIN_NUM = 8;
const int WZ_GAIN_DEN = 10;

#define VX_SIGN -1  // 若前推后退，改成 1
#define VY_SIGN -1  // 若左推右移方向不对，改成 -1
#define WZ_SIGN 1   // 若右推自旋方向不对，改成 -1
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

  long quadratic_value = (long)magnitude * (long)magnitude / active_range;
  long scaled = quadratic_value * MOTOR_SPEED_MAX / active_range;
  int speed = (int)scaled;

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

static void normalize_mecanum_targets(int *fl, int *fr, int *rl, int *rr) {
  int abs_fl = abs(*fl);
  int abs_fr = abs(*fr);
  int abs_rl = abs(*rl);
  int abs_rr = abs(*rr);

  int max_abs_target = abs_fl;
  if (abs_fr > max_abs_target) max_abs_target = abs_fr;
  if (abs_rl > max_abs_target) max_abs_target = abs_rl;
  if (abs_rr > max_abs_target) max_abs_target = abs_rr;

  if (max_abs_target == 0) return;
  if (max_abs_target <= MOTOR_SPEED_MAX) return;

  *fl = ((long)(*fl) * MOTOR_SPEED_MAX) / max_abs_target;
  *fr = ((long)(*fr) * MOTOR_SPEED_MAX) / max_abs_target;
  *rl = ((long)(*rl) * MOTOR_SPEED_MAX) / max_abs_target;
  *rr = ((long)(*rr) * MOTOR_SPEED_MAX) / max_abs_target;
}

void loop_key(void) {
  static int wheel_fl_speed = 0;
  static int wheel_fr_speed = 0;
  static int wheel_rl_speed = 0;
  static int wheel_rr_speed = 0;

  int vx = VX_SIGN * map_axis_to_speed(PS2_LEFT_Y);
  int vy = VY_SIGN * map_axis_to_speed(PS2_LEFT_X);
  int wz = WZ_SIGN * map_axis_to_speed(PS2_RIGHT_X);

  vy = vy * VY_GAIN_NUM / VY_GAIN_DEN;
  wz = wz * WZ_GAIN_NUM / WZ_GAIN_DEN;

  // 标准麦克纳姆混控（fl/fr/rl/rr = 前左/前右/后左/后右）：
  // vx 前后，vy 横移，wz 自旋
  int target_fl_speed = vx + vy + wz;
  int target_fr_speed = vx - vy - wz;
  int target_rl_speed = vx - vy + wz;
  int target_rr_speed = vx + vy - wz;

  normalize_mecanum_targets(&target_fl_speed, &target_fr_speed, &target_rl_speed, &target_rr_speed);

  target_fl_speed = clamp_motor_speed(target_fl_speed);
  target_fr_speed = clamp_motor_speed(target_fr_speed);
  target_rl_speed = clamp_motor_speed(target_rl_speed);
  target_rr_speed = clamp_motor_speed(target_rr_speed);

  wheel_fl_speed = approach_speed(wheel_fl_speed, target_fl_speed);
  wheel_fr_speed = approach_speed(wheel_fr_speed, target_fr_speed);
  wheel_rl_speed = approach_speed(wheel_rl_speed, target_rl_speed);
  wheel_rr_speed = approach_speed(wheel_rr_speed, target_rr_speed);

  motor4_SetSpeed(wheel_fl_speed, wheel_fr_speed, wheel_rl_speed, wheel_rr_speed);
}