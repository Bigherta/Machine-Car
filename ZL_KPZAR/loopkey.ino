extern bool g_ps2_link_ok;
extern unsigned long g_ps2_last_read_ms;
extern bool g_ps2_frame_ready;


#ifndef MOTOR_DRIVER_RX_TEST_MODE
#define MOTOR_DRIVER_RX_TEST_MODE 0
#endif
#ifndef MOTOR_DRIVER_RX_TEST_PROFILE
#define MOTOR_DRIVER_RX_TEST_PROFILE 1
#endif
#ifndef MOTOR_DRIVER_RX_TEST_SPEED
#define MOTOR_DRIVER_RX_TEST_SPEED 60
#endif
#ifndef MOTOR_DRIVER_RX_TEST_PWM
#define MOTOR_DRIVER_RX_TEST_PWM 80
#endif
#ifndef MOTOR_DRIVER_RX_TEST_SEND_INTERVAL_MS
#define MOTOR_DRIVER_RX_TEST_SEND_INTERVAL_MS 500
#endif
#ifndef MOTOR_DRIVER_RX_TEST_BOOT_DELAY_MS
#define MOTOR_DRIVER_RX_TEST_BOOT_DELAY_MS 8000
#endif

void motor_set_wheels_pwm(int lf, int rf, int lr, int rr);
void motor_set_wheels_mtep_p_loop(int lf, int rf, int lr, int rr);
void motor_hold_zero_speed_mtep(void);
void motor_reset_hold_integral(void);
void motor_reconfigure_and_set_wheels(int lf, int rf, int lr, int rr);
void motor_get_wheel_mtep_cps(float *lf, float *rf, float *lr, float *rr);
bool motor_is_mtep_feedback_fresh(void);

// ================= 电机驱动板持续接收验证 =================
// 目的：绕开 PS2 和舵机，只验证 Arduino 主板是否能持续向电机驱动板发送 $pwm 指令。
// 现象：
//   1. 上电后前 3 秒反复发送 0 速，LED 会闪；
//   2. 之后循环执行：正转 -> 停 -> 反转 -> 停；
//   3. 若驱动板持续收到命令，轮子状态应跟随这个节奏不断变化；
//   4. 若轮子只保持第一次动作，说明后续 $pwm 指令也没有被驱动板正确执行。
static void loopkey_driver_rx_test(void) {
  static unsigned long last_send_ms = 0;
  static bool led_state = false;
  static int last_phase = -1;

  unsigned long now_ms = millis();
  if (now_ms - last_send_ms < MOTOR_DRIVER_RX_TEST_SEND_INTERVAL_MS) {
    motor_update();
    return;
  }
  last_send_ms = now_ms;

  int lf = 0;
  int rf = 0;
  int lr = 0;
  int rr = 0;
  int phase = -1;

  if (now_ms < MOTOR_DRIVER_RX_TEST_BOOT_DELAY_MS) {
    // 前 5 秒持续发 0 输出。
    // 目的：验证“不是只有刚开机那一刻才能发命令”。
    phase = -1;
    lf = rf = lr = rr = 0;
  } else {
    unsigned long t = now_ms - MOTOR_DRIVER_RX_TEST_BOOT_DELAY_MS;
    phase = (t / 3000UL) % 4;

    if (phase == 0) {
      // 超低速正转 3 秒
      lf = MOTOR_DRIVER_RX_TEST_SPEED;
      rf = MOTOR_DRIVER_RX_TEST_SPEED;
      lr = MOTOR_DRIVER_RX_TEST_SPEED;
      rr = MOTOR_DRIVER_RX_TEST_SPEED;
    } else if (phase == 1) {
      // 停 3 秒
      lf = rf = lr = rr = 0;
    } else if (phase == 2) {
      // 超低速反转 3 秒
      lf = -MOTOR_DRIVER_RX_TEST_SPEED;
      rf = -MOTOR_DRIVER_RX_TEST_SPEED;
      lr = -MOTOR_DRIVER_RX_TEST_SPEED;
      rr = -MOTOR_DRIVER_RX_TEST_SPEED;
    } else {
      // 停 3 秒
      lf = rf = lr = rr = 0;
    }
  }

#if MOTOR_DRIVER_RX_TEST_PROFILE == 1
  // Profile 1：原始 $spd 超低速测试，不加换行，不重新配置。
  motor_set_wheels(lf, rf, lr, rr);

#elif MOTOR_DRIVER_RX_TEST_PROFILE == 2
  // Profile 2：$pwm 直接输出测试。当前 v6 默认运行这一项。
  // 如果 PWM 能持续改变而 SPD 不能，说明问题更可能在速度闭环/编码器参数/驱动板速度模式。
  int pwm_lf = 0;
  int pwm_rf = 0;
  int pwm_lr = 0;
  int pwm_rr = 0;
  if (lf > 0) {
    pwm_lf = pwm_rf = pwm_lr = pwm_rr = MOTOR_DRIVER_RX_TEST_PWM;
  } else if (lf < 0) {
    pwm_lf = pwm_rf = pwm_lr = pwm_rr = -MOTOR_DRIVER_RX_TEST_PWM;
  }
  motor_set_wheels_pwm(pwm_lf, pwm_rf, pwm_lr, pwm_rr);

#elif MOTOR_DRIVER_RX_TEST_PROFILE == 3
  // Profile 3：每次相位变化时重新发送配置，再发 $spd。
  // 如果只有这个模式能持续改变，说明驱动板可能需要被重新初始化/唤醒。
  if (phase != last_phase) {
    motor_reconfigure_and_set_wheels(lf, rf, lr, rr);
    last_phase = phase;
  } else {
    motor_set_wheels(lf, rf, lr, rr);
  }

#else
  motor_set_wheels(lf, rf, lr, rr);
#endif

  // 板载 LED 每发一次命令翻转一次，用来证明 Arduino 主循环没有卡死。
  led_state = !led_state;
  digitalWrite(LED_BUILTIN, led_state ? HIGH : LOW);

  motor_update();
}


const int MOTOR_SPEED_MIN = -1000;
const int MOTOR_SPEED_MAX = 1000;

const int JOYSTICK_DEADZONE = 8;
const int JOYSTICK_MAX_ABS = 128;

const int THROTTLE_MIN_EFFECTIVE = 180;  // v15: 提高起步目标，避免轻推时输出太小
const int STRAFE_MIN_EFFECTIVE   = 180;  // v15: 提高左右平移起步目标
const int ROTATE_MIN_EFFECTIVE   = 260;  // v15: 原地旋转需要更高起步力

const int MOTOR_SLEW_ACCEL_STEP = 90;    // v15: 加快从 0 到目标值的爬升速度
const int MOTOR_SLEW_DECEL_STEP = 130;   // v15: 松杆/反向时更快释放上一状态

const int THROTTLE_GAIN_NUM = 10;
const int THROTTLE_GAIN_DEN = 10;

const int STRAFE_GAIN_NUM = 10;
const int STRAFE_GAIN_DEN = 10;

const int ROTATE_GAIN_NUM = 15;
const int ROTATE_GAIN_DEN = 10;

const int ROTATE_PRIORITY_THRESHOLD    = 140;
const int TRANSLATION_IGNORE_THRESHOLD = 120;

// ================= v16：编码器直线保持 / 防自转 =================
// 作用：当前进、后退、左右平移且没有主动旋转输入时，
// 用四轮 MTEP 反馈估计车身是否在自转，并自动叠加一个反向旋转修正量。
// 这不会改变你已经调好的摇杆映射，只是在“纯平移”时补偿跑偏。
#define STRAIGHT_HOLD_ENABLE 1

// 只有平移目标足够明显时才启用，避免摇杆微小漂移触发修正。
const int STRAIGHT_HOLD_TRANSLATION_MIN_PWM = 80;

// 估计到的自转速度小于该值时认为是编码器抖动，不补偿。单位：counts/s。
const float STRAIGHT_HOLD_YAW_CPS_DEADBAND = 120.0f;

// 自转速度 -> 旋转 PWM 修正量的比例。若修正太弱可升到 0.06~0.08；
// 若左右摇摆/过度纠正，可降到 0.02~0.03。
const float STRAIGHT_HOLD_KP = 0.04f;

// 最大旋转修正，防止某个轮子打滑/编码器异常时补偿过猛。
const int STRAIGHT_HOLD_MAX_CORRECTION = 140;

// 如果直线保持开启后偏得更厉害，把 -1 改成 1。
#define STRAIGHT_HOLD_CORRECTION_SIGN -1

// ================= 挡位设置 =================
// 0 = 低速挡，1 = 中速挡，2 = 高速挡
static int g_speed_gear = 1;

const int LOW_GEAR_PERCENT  = 20;        // v15: 低速挡也给足最低起步量
const int MID_GEAR_PERCENT  = 45;        // v15: 默认中速挡从 30% 提高到 45%
const int HIGH_GEAR_PERCENT = 100;

// 右摇杆Y轴三挡逻辑
const int GEAR_HIGH_THRESHOLD = 110;
const int GEAR_LOW_THRESHOLD  = -110;

// 如果你发现右摇杆Y方向和挡位逻辑反了，就把 1 改成 -1
#define GEAR_AXIS_SIGN -1

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


static int current_lf = 0;
static int current_rf = 0;
static int current_lr = 0;
static int current_rr = 0;

static bool loopkey_gear_shift_ready = false;

// ================= 安全停车设置 =================
// 只要 PS2 连接正常，摇杆直接控制小车；摇杆回中时进入 Hold 零速保持。
// 只有 PS2 断连/长时间未刷新时，才进入安全停车。
const unsigned long MOTOR_STOP_REFRESH_MS = 80;
const unsigned long PS2_FRAME_TIMEOUT_MS = 150;
static unsigned long loopkey_last_stop_command_ms = 0;

static void loopkey_stop_motor_safely(void) {
  current_lf = 0;
  current_rf = 0;
  current_lr = 0;
  current_rr = 0;

  motor_reset_hold_integral();

  unsigned long now_ms = millis();
  if (loopkey_last_stop_command_ms == 0 ||
      now_ms - loopkey_last_stop_command_ms >= MOTOR_STOP_REFRESH_MS) {
    motor_stop_all();
    loopkey_last_stop_command_ms = now_ms;
  }

  motor_update();
}

static void loopkey_hold_zero_speed(void) {
  current_lf = 0;
  current_rf = 0;
  current_lr = 0;
  current_rr = 0;

  // v13：PS2 正常连接且摇杆回中时进入零速保持。
  // PS2 断连/断帧时仍然使用 loopkey_stop_motor_safely()，不会保留扭矩。
  motor_hold_zero_speed_mtep();
  loopkey_last_stop_command_ms = 0;
  motor_update();
}

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

static int loopkey_round_float_to_int(float value) {
  if (value >= 0.0f) return (int)(value + 0.5f);
  return (int)(value - 0.5f);
}

static int loopkey_calc_straight_hold_rotate_fix(int throttle, int strafe, int rotate) {
#if STRAIGHT_HOLD_ENABLE
  // 用户主动给了旋转输入时，不抢控制权。
  if (rotate != 0) return 0;

  // 只有“纯平移”时才做直线保持。
  if (abs(throttle) < STRAIGHT_HOLD_TRANSLATION_MIN_PWM &&
      abs(strafe) < STRAIGHT_HOLD_TRANSLATION_MIN_PWM) {
    return 0;
  }

  if (!motor_is_mtep_feedback_fresh()) return 0;

  float fb_lf, fb_rf, fb_lr, fb_rr;
  motor_get_wheel_mtep_cps(&fb_lf, &fb_rf, &fb_lr, &fb_rr);

  // 按当前正常混控公式反推出“自转分量”：
  // target_lf = throttle + strafe + rotate
  // target_rf = throttle - strafe - rotate
  // target_lr = throttle - strafe + rotate
  // target_rr = throttle + strafe - rotate
  // 因此 yaw_cps ≈ (lf - rf + lr - rr) / 4。
  float yaw_cps = (fb_lf - fb_rf + fb_lr - fb_rr) / 4.0f;

  if (yaw_cps > -STRAIGHT_HOLD_YAW_CPS_DEADBAND &&
      yaw_cps <  STRAIGHT_HOLD_YAW_CPS_DEADBAND) {
    return 0;
  }

  float correction = (float)STRAIGHT_HOLD_CORRECTION_SIGN * yaw_cps * STRAIGHT_HOLD_KP;
  if (correction > STRAIGHT_HOLD_MAX_CORRECTION) correction = STRAIGHT_HOLD_MAX_CORRECTION;
  if (correction < -STRAIGHT_HOLD_MAX_CORRECTION) correction = -STRAIGHT_HOLD_MAX_CORRECTION;

  return loopkey_round_float_to_int(correction);
#else
  (void)throttle;
  (void)strafe;
  (void)rotate;
  return 0;
#endif
}

static void loopkey_update_gear_by_right_y(void) {
  int gear_axis = GEAR_AXIS_SIGN * loopkey_apply_deadzone(PS2_RIGHT_Y);

  // 回到中间区后允许下一次换挡
  if (gear_axis < GEAR_HIGH_THRESHOLD && gear_axis > GEAR_LOW_THRESHOLD) {
    loopkey_gear_shift_ready = true;
    return;
  }

  // 未准备好时忽略，避免持续推杆连跳
  if (!loopkey_gear_shift_ready) return;

  if (gear_axis >= GEAR_HIGH_THRESHOLD) {
    if (g_speed_gear < 2) g_speed_gear++;   // 升高一档
    loopkey_gear_shift_ready = false;
  } else if (gear_axis <= GEAR_LOW_THRESHOLD) {
    if (g_speed_gear > 0) g_speed_gear--;   // 降低一档
    loopkey_gear_shift_ready = false;
  }
}

void loop_key(void) {
#if MOTOR_DRIVER_RX_TEST_MODE
  loopkey_driver_rx_test();
  return;
#endif

  // 先处理一小段电机驱动板回传数据。注意底层已经做了“限时/限字节”，不会长期卡住 PS2 控制。
  motor_update();

  // PS2 未连接成功，或主循环超过一段时间没有拿到新的 PS2 帧时，立即停车。
  // 这能避免“手柄断帧后一直沿用上一条摇杆速度命令”。
  // 这里的安全停车只负责处理手柄断连/断帧。
  if (!g_ps2_link_ok || !g_ps2_frame_ready ||
      (millis() - g_ps2_last_read_ms > PS2_FRAME_TIMEOUT_MS)) {
    loopkey_stop_motor_safely();
    return;
  }

  // PS2 正常连接后，摇杆直接控制；摇杆回中直接进入 Hold 零速保持。

  // ===== 按右摇杆Y选择挡位 =====
  loopkey_update_gear_by_right_y();

  // ===== 摇杆映射：根据实测结果 =====
  // 左摇杆X -> 前后移动 (throttle)
  // 左摇杆Y -> 原地旋转 (rotate)
  // 右摇杆X -> 左右平移 (strafe)，方向取反修正
  // 右摇杆Y -> 挡位切换 (gear)
  int throttle = - THROTTLE_SIGN * loopkey_map_axis_to_speed_with_min(PS2_LEFT_X, THROTTLE_MIN_EFFECTIVE);
  int rotate   = ROTATE_SIGN   * loopkey_map_axis_to_speed_with_min(PS2_RIGHT_X, ROTATE_MIN_EFFECTIVE);
  int strafe   = - STRAFE_SIGN  * loopkey_map_axis_to_speed_with_min(PS2_LEFT_Y, STRAFE_MIN_EFFECTIVE);

  throttle = throttle * THROTTLE_GAIN_NUM / THROTTLE_GAIN_DEN;
  rotate   = rotate   * ROTATE_GAIN_NUM   / ROTATE_GAIN_DEN;
  strafe   = strafe   * STRAFE_GAIN_NUM   / STRAFE_GAIN_DEN;

  // v16：纯前进/后退/左右平移时，自动压制由轮速不一致造成的自转趋势。
  // 注意：这里是在混控前给 rotate 叠加一个小修正，不改变你当前的摇杆轴映射。
  rotate = loopkey_clamp_motor_speed(rotate +
           loopkey_calc_straight_hold_rotate_fix(throttle, strafe, rotate));

  int target_lf, target_rf, target_lr, target_rr;

  // ===== 旋转优先模式 =====
  if (abs(rotate) >= ROTATE_PRIORITY_THRESHOLD &&
      abs(throttle) < TRANSLATION_IGNORE_THRESHOLD &&
      abs(strafe) < TRANSLATION_IGNORE_THRESHOLD) {
    target_lf = loopkey_clamp_motor_speed(-rotate);
    target_rf = loopkey_clamp_motor_speed(-rotate);
    target_lr = loopkey_clamp_motor_speed( rotate);
    target_rr = loopkey_clamp_motor_speed( rotate);
  } else {
    // 正常混控（麦轮/四轮独立驱动混控公式）
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
  
  // ===== 摇杆回中：进入 v12 零速保持 =====
  // 目标速度设为 0，读取 MTEP 判断是否仍在滑动；若滑动，则输出反向 PWM 抵消。
  // 注意：PS2 断连/断帧仍走 loopkey_stop_motor_safely()，不会启用保持扭矩。
  if (target_lf == 0 && target_rf == 0 && target_lr == 0 && target_rr == 0) {
    loopkey_hold_zero_speed();
    return;
  }

  // ===== 平滑过渡 =====
  current_lf = loopkey_approach_speed(current_lf, target_lf);
  current_rf = loopkey_approach_speed(current_rf, target_rf);
  current_lr = loopkey_approach_speed(current_lr, target_lr);
  current_rr = loopkey_approach_speed(current_rr, target_rr);

  // ===== 输出到四个轮子 =====
  // v10：这里不再直接输出普通 $pwm，而是用 MTEP 反馈做 Arduino 侧 P 闭环修正。
  // 底层仍然发送稳定的 $pwm:...#，不使用驱动板内部不稳定的 $spd。
  motor_set_wheels_mtep_p_loop(current_lf, current_rf, current_lr, current_rr);
  loopkey_last_stop_command_ms = 0;
  motor_update();

  delay(10);
}
