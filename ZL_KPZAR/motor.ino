/****************************************************************************
  USART encoder motor control
****************************************************************************/

#include "app_motor_usart.hpp"
#include "bsp_motor_usart.hpp"

// If a wheel direction is reversed, flip the sign here.
#define LEFT_FRONT_DIR 1
#define RIGHT_FRONT_DIR 1
#define LEFT_REAR_DIR 1
#define RIGHT_REAR_DIR 1

// Motor configuration copied from USART example.
#define MOTOR_TYPE 1

// 0=关闭上传；1=上传总编码值 MAll；2=上传 10ms 编码增量 MTEP；3=上传实时速度 MSPD。
// v14 正常运行版：使用 MTEP 做实时速度反馈，并在 Arduino 侧做 P 闭环 + 死区补偿 + 优化后的零速保持。
// 原因：你已实测 MTEP 在轮子转动时会变化，而 MSPD 仍为 0。
// MTEP 是 10ms 编码器增量，可以直接换算成 counts/s。
#define UPLOAD_DATA 2
#define MOTOR_UPLOAD_REFRESH_MS 1000

static int motor_clamp_speed(int speed) {
  if (speed < -1000)
    return -1000;
  if (speed > 1000)
    return 1000;
  return speed;
}

static int motor_apply_dir(int speed, int dir_sign) {
  int value = motor_clamp_speed(speed);
  value = value * dir_sign;
  return motor_clamp_speed(value);
}

static int last_lf = 0;
static int last_rf = 0;
static int last_lr = 0;
static int last_rr = 0;

// ================= MTEP 速度反馈参数 =================
// MTEP 表示 10ms 内的编码器计数增量，所以 counts/s = MTEP * 100。
// 下面的 ENCODER_*_SIGN 只影响“显示/反馈速度”的正负方向，不影响电机转动方向。
// 如果某个轮子正转时反馈为负，就把对应值从 1 改成 -1。
// 依据你刚刚的日志：PWM 为正时 MTEP 多数为负，PWM 为负时 MTEP 多数为正。
// 因此这里默认全部取反，让“反馈速度方向”和“PWM 指令方向”一致。
// 如果之后发现某一路越调越快/方向明显反了，把对应 -1 改成 1。
#define ENCODER_LEFT_FRONT_SIGN  -1
#define ENCODER_RIGHT_FRONT_SIGN -1
#define ENCODER_LEFT_REAR_SIGN   -1
#define ENCODER_RIGHT_REAR_SIGN  -1

// 如果你以后知道“轮子转一圈对应多少编码器计数”，可以填到这里。
// 现在先设为 0，表示暂时只输出 counts/s，不计算 RPM 和 m/s。
#define ENCODER_COUNTS_PER_WHEEL_REV 0.0f
#define WHEEL_DIAMETER_MM_FOR_FEEDBACK 67.0f

// ================= Arduino 侧 MTEP 比例闭环 =================
// 这不是使用驱动板内部的 $spd 闭环，而是：
//   PS2 摇杆目标 -> 估算 target_cps -> 读取 MTEP 实际 cps -> 修正 $pwm 输出。
// 第一版先只用 P 控制，避免 PID 参数太多导致难调。
#define MTEP_P_LOOP_ENABLE 1

// 从你的日志估计：PWM 约 1000 时，MTEP 约 35~43，即 cps 约 3500~4300。
// 所以先取 4.0 counts/s per PWM：target_cps = target_pwm * 4.0。
// 如果你觉得目标速度太激进/太保守，可以优先调这个。
#define MTEP_TARGET_CPS_PER_PWM 4.0f

// P 增益，单位约为 PWM/(counts/s)。
// v11 略微提高到 0.12：让速度跟随更积极。
// 若小车抖动/忽快忽慢，减小到 0.08 或 0.06；若修正太弱，可小步增大到 0.15。
#define MTEP_P_GAIN 0.14f   // v15: 行驶 P 闭环稍微更积极，起步时补偿更快

// 单次 P 修正的最大幅度，防止某一路卡住时补偿过猛。
#define MTEP_MAX_CORRECTION_PWM 320.0f

// ================= v11：最小有效 PWM / 死区补偿 =================
// 现象：PWM 太小时电机不一定能克服静摩擦，编码器 MTEP 会一直为 0。
// 做法：只要摇杆目标不为 0，最终输出 PWM 至少达到一个“能动起来”的门槛。
// 这不是闭环本身，而是让闭环有机会工作；否则 P 控制一直加得很慢，低速会卡住。
#define MTEP_DEADZONE_COMP_ENABLE 1

// 普通轮子的最小驱动 PWM。若低速太冲，降到 160；若低速仍不动，升到 220。
#define MTEP_MIN_DRIVE_PWM_NORMAL 230   // v15: 普通轮最低有效 PWM 提高，减少磨蹭起步

// 第三路/左后轮之前实测启动门槛偏高。默认给一个温和补偿。
// 如果你暂时不想特别处理第三路，可把它改成和 NORMAL 一样，例如 190。
#define MTEP_MIN_DRIVE_PWM_LEFT_REAR 360  // v15: 第三路起步门槛偏高，继续给一点补偿

// 当摇杆目标很小的时候，如果直接跳到最小 PWM 会显得突兀。
// 这里给一个软启动：目标 PWM 小于该值时，按比例降低最小补偿。
#define MTEP_DEADZONE_SOFT_RANGE_PWM 160  // v15: 更快达到最低有效 PWM，但仍保留软启动

// 超过这个时间没有收到新的 MTEP，就退回普通 PWM，不做闭环修正。
#define MTEP_FEEDBACK_TIMEOUT_MS 120


// ================= v12：摇杆回中零速保持 / 斜坡防溜 =================
// 目标：摇杆回中后，不再简单 $pwm:0,0,0,0# 放空；
// 而是读取 MTEP 速度反馈，把目标速度设为 0，检测到滑动就施加反向扭矩。
// 注意：这只是初版防溜，不等同于工业级刹车；第一次必须架空或手托测试。
#define MTEP_HOLD_ENABLE 1

// 速度死区：小于这个 cps 认为接近静止，避免编码器抖动导致持续来回补偿。
#define MTEP_HOLD_CPS_DEADBAND 180.0f

// 零速保持 P 项：检测到滑动时，立即按速度反向给力。
// 若车在坡上越滑越快，可小步增大；若回中后明显抖动/来回抽动，减小。
#define MTEP_HOLD_P_GAIN 0.06f

// 零速保持 I 项：让车停住后仍保留一部分抵抗斜坡的扭矩。
// 若停住后仍慢慢溜，增大；若停住后自己往反方向爬，减小。
#define MTEP_HOLD_I_GAIN 0.06f

// 保持扭矩上限。第一版不要太大，避免方向符号错时车突然冲出去。
#define MTEP_HOLD_MAX_PWM 260.0f

// 一旦检测到明显滑动，保持输出至少达到这个值，帮助克服静摩擦。
// 若车回中后有轻微冲一下，可降到 100；若坡上仍刹不住，可升到 160。
#define MTEP_HOLD_MIN_PWM_WHEN_SLIPPING 80

// 积分限幅，单位约为 cps*s。I_GAIN * LIMIT 不应超过 HOLD_MAX_PWM 太多。
#define MTEP_HOLD_INTEGRAL_LIMIT 2200.0f

// 如果长时间没有刷新 hold，自动按较大的 dt 限制处理，防止积分突然暴涨。
#define MTEP_HOLD_MAX_DT_SEC 0.05f

// v14：当已经接近静止时，积分项缓慢泄放，避免平地上一直憋着力。
// 注意这不是立刻清零；如果车又开始滑，积分项会重新增加。
// 数值越接近 1，保持力衰减越慢；越小，平地释放越快但坡上更容易出现“滑一下补一下”。
#define MTEP_HOLD_INTEGRAL_DECAY_WHEN_STATIC 0.995f
#define MTEP_HOLD_INTEGRAL_ZERO_EPS 2.0f
#define MTEP_HOLD_OUTPUT_ZERO_PWM 6

void motor_set_wheels_pwm(int lf, int rf, int lr, int rr);
void motor_set_wheels_mtep_p_loop(int lf, int rf, int lr, int rr);
void motor_hold_zero_speed_mtep(void);
void motor_reset_hold_integral(void);
void motor_get_wheel_mtep_cps(float *lf, float *rf, float *lr, float *rr);
static bool motor_mtep_feedback_is_fresh(void);
static int motor_calc_p_loop_pwm(int index, int base_pwm, float feedback_cps, bool feedback_fresh);
static int motor_apply_deadzone_compensation(int index, int base_pwm, int output_pwm);
static int motor_get_min_drive_pwm_for_wheel(int index);
static int motor_calc_zero_hold_pwm(int index, float feedback_cps, bool feedback_fresh);
static float motor_apply_encoder_sign(int index, float value);
static float motor_clamp_float(float value, float min_value, float max_value);
static int motor_round_float_to_int(float value);

// 提前定义：避免 Arduino IDE 在 .ino 自动生成原型时漏掉这个函数，导致编译报
// 'motor_get_wheel_mtep_cps' was not declared in this scope。
static float motor_apply_encoder_sign(int index, float value) {
  if (index == 0) return value * ENCODER_LEFT_FRONT_SIGN;
  if (index == 1) return value * ENCODER_RIGHT_FRONT_SIGN;
  if (index == 2) return value * ENCODER_LEFT_REAR_SIGN;
  return value * ENCODER_RIGHT_REAR_SIGN;
}

// 用 MTEP 换算编码器计数速度，单位：counts/s。
// 这一步不依赖轮径、减速比，也不依赖驱动板内部 MSPD，所以最稳定。
void motor_get_wheel_mtep_cps(float *lf, float *rf, float *lr, float *rr) {
  if (lf) *lf = motor_apply_encoder_sign(0, g_mtep_counts_per_second[0]);
  if (rf) *rf = motor_apply_encoder_sign(1, g_mtep_counts_per_second[1]);
  if (lr) *lr = motor_apply_encoder_sign(2, g_mtep_counts_per_second[2]);
  if (rr) *rr = motor_apply_encoder_sign(3, g_mtep_counts_per_second[3]);
}

static float motor_clamp_float(float value, float min_value, float max_value) {
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

static int motor_round_float_to_int(float value) {
  if (value >= 0.0f) return (int)(value + 0.5f);
  return (int)(value - 0.5f);
}

static float g_loop_target_cps_dbg[4] = {0, 0, 0, 0};
static float g_loop_feedback_cps_dbg[4] = {0, 0, 0, 0};
static int g_loop_base_pwm_dbg[4] = {0, 0, 0, 0};
static int g_loop_output_pwm_dbg[4] = {0, 0, 0, 0};
static bool g_loop_active_dbg = false;
static bool g_loop_feedback_fresh_dbg = false;

static float g_hold_integral_cps_s[4] = {0, 0, 0, 0};
static int g_hold_output_pwm_dbg[4] = {0, 0, 0, 0};
static bool g_hold_active_dbg = false;
static bool g_hold_feedback_fresh_dbg = false;
static unsigned long g_hold_last_update_ms = 0;

static void motor_configure_module(void) {
  send_upload_data(false, false, false);

#if MOTOR_TYPE == 1
  send_motor_type(MOTOR_520);
  delay(100);
  send_pulse_phase(56);
  delay(100);
  send_pulse_line(11);
  delay(100);
  send_wheel_diameter(67.00);
  delay(100);
  send_motor_deadzone(1600);
  delay(100);
#elif MOTOR_TYPE == 2
  send_motor_type(MOTOR_310);
  delay(100);
  send_pulse_phase(20);
  delay(100);
  send_pulse_line(13);
  delay(100);
  send_wheel_diameter(48.00);
  delay(100);
  send_motor_deadzone(1300);
  delay(100);
#elif MOTOR_TYPE == 3
  send_motor_type(MOTOR_TT_Encoder);
  delay(100);
  send_pulse_phase(45);
  delay(100);
  send_pulse_line(13);
  delay(100);
  send_wheel_diameter(68.00);
  delay(100);
  send_motor_deadzone(1250);
  delay(100);
#elif MOTOR_TYPE == 4
  send_motor_type(MOTOR_TT);
  delay(100);
  send_pulse_phase(48);
  delay(100);
  send_motor_deadzone(1000);
  delay(100);
#elif MOTOR_TYPE == 5
  send_motor_type(MOTOR_520);
  delay(100);
  send_pulse_phase(40);
  delay(100);
  send_pulse_line(11);
  delay(100);
  send_wheel_diameter(67.00);
  delay(100);
  send_motor_deadzone(1600);
  delay(100);
#endif

#if UPLOAD_DATA == 1
  send_upload_data(true, false, false);
  delay(10);
#elif UPLOAD_DATA == 2
  send_upload_data(false, true, false);
  delay(10);
#elif UPLOAD_DATA == 3
  send_upload_data(false, false, true);
  delay(10);
#endif
}

static void motor_refresh_upload_request(void) {
#if UPLOAD_DATA == 1
  send_upload_data(true, false, false);
#elif UPLOAD_DATA == 2
  send_upload_data(false, true, false);
#elif UPLOAD_DATA == 3
  send_upload_data(false, false, true);
#endif
}

void setup_motor(void) {
  motor1_speed = 0;
  motor2_speed = 0;
  g_recv_flag = 0;
  last_lf = 0;
  last_rf = 0;
  last_lr = 0;
  last_rr = 0;

  Usart_init();
  delay(100);
  motor_configure_module();

  // 上电后立即明确发送 0 输出，防止驱动板沿用旧状态或例程默认加速启动。
  // 这里同时发 $pwm 和 $spd 的 0 值；后续正常控制只使用 $pwm。
  Contrl_Pwm(0, 0, 0, 0);
  delay(20);
  Contrl_Speed(0, 0, 0, 0);
  delay(20);
}

void motor_update(void) {
  static unsigned long last_upload_refresh_ms = 0;

  // 周期性重复发送上传配置，避免驱动板因复位/状态切换后停止上传。
  if (millis() - last_upload_refresh_ms >= MOTOR_UPLOAD_REFRESH_MS) {
    last_upload_refresh_ms = millis();
    motor_refresh_upload_request();
  }

  Motor_USART_Recieve();
  if (g_recv_flag == 1) {
    g_recv_flag = 0;
    Deal_data_real();
  }
}

// Legacy interface
// 旧接口也改为 $pwm 输出，避免正常运行时再次触发 $spd 速度闭环问题。
void motor1_SetSpeed(int Speed) {
  motor_set_wheels_pwm(Speed, last_rf, Speed, last_rr);
}

void motor2_SetSpeed(int Speed) {
  motor_set_wheels_pwm(last_lf, Speed, last_lr, Speed);
}

void motor_set_wheels(int lf, int rf, int lr, int rr) {
  last_lf = lf;
  last_rf = rf;
  last_lr = lr;
  last_rr = rr;

  int m1 = motor_apply_dir(lf, LEFT_FRONT_DIR);
  int m2 = motor_apply_dir(rf, RIGHT_FRONT_DIR);
  int m3 = motor_apply_dir(lr, LEFT_REAR_DIR);
  int m4 = motor_apply_dir(rr, RIGHT_REAR_DIR);

  Contrl_Speed(m1, m2, m3, m4);
}

void motor_stop_all(void) {
  motor_reset_hold_integral();
  motor_set_wheels_pwm(0, 0, 0, 0);
}

void motor_set_wheels_pwm(int lf, int rf, int lr, int rr) {
  last_lf = lf;
  last_rf = rf;
  last_lr = lr;
  last_rr = rr;

  int m1 = motor_apply_dir(lf, LEFT_FRONT_DIR);
  int m2 = motor_apply_dir(rf, RIGHT_FRONT_DIR);
  int m3 = motor_apply_dir(lr, LEFT_REAR_DIR);
  int m4 = motor_apply_dir(rr, RIGHT_REAR_DIR);

  Contrl_Pwm(m1, m2, m3, m4);
}

void motor_set_wheels_mtep_p_loop(int lf, int rf, int lr, int rr) {
  // 手柄有非零目标时，退出零速保持，清除保持积分，避免切回手动时残留扭矩。
  motor_reset_hold_integral();

  last_lf = lf;
  last_rf = rf;
  last_lr = lr;
  last_rr = rr;

  float fb_lf, fb_rf, fb_lr, fb_rr;
  motor_get_wheel_mtep_cps(&fb_lf, &fb_rf, &fb_lr, &fb_rr);

  bool feedback_fresh = motor_mtep_feedback_is_fresh();
  g_loop_feedback_fresh_dbg = feedback_fresh;
#if MTEP_P_LOOP_ENABLE
  g_loop_active_dbg = feedback_fresh;
#else
  g_loop_active_dbg = false;
#endif

  int out_lf = motor_calc_p_loop_pwm(0, lf, fb_lf, feedback_fresh);
  int out_rf = motor_calc_p_loop_pwm(1, rf, fb_rf, feedback_fresh);
  int out_lr = motor_calc_p_loop_pwm(2, lr, fb_lr, feedback_fresh);
  int out_rr = motor_calc_p_loop_pwm(3, rr, fb_rr, feedback_fresh);

  int m1 = motor_apply_dir(out_lf, LEFT_FRONT_DIR);
  int m2 = motor_apply_dir(out_rf, RIGHT_FRONT_DIR);
  int m3 = motor_apply_dir(out_lr, LEFT_REAR_DIR);
  int m4 = motor_apply_dir(out_rr, RIGHT_REAR_DIR);

  Contrl_Pwm(m1, m2, m3, m4);
}


void motor_reset_hold_integral(void) {
  for (int i = 0; i < 4; i++) {
    g_hold_integral_cps_s[i] = 0.0f;
    g_hold_output_pwm_dbg[i] = 0;
  }
  g_hold_active_dbg = false;
  g_hold_feedback_fresh_dbg = false;
  g_hold_last_update_ms = 0;
}

void motor_hold_zero_speed_mtep(void) {
#if MTEP_HOLD_ENABLE
  float fb_lf, fb_rf, fb_lr, fb_rr;
  motor_get_wheel_mtep_cps(&fb_lf, &fb_rf, &fb_lr, &fb_rr);

  bool feedback_fresh = motor_mtep_feedback_is_fresh();
  g_hold_feedback_fresh_dbg = feedback_fresh;
  g_hold_active_dbg = feedback_fresh;

  // 零速保持时，行驶 P 闭环调试项显示为 OFF，但保留反馈值。
  g_loop_active_dbg = false;
  g_loop_feedback_fresh_dbg = feedback_fresh;
  g_loop_base_pwm_dbg[0] = 0; g_loop_base_pwm_dbg[1] = 0;
  g_loop_base_pwm_dbg[2] = 0; g_loop_base_pwm_dbg[3] = 0;
  g_loop_target_cps_dbg[0] = 0; g_loop_target_cps_dbg[1] = 0;
  g_loop_target_cps_dbg[2] = 0; g_loop_target_cps_dbg[3] = 0;
  g_loop_feedback_cps_dbg[0] = fb_lf;
  g_loop_feedback_cps_dbg[1] = fb_rf;
  g_loop_feedback_cps_dbg[2] = fb_lr;
  g_loop_feedback_cps_dbg[3] = fb_rr;

  int out_lf = motor_calc_zero_hold_pwm(0, fb_lf, feedback_fresh);
  int out_rf = motor_calc_zero_hold_pwm(1, fb_rf, feedback_fresh);
  int out_lr = motor_calc_zero_hold_pwm(2, fb_lr, feedback_fresh);
  int out_rr = motor_calc_zero_hold_pwm(3, fb_rr, feedback_fresh);

  g_loop_output_pwm_dbg[0] = out_lf;
  g_loop_output_pwm_dbg[1] = out_rf;
  g_loop_output_pwm_dbg[2] = out_lr;
  g_loop_output_pwm_dbg[3] = out_rr;

  g_hold_last_update_ms = millis();

  int m1 = motor_apply_dir(out_lf, LEFT_FRONT_DIR);
  int m2 = motor_apply_dir(out_rf, RIGHT_FRONT_DIR);
  int m3 = motor_apply_dir(out_lr, LEFT_REAR_DIR);
  int m4 = motor_apply_dir(out_rr, RIGHT_REAR_DIR);

  Contrl_Pwm(m1, m2, m3, m4);
#else
  motor_stop_all();
#endif
}

void motor_reconfigure_and_set_wheels(int lf, int rf, int lr, int rr) {
  // 诊断用：在发送速度前重新发一遍关键配置，测试驱动板是否需要重新初始化/唤醒。
  // 注意：这不是最终控制方案，只用于定位问题。
  send_upload_data(false, false, false);
  delay(20);
  motor_configure_module();
  delay(20);
  motor_set_wheels(lf, rf, lr, rr);
}

static bool motor_mtep_feedback_is_fresh(void) {
  if (g_mtep_last_update_ms == 0) return false;
  if (g_motor_rx_last_type != 2) return false;
  return (millis() - g_mtep_last_update_ms) <= MTEP_FEEDBACK_TIMEOUT_MS;
}

static int motor_calc_p_loop_pwm(int index, int base_pwm, float feedback_cps, bool feedback_fresh) {
  g_loop_base_pwm_dbg[index] = base_pwm;
  g_loop_feedback_cps_dbg[index] = feedback_cps;

  // v11：目标速度仍由摇杆给出的 base_pwm 决定。
  // 死区补偿只影响最终输出，不把很小的摇杆目标强行改成很大的目标速度。
  g_loop_target_cps_dbg[index] = (float)base_pwm * MTEP_TARGET_CPS_PER_PWM;

#if MTEP_P_LOOP_ENABLE
  if (base_pwm == 0) {
    g_loop_output_pwm_dbg[index] = 0;
    return 0;
  }

  int output_pwm = base_pwm;

  if (feedback_fresh) {
    float error_cps = g_loop_target_cps_dbg[index] - feedback_cps;
    float correction = error_cps * MTEP_P_GAIN;
    correction = motor_clamp_float(correction, -MTEP_MAX_CORRECTION_PWM, MTEP_MAX_CORRECTION_PWM);
    output_pwm = base_pwm + motor_round_float_to_int(correction);
  }

  output_pwm = motor_clamp_speed(output_pwm);
  output_pwm = motor_apply_deadzone_compensation(index, base_pwm, output_pwm);
  output_pwm = motor_clamp_speed(output_pwm);

  g_loop_output_pwm_dbg[index] = output_pwm;
  return output_pwm;
#else
  int output_pwm = motor_apply_deadzone_compensation(index, base_pwm, base_pwm);
  g_loop_output_pwm_dbg[index] = output_pwm;
  return output_pwm;
#endif
}


static int motor_calc_zero_hold_pwm(int index, float feedback_cps, bool feedback_fresh) {
  if (!feedback_fresh) {
    g_hold_integral_cps_s[index] = 0.0f;
    g_hold_output_pwm_dbg[index] = 0;
    return 0;
  }

  unsigned long now_ms = millis();
  float dt = 0.01f;
  if (g_hold_last_update_ms != 0) {
    unsigned long dt_ms = now_ms - g_hold_last_update_ms;
    dt = (float)dt_ms / 1000.0f;
    if (dt <= 0.0f) dt = 0.01f;
    if (dt > MTEP_HOLD_MAX_DT_SEC) dt = MTEP_HOLD_MAX_DT_SEC;
  }

  float error_cps = -feedback_cps;  // target_cps = 0，所以 error = 0 - feedback。
  bool slipping = (feedback_cps > MTEP_HOLD_CPS_DEADBAND) ||
                  (feedback_cps < -MTEP_HOLD_CPS_DEADBAND);

  // 只有明显滑动时才积分，避免编码器零点小抖动越积越大。
  if (slipping) {
    // 如果滑动方向和之前积累的保持方向相反，先快速削弱旧积分，避免反向时拖泥带水。
    if (g_hold_integral_cps_s[index] * error_cps < 0.0f) {
      g_hold_integral_cps_s[index] *= 0.70f;
    }

    g_hold_integral_cps_s[index] += error_cps * dt;
    g_hold_integral_cps_s[index] = motor_clamp_float(g_hold_integral_cps_s[index],
                                                     -MTEP_HOLD_INTEGRAL_LIMIT,
                                                      MTEP_HOLD_INTEGRAL_LIMIT);
  } else {
    // v14：接近静止时，积分项慢慢泄放。
    // 这样平地上不会一直憋力；在坡上如果又开始滑，积分会再次建立。
    g_hold_integral_cps_s[index] *= MTEP_HOLD_INTEGRAL_DECAY_WHEN_STATIC;
    if (g_hold_integral_cps_s[index] > -MTEP_HOLD_INTEGRAL_ZERO_EPS &&
        g_hold_integral_cps_s[index] <  MTEP_HOLD_INTEGRAL_ZERO_EPS) {
      g_hold_integral_cps_s[index] = 0.0f;
    }
  }

  float output = MTEP_HOLD_P_GAIN * error_cps +
                 MTEP_HOLD_I_GAIN * g_hold_integral_cps_s[index];
  output = motor_clamp_float(output, -MTEP_HOLD_MAX_PWM, MTEP_HOLD_MAX_PWM);

  // 检测到滑动时，给一个最低保持扭矩；方向由 output/error 决定。
  if (slipping && output != 0.0f) {
    int sign = output > 0.0f ? 1 : -1;
    float abs_out = output > 0.0f ? output : -output;
    if (abs_out < MTEP_HOLD_MIN_PWM_WHEN_SLIPPING) {
      abs_out = MTEP_HOLD_MIN_PWM_WHEN_SLIPPING;
    }
    output = sign * abs_out;
  }

  int pwm = motor_clamp_speed(motor_round_float_to_int(output));

  // 很小的输出直接置零，避免 $pwm:3,-2... 这类无意义抖动。
  if (pwm > -MTEP_HOLD_OUTPUT_ZERO_PWM && pwm < MTEP_HOLD_OUTPUT_ZERO_PWM) {
    pwm = 0;
  }

  g_hold_output_pwm_dbg[index] = pwm;
  return pwm;
}

static int motor_get_min_drive_pwm_for_wheel(int index) {
  if (index == 2) {
    return MTEP_MIN_DRIVE_PWM_LEFT_REAR;
  }
  return MTEP_MIN_DRIVE_PWM_NORMAL;
}

static int motor_apply_deadzone_compensation(int index, int base_pwm, int output_pwm) {
#if MTEP_DEADZONE_COMP_ENABLE
  if (base_pwm == 0 || output_pwm == 0) return 0;

  int sign = base_pwm > 0 ? 1 : -1;
  int out_abs = abs(output_pwm);
  int base_abs = abs(base_pwm);
  int min_pwm = motor_get_min_drive_pwm_for_wheel(index);

  // 软启动：小摇杆输入时，不立刻跳到完整最小 PWM。
  // 例：base_abs=110，soft_range=220，则补偿门槛约为 min_pwm 的一半。
  if (base_abs < MTEP_DEADZONE_SOFT_RANGE_PWM) {
    long softened = (long)min_pwm * base_abs / MTEP_DEADZONE_SOFT_RANGE_PWM;
    if (softened < MTEP_MIN_DRIVE_PWM_NORMAL / 2) {
      softened = MTEP_MIN_DRIVE_PWM_NORMAL / 2;
    }
    min_pwm = (int)softened;
  }

  if (out_abs < min_pwm) {
    out_abs = min_pwm;
  }

  return sign * out_abs;
#else
  (void)index;
  (void)base_pwm;
  return output_pwm;
#endif
}


void motor_get_mtep_hold_debug(int hold_pwm[4], bool *hold_active, bool *feedback_fresh) {
  for (int i = 0; i < 4; i++) {
    if (hold_pwm) hold_pwm[i] = g_hold_output_pwm_dbg[i];
  }
  if (hold_active) *hold_active = g_hold_active_dbg;
  if (feedback_fresh) *feedback_fresh = g_hold_feedback_fresh_dbg;
}

void motor_get_mtep_p_loop_debug(float target_cps[4], float feedback_cps[4],
                                 int base_pwm[4], int output_pwm[4],
                                 bool *loop_active, bool *feedback_fresh) {
  for (int i = 0; i < 4; i++) {
    if (target_cps) target_cps[i] = g_loop_target_cps_dbg[i];
    if (feedback_cps) feedback_cps[i] = g_loop_feedback_cps_dbg[i];
    if (base_pwm) base_pwm[i] = g_loop_base_pwm_dbg[i];
    if (output_pwm) output_pwm[i] = g_loop_output_pwm_dbg[i];
  }
  if (loop_active) *loop_active = g_loop_active_dbg;
  if (feedback_fresh) *feedback_fresh = g_loop_feedback_fresh_dbg;
}

// 读取 MTEP 原始值：驱动板上传的 10ms 编码器增量。
// 对应关系：0=左前，1=右前，2=左后，3=右后。
void motor_get_wheel_mtep(int *lf, int *rf, int *lr, int *rr) {
  if (lf) *lf = Encoder_Offset[0];
  if (rf) *rf = Encoder_Offset[1];
  if (lr) *lr = Encoder_Offset[2];
  if (rr) *rr = Encoder_Offset[3];
}

// 如果 ENCODER_COUNTS_PER_WHEEL_REV 已知且大于 0，可以把 counts/s 换算成 RPM。
void motor_get_wheel_mtep_rpm(float *lf, float *rf, float *lr, float *rr) {
  if (ENCODER_COUNTS_PER_WHEEL_REV > 0.0f) {
    float cps_lf, cps_rf, cps_lr, cps_rr;
    motor_get_wheel_mtep_cps(&cps_lf, &cps_rf, &cps_lr, &cps_rr);
    if (lf) *lf = cps_lf * 60.0f / ENCODER_COUNTS_PER_WHEEL_REV;
    if (rf) *rf = cps_rf * 60.0f / ENCODER_COUNTS_PER_WHEEL_REV;
    if (lr) *lr = cps_lr * 60.0f / ENCODER_COUNTS_PER_WHEEL_REV;
    if (rr) *rr = cps_rr * 60.0f / ENCODER_COUNTS_PER_WHEEL_REV;
  } else {
    if (lf) *lf = 0;
    if (rf) *rf = 0;
    if (lr) *lr = 0;
    if (rr) *rr = 0;
  }
}

float motor_get_average_abs_mtep_cps(void) {
  float lf, rf, lr, rr;
  motor_get_wheel_mtep_cps(&lf, &rf, &lr, &rr);
  if (lf < 0) lf = -lf;
  if (rf < 0) rf = -rf;
  if (lr < 0) lr = -lr;
  if (rr < 0) rr = -rr;
  return (lf + rf + lr + rr) / 4.0f;
}

// 读取编码器驱动板上传的四轮实时速度。
// 注意：这个函数读取的是驱动板 MSPD，当前实测 MSPD 仍为 0。
// v9 推荐优先使用 motor_get_wheel_mtep_cps()。
void motor_get_wheel_speed(float *lf, float *rf, float *lr, float *rr) {
  if (lf) *lf = g_Speed[0];
  if (rf) *rf = g_Speed[1];
  if (lr) *lr = g_Speed[2];
  if (rr) *rr = g_Speed[3];
}

static float motor_abs_float(float value) {
  return value >= 0 ? value : -value;
}

float motor_get_average_abs_speed(void) {
  return (motor_abs_float(g_Speed[0]) + motor_abs_float(g_Speed[1]) +
          motor_abs_float(g_Speed[2]) + motor_abs_float(g_Speed[3])) / 4.0;
}
