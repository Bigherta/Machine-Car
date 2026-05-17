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

// 1=上传总编码值 MAll；2=上传 10ms 编码增量 MTEP；3=上传实时速度 MSPD。
// 这里选择 3，让驱动板持续回传四个轮子的实时速度，解析后存入 g_Speed[0..3]。
#define UPLOAD_DATA 3

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

  // 上电后立即明确发送 0 速，防止驱动板沿用旧状态或例程默认加速启动。
  Contrl_Speed(0, 0, 0, 0);
  delay(20);
}

void motor_update(void) {
  Motor_USART_Recieve();
  if (g_recv_flag == 1) {
    g_recv_flag = 0;
    Deal_data_real();   // 若收到 $MSPD:...#，这里会刷新 g_Speed[0..3]
  }
}

// Legacy interface
void motor1_SetSpeed(int Speed) {
  motor_set_wheels(Speed, last_rf, Speed, last_rr);
}

void motor2_SetSpeed(int Speed) {
  motor_set_wheels(last_lf, Speed, last_lr, Speed);
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

void motor_stop_all(void) { motor_set_wheels(0, 0, 0, 0); }


// 读取编码器驱动板上传的四轮实时速度。
// 对应关系：0=左前，1=右前，2=左后，3=右后。
// 单位取决于驱动板固件的 MSPD 上传定义；通常与 send_wheel_diameter() 配置有关。
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
