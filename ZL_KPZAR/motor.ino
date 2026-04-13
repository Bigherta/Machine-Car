/****************************************************************************
  总线马达版本
  4个电机:
  左前 000
  右前 001
  左后 008
  右后 009
****************************************************************************/

#define LEFT_FRONT_ID 0
#define RIGHT_FRONT_ID 1
#define LEFT_REAR_ID 8
#define RIGHT_REAR_ID 9

// 如果某个轮子方向反了，只改这里的符号
#define LEFT_FRONT_DIR 1
#define RIGHT_FRONT_DIR 1
#define LEFT_REAR_DIR 1
#define RIGHT_REAR_DIR 1

#define BUS_PWM_STOP 1500
#define BUS_PWM_MIN_OFFSET 200
#define BUS_PWM_MAX_OFFSET 900
const int PARKING_MODE_SPEED_LIMIT = 340;

extern bool g_parking_mode_active;

static int last_lf_cmd = 12345;
static int last_rf_cmd = 12345;
static int last_lr_cmd = 12345;
static int last_rr_cmd = 12345;

static int motor_bus_clamp_speed(int speed) {
  if (speed < -1000)
    return -1000;
  if (speed > 1000)
    return 1000;
  return speed;
}

static int motor_bus_speed_to_pwm(int speed) {
  speed = motor_bus_clamp_speed(speed);

  if (speed == 0)
    return BUS_PWM_STOP;

  int sign = (speed > 0) ? 1 : -1;
  int mag = abs(speed);

  long offset = BUS_PWM_MIN_OFFSET +
                (long)(BUS_PWM_MAX_OFFSET - BUS_PWM_MIN_OFFSET) * mag / 1000;

  int pwm = BUS_PWM_STOP + sign * (int)offset;

  if (pwm > 2500)
    pwm = 2500;
  if (pwm < 500)
    pwm = 500;

  return pwm;
}

static void bus_send_motor_cmd(u8 id, int pwm) {
  char cmd[20];
  sprintf(cmd, "#%03dP%04dT0000!", id, pwm);
  uart_send_str((u8 *)cmd);
  delay(3);
}

static void set_one_bus_motor(u8 id, int speed, int dir_sign, int &last_cmd) {
  if (g_parking_mode_active) {
    if (speed > PARKING_MODE_SPEED_LIMIT)
      speed = PARKING_MODE_SPEED_LIMIT;
    if (speed < -PARKING_MODE_SPEED_LIMIT)
      speed = -PARKING_MODE_SPEED_LIMIT;
  }

  speed = motor_bus_clamp_speed(speed);

  if (speed == last_cmd)
    return;
  last_cmd = speed;

  int actual_speed = speed * dir_sign;
  int pwm = motor_bus_speed_to_pwm(actual_speed);
  bus_send_motor_cmd(id, pwm);
}

void setup_motor(void) {
  motor1_speed = 0;
  motor2_speed = 0;

  last_lf_cmd = 12345;
  last_rf_cmd = 12345;
  last_lr_cmd = 12345;
  last_rr_cmd = 12345;
}

// 保留旧接口，兼容主程序里可能的调用
void motor1_SetSpeed(int Speed) {
  set_one_bus_motor(LEFT_FRONT_ID, Speed, LEFT_FRONT_DIR, last_lf_cmd);
  set_one_bus_motor(LEFT_REAR_ID, Speed, LEFT_REAR_DIR, last_lr_cmd);
}

void motor2_SetSpeed(int Speed) {
  set_one_bus_motor(RIGHT_FRONT_ID, Speed, RIGHT_FRONT_DIR, last_rf_cmd);
  set_one_bus_motor(RIGHT_REAR_ID, Speed, RIGHT_REAR_DIR, last_rr_cmd);
}

// 新接口：直接控制四个轮子
void motor_set_wheels(int lf, int rf, int lr, int rr) {
  set_one_bus_motor(LEFT_FRONT_ID, lf, LEFT_FRONT_DIR, last_lf_cmd);
  set_one_bus_motor(RIGHT_FRONT_ID, rf, RIGHT_FRONT_DIR, last_rf_cmd);
  set_one_bus_motor(LEFT_REAR_ID, lr, LEFT_REAR_DIR, last_lr_cmd);
  set_one_bus_motor(RIGHT_REAR_ID, rr, RIGHT_REAR_DIR, last_rr_cmd);
}

void motor_stop_all(void) { motor_set_wheels(0, 0, 0, 0); }
