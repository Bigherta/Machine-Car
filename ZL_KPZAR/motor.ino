/****************************************************************************
  @功能  : 存放电机相关的函数（总线马达版本）
  @说明  : 保留原来的 motor1 / motor2 框架
           motor1 = 左侧两轮
           motor2 = 右侧两轮
****************************************************************************/

/************ 4个总线电机 ID（已按你实测结果填写） ************/
#define LEFT_FRONT_ID   0    // 第一块ZMotorD 左电机
#define RIGHT_FRONT_ID  1    // 第一块ZMotorD 右电机
#define LEFT_REAR_ID    8    // 第二块ZMotorD 左电机
#define RIGHT_REAR_ID   9    // 第二块ZMotorD 右电机

/************ 方向修正 ************/
#define LEFT_SIDE_DIR   1
#define RIGHT_SIDE_DIR  1

/************ 总线电机参数 ************/
#define BUS_PWM_STOP         1500
#define BUS_PWM_MIN_OFFSET    200
#define BUS_PWM_MAX_OFFSET    900

const int SPEED_CMD_INVALID = 12345;

static int last_left_speed_cmd  = SPEED_CMD_INVALID;
static int last_right_speed_cmd = SPEED_CMD_INVALID;
static int last_left_front_speed_cmd  = SPEED_CMD_INVALID;
static int last_right_front_speed_cmd = SPEED_CMD_INVALID;
static int last_left_rear_speed_cmd   = SPEED_CMD_INVALID;
static int last_right_rear_speed_cmd  = SPEED_CMD_INVALID;

/***********************************************
  函数名称: motor_bus_clamp_speed()
  功能介绍: 将速度限制在 -1000 ~ 1000
 ***********************************************/
static int motor_bus_clamp_speed(int speed) {
  if (speed < -1000) return -1000;
  if (speed > 1000)  return 1000;
  return speed;
}

/***********************************************
  函数名称: motor_bus_speed_to_pwm()
  功能介绍: 把 -1000~1000 映射成总线马达 PWM
            1500停止，>1500正转，<1500反转
 ***********************************************/
static int motor_bus_speed_to_pwm(int speed) {
  speed = motor_bus_clamp_speed(speed);

  if (speed == 0) return BUS_PWM_STOP;

  int sign = (speed > 0) ? 1 : -1;
  int mag  = abs(speed);

  long offset = BUS_PWM_MIN_OFFSET +
                (long)(BUS_PWM_MAX_OFFSET - BUS_PWM_MIN_OFFSET) * mag / 1000;

  int pwm = BUS_PWM_STOP + sign * (int)offset;

  if (pwm > 2500) pwm = 2500;
  if (pwm < 500)  pwm = 500;

  return pwm;
}

/***********************************************
  函数名称: bus_send_motor_cmd()
  功能介绍: 给一个总线电机发送命令
 ***********************************************/
static void bus_send_motor_cmd(u8 id, int pwm) {
  char cmd[20];
  sprintf(cmd, "#%03dP%04dT0000!", id, pwm);
  uart_send_str((u8 *)cmd);
  delay(3);
}

/***********************************************
  函数名称: set_one_bus_motor()
  功能介绍: 控制单个总线电机
 ***********************************************/
static void set_one_bus_motor(u8 id, int speed, int dir_sign) {
  int actual_speed = speed * dir_sign;
  int pwm = motor_bus_speed_to_pwm(actual_speed);
  bus_send_motor_cmd(id, pwm);
}

/***********************************************
  函数名称: setup_motor()
  功能介绍: 初始化电机相关变量
 ***********************************************/
void setup_motor(void) {
  motor1_speed = 0;
  motor2_speed = 0;
  last_left_speed_cmd  = SPEED_CMD_INVALID;
  last_right_speed_cmd = SPEED_CMD_INVALID;
  last_left_front_speed_cmd  = SPEED_CMD_INVALID;
  last_right_front_speed_cmd = SPEED_CMD_INVALID;
  last_left_rear_speed_cmd   = SPEED_CMD_INVALID;
  last_right_rear_speed_cmd  = SPEED_CMD_INVALID;
}

/***********************************************
  函数名称: motor4_SetSpeed()
  功能介绍: 控制四个轮子（麦克纳姆）
 ***********************************************/
void motor4_SetSpeed(int left_front_speed, int right_front_speed, int left_rear_speed, int right_rear_speed) {
  left_front_speed  = motor_bus_clamp_speed(left_front_speed);
  right_front_speed = motor_bus_clamp_speed(right_front_speed);
  left_rear_speed   = motor_bus_clamp_speed(left_rear_speed);
  right_rear_speed  = motor_bus_clamp_speed(right_rear_speed);

  if (left_front_speed != last_left_front_speed_cmd) {
    last_left_front_speed_cmd = left_front_speed;
    set_one_bus_motor(LEFT_FRONT_ID, left_front_speed, LEFT_SIDE_DIR);
  }
  if (right_front_speed != last_right_front_speed_cmd) {
    last_right_front_speed_cmd = right_front_speed;
    set_one_bus_motor(RIGHT_FRONT_ID, right_front_speed, RIGHT_SIDE_DIR);
  }
  if (left_rear_speed != last_left_rear_speed_cmd) {
    last_left_rear_speed_cmd = left_rear_speed;
    set_one_bus_motor(LEFT_REAR_ID, left_rear_speed, LEFT_SIDE_DIR);
  }
  if (right_rear_speed != last_right_rear_speed_cmd) {
    last_right_rear_speed_cmd = right_rear_speed;
    set_one_bus_motor(RIGHT_REAR_ID, right_rear_speed, RIGHT_SIDE_DIR);
  }

  if (left_front_speed == left_rear_speed) {
    last_left_speed_cmd = left_front_speed;
  } else {
    last_left_speed_cmd = SPEED_CMD_INVALID;
  }

  if (right_front_speed == right_rear_speed) {
    last_right_speed_cmd = right_front_speed;
  } else {
    last_right_speed_cmd = SPEED_CMD_INVALID;
  }
}

/***********************************************
  函数名称: motor1_SetSpeed()
  功能介绍: 控制左侧两轮
 ***********************************************/
void motor1_SetSpeed(int Speed) {
  Speed = motor_bus_clamp_speed(Speed);

  if (Speed == last_left_speed_cmd) return;
  last_left_speed_cmd = Speed;
  last_left_front_speed_cmd = Speed;
  last_left_rear_speed_cmd = Speed;

  set_one_bus_motor(LEFT_FRONT_ID, Speed, LEFT_SIDE_DIR);
  set_one_bus_motor(LEFT_REAR_ID,  Speed, LEFT_SIDE_DIR);
}

/***********************************************
  函数名称: motor2_SetSpeed()
  功能介绍: 控制右侧两轮
 ***********************************************/
void motor2_SetSpeed(int Speed) {
  Speed = motor_bus_clamp_speed(Speed);

  if (Speed == last_right_speed_cmd) return;
  last_right_speed_cmd = Speed;
  last_right_front_speed_cmd = Speed;
  last_right_rear_speed_cmd = Speed;

  set_one_bus_motor(RIGHT_FRONT_ID, Speed, RIGHT_SIDE_DIR);
  set_one_bus_motor(RIGHT_REAR_ID,  Speed, RIGHT_SIDE_DIR);
}
