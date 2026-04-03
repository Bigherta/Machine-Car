/****************************************************************************
	*	@笔者	：	W
	*	@日期	：	2019年12月28日
	*	@所属	：	杭州众灵科技
	*	@论坛	：	www.ZL-robot.com
	*	@功能	：	存放电机相关的函数
	*	@函数列表：
	*	1.	void motor1_SetSpeed(int Speed) -- 电机1速度控制
	*	2.	void motor2_SetSpeed(int Speed) -- 电机2速度控制
 ****************************************************************************/

/*******电机控制指令表*******/
#define motor1_p(x) analogWrite(3, x)
#define motor1_n(x) analogWrite(5, x)
#define motor2_p(x) analogWrite(6, x)
#define motor2_n(x) analogWrite(9, x)
void setup_motor(void) {
	motor1_speed = 0;
	motor2_speed = 0;
	motor1_SetSpeed(motor1_speed);
	motor2_SetSpeed(motor2_speed);
}  // 初始化电机速度为0
/***********************************************
	函数名称:		motor1_SetSpeed() 
	功能介绍:		电机1速度控制
	函数参数:		Speed 速度
	返回值:			无
 ***********************************************/
void motor1_SetSpeed(int Speed) {
	if (Speed == 0) {
		motor1_p(0);
		motor1_n(0);
	} else if (Speed > 0) {
		Speed = Speed / 5 + 55;
		motor1_p(Speed);
		motor1_n(0);
	} else {
		Speed = -1 * Speed / 5 + 55;
		motor1_p(0);
		motor1_n(Speed);
	}
}

/***********************************************
	函数名称:		motor2_SetSpeed() 
	功能介绍:		电机2速度控制
	函数参数:		Speed 速度
	返回值:			无
 ***********************************************/
void motor2_SetSpeed(int Speed) {
	if (Speed == 0) {
		motor2_p(0);
		motor2_n(0);
	} else if (Speed > 0) {
		Speed = Speed / 5 + 55;
		motor2_p(Speed);
		motor2_n(0);
	} else {
		Speed = -1 * Speed / 5 + 55;
		motor2_p(0);
		motor2_n(Speed);
	}
}
