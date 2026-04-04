/****************************************************************************
	*	@笔者	：	W
	*	@日期	：	2019年12月28日
	*	@所属	：	杭州众灵科技
	*	@论坛	：	www.ZL-robot.com
	*	@功能	：	ZL-KPZ控制板（AR版）电机直驱测试
	*	@函数列表：
	*	1.	void setup(void) -- 初始化函数
	*	2.	void loop(void) -- 主循环函数
 ****************************************************************************/
#include <Arduino.h>
typedef unsigned char u8;
typedef unsigned long u32;

/*******全局变量宏定义*******/
#define UART_RECEIVE_BUF_SIZE 100

/*******全局变量定义*******/
u8 i = 0;
int motor1_speed = 0, motor2_speed = 0;
u8 uart_receive_buf[UART_RECEIVE_BUF_SIZE] = { 0 }, uart_receive_buf_index, uart_get_ok;

void setup(void) {  //ZL
	setup_motor();
	setup_uart();  //初始化串口
	key_init();  //初始化按键
}

void loop(void) {
	loop_key();
	delay(20);
}
