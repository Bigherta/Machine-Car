/****************************************************************************
	*	@笔者	：	W
	*	@日期	：	2019年12月28日
	*	@所属	：	杭州众灵科技
	*	@论坛	：	www.ZL-robot.com
	*	@功能	：	ZL-KPZ控制板（AR版）模块例程6————PS2手柄解析
	*	@函数列表：
	*	1.	void setup(void) -- 初始化函数
	*	2.	void loop(void) -- 主循环函数
 ****************************************************************************/

#include <PS2X_lib.h>  //ps2手柄库

/*******全局变量宏定义*******/
#define UART_RECEIVE_BUF_SIZE 100
#define ENABLE_KEY_INIT 0  // 0: 关闭按键初始化，避免占用 D6

/*******PS2管脚映射表*******/
#define PS2_DAT 12
#define PS2_CMD A0
#define PS2_ATT A3
#define PS2_CLK 11

/*******PS2模式数据表*******/
#define PS2_MODE_GRN 0x41
#define PS2_MODE_RED 0x73

/*******PS2按键检测表*******/
#define PS2_P_LEFT_UP ps2.ButtonPressed(PSB_PAD_UP)
#define PS2_P_LEFT_RIGHT ps2.ButtonPressed(PSB_PAD_RIGHT)
#define PS2_P_LEFT_DOWN ps2.ButtonPressed(PSB_PAD_DOWN)
#define PS2_P_LEFT_LEFT ps2.ButtonPressed(PSB_PAD_LEFT)

#define PS2_P_SELECT ps2.ButtonPressed(PSB_SELECT)
#define PS2_P_START ps2.ButtonPressed(PSB_START)

#define PS2_P_RIGHT_UP ps2.ButtonPressed(PSB_GREEN)
#define PS2_P_RIGHT_RIGHT ps2.ButtonPressed(PSB_RED)
#define PS2_P_RIGHT_DOWN ps2.ButtonPressed(PSB_BLUE)
#define PS2_P_RIGHT_LEFT ps2.ButtonPressed(PSB_PINK)

#define PS2_P_LEFT_2 ps2.ButtonPressed(PSB_L2)
#define PS2_P_RIGHT_2 ps2.ButtonPressed(PSB_R2)
#define PS2_P_LEFT_1 ps2.ButtonPressed(PSB_L1)
#define PS2_P_RIGHT_1 ps2.ButtonPressed(PSB_R1)

#define PS2_R_LEFT_UP ps2.ButtonReleased(PSB_PAD_UP)
#define PS2_R_LEFT_RIGHT ps2.ButtonReleased(PSB_PAD_RIGHT)
#define PS2_R_LEFT_DOWN ps2.ButtonReleased(PSB_PAD_DOWN)
#define PS2_R_LEFT_LEFT ps2.ButtonReleased(PSB_PAD_LEFT)

#define PS2_R_SELECT ps2.ButtonReleased(PSB_SELECT)
#define PS2_R_START ps2.ButtonReleased(PSB_START)

#define PS2_R_RIGHT_UP ps2.ButtonReleased(PSB_GREEN)
#define PS2_R_RIGHT_RIGHT ps2.ButtonReleased(PSB_RED)
#define PS2_R_RIGHT_DOWN ps2.ButtonReleased(PSB_BLUE)
#define PS2_R_RIGHT_LEFT ps2.ButtonReleased(PSB_PINK)

#define PS2_R_LEFT_2 ps2.ButtonReleased(PSB_L2)
#define PS2_R_RIGHT_2 ps2.ButtonReleased(PSB_R2)
#define PS2_R_LEFT_1 ps2.ButtonReleased(PSB_L1)
#define PS2_R_RIGHT_1 ps2.ButtonReleased(PSB_R1)

#define PS2_RIGHT_X_RAW (int)(ps2.Analog(PSS_RX))
#define PS2_RIGHT_Y_RAW (int)(ps2.Analog(PSS_RY))
#define PS2_LEFT_X_RAW (int)(ps2.Analog(PSS_LX))
#define PS2_LEFT_Y_RAW (int)(ps2.Analog(PSS_LY))

// 开机时记录零点，运行时用相对值，避免固定偏置导致某轴近似不变。
#define PS2_RIGHT_X (PS2_RIGHT_X_RAW - origin_right_x)
#define PS2_RIGHT_Y (PS2_RIGHT_Y_RAW - origin_right_y)
#define PS2_LEFT_X (PS2_LEFT_X_RAW - origin_left_x)
#define PS2_LEFT_Y (PS2_LEFT_Y_RAW - origin_left_y)

/*******全局变量定义*******/
u8 i = 0;
int motor1_speed = 0, motor2_speed = 0;
u8 uart_receive_buf[UART_RECEIVE_BUF_SIZE] = { 0 }, uart_receive_buf_index, uart_get_ok;
u8 ps2_mode = 0;
PS2X ps2;
int origin_left_x;
int origin_right_x;
int origin_left_y;
int origin_right_y;
void setup(void) {  //ZL
	setup_motor();
	setup_uart();  //初始化串口
	setup_ps2();
	origin_left_x = PS2_LEFT_X_RAW;
	origin_right_x = PS2_RIGHT_X_RAW;
	origin_left_y = PS2_LEFT_Y_RAW;
	origin_right_y = PS2_RIGHT_Y_RAW;
}
void loop(void) {
	loop_ps2();  //循环检测手柄状态
	loop_key();
	delay(20);
}