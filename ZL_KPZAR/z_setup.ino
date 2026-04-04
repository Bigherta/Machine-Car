/****************************************************************************
	*	@笔者	：	W
	*	@日期	：	2019年12月28日
	*	@所属	：	杭州众灵科技
	*	@论坛	：	www.ZL-robot.com
	*	@功能	：	存放初始化相关的函数
	*	@函数列表：
	*	1.	void setup_uart(void) -- 初始化串口
	*	2.	void setup_ps2(void) -- 初始化ps2
 ****************************************************************************/

/***********************************************
	函数名称:		setup_uart() 
	功能介绍:		初始化串口
	函数参数:		无
	返回值:			无
 ***********************************************/
void setup_uart(void) {
	uart_init(115200);
	uart_send_str((u8 *)"uart check ok!\r\n");
}

/***********************************************
	函数名称:		setup_ps2() 
	功能介绍:		初始化ps2
	函数参数:		无
	返回值:			无
 ***********************************************/
void setup_ps2(void) {
	ps2.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT);
	ps2.read_gamepad();
	int init_status = ps2.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT); // 初始化
    switch (init_status) {
        case 0:
            uart_send_str("PS2 controller connected successfully!\r\n");
            break;
        case 1:
            uart_send_str("Error: No PS2 controller detected!\r\n");
            break;
        case 2:
            uart_send_str("Error: PS2 communication failed (pin/wiring issue)!\r\n");
            break;
        case 3:
            uart_send_str("Error: Incompatible controller type (not DualShock)!\r\n");
            break;
        default:
            uart_send_str("Unknown error!\r\n");
            break;
    }
}