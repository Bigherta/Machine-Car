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
	int init_status = ps2.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT); // 初始化
	ps2.read_gamepad();
    switch (init_status) {
        case 0:
            uart_send_str("PS2控制器连接成功！\r\n");
            break;
        case 1:
            uart_send_str("错误：未检测到PS2控制器！\r\n");
            break;
        case 2:
            uart_send_str("错误：PS2通讯失败（引脚/线路问题）！\r\n");
            break;
        case 3:
            uart_send_str("错误：控制器类型不兼容（非DualShock）！\r\n");
            break;
        default:
            uart_send_str("未知错误！\r\n");
            break;
    }
}
