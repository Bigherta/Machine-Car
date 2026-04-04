/****************************************************************************
	*	@笔者	：	W
	*	@日期	：	2019年12月28日
	*	@所属	：	杭州众灵科技
	*	@论坛	：	www.ZL-robot.com
	*	@功能	：	存放永久循环执行的函数
	*	@函数列表：
	*	1.	void loop_ps2(u32 baud) -- 循环接收处理手柄数据
 ****************************************************************************/

  /***********************************************
	函数名称:		loop_ps2() 
	功能介绍:		循环接收处理手柄数据
	函数参数:		无
	返回值:			无
 ***********************************************/
  void
  loop_ps2(void) {
	static u32 systick_ms_bak = 0;
	static u32 last_right_x = 0, last_right_y = 0, last_left_x = 0, last_left_y = 0;
	ps2.read_gamepad();
	if (millis() - systick_ms_bak >= 20) {
		systick_ms_bak = millis();
#if 0	
			sprintf(cmd_return, "%02d,%02d,%02d,%02d\r\n", 
			PS2_RIGHT_X, PS2_RIGHT_Y, PS2_LEFT_X, PS2_LEFT_Y);
			uart_send_str(cmd_return);
#endif
		if (PS2_RIGHT_X != last_right_x) {
			last_right_x = PS2_RIGHT_X;
			uart_send_str((u8 *)"\r\nPS2_RIGHT_X:\t");
			uart_send_int(last_right_x);
		}
		if (PS2_RIGHT_Y != last_right_y) {
			last_right_y = PS2_RIGHT_Y;
			uart_send_str((u8 *)"\r\nPS2_RIGHT_Y:\t");
			uart_send_int(last_right_y);
		}
		if (PS2_LEFT_X != last_left_x) {
			last_left_x = PS2_LEFT_X;
			uart_send_str((u8 *)"\r\nPS2_LEFT_X:\t");
			uart_send_int(last_left_x);
		}
		if (PS2_LEFT_Y != last_left_y) {
			last_left_y = PS2_LEFT_Y;
			uart_send_str((u8 *)"\r\nPS2_LEFT_Y:\t");
			uart_send_int(last_left_y);
		}


		if (PS2_P_LEFT_UP) {
			uart_send_str((u8 *)"\r\nPS2_LEFT_UP is pressed!");
		} else if (PS2_P_LEFT_DOWN) {
			uart_send_str((u8 *)"\r\nPS2_LEFT_DOWN is pressed!");
		} else if (PS2_P_LEFT_LEFT) {
			uart_send_str((u8 *)"\r\nPS2_LEFT_LEFT is pressed!");
		} else if (PS2_P_LEFT_RIGHT) {
			uart_send_str((u8 *)"\r\nPS2_LEFT_RIGHT is pressed!");
		} else if (PS2_P_RIGHT_UP) {
			uart_send_str((u8 *)"\r\nPS2_RIGHT_UP is pressed!");
		} else if (PS2_P_RIGHT_DOWN) {
			uart_send_str((u8 *)"\r\nPS2_RIGHT_DOWN is pressed!");
		} else if (PS2_P_RIGHT_LEFT) {
			uart_send_str((u8 *)"\r\nPS2_RIGHT_LEFT is pressed!");
		} else if (PS2_P_RIGHT_RIGHT) {
			uart_send_str((u8 *)"\r\nPS2_RIGHT_RIGHT is pressed!");
		} else if (PS2_P_LEFT_1) {
			uart_send_str((u8 *)"\r\nPS2_LEFT_1 is pressed!");
		} else if (PS2_P_LEFT_2) {
			uart_send_str((u8 *)"\r\nPS2_LEFT_2 is pressed!");
		} else if (PS2_P_RIGHT_1) {
			uart_send_str((u8 *)"\r\nPS2_RIGHT_1 is pressed!");
		} else if (PS2_P_RIGHT_2) {
			uart_send_str((u8 *)"\r\nPS2_RIGHT_2 is pressed!");
		} else if (PS2_P_SELECT) {
			uart_send_str((u8 *)"\r\nPS2_SELECT is pressed!");
		} else if (PS2_P_START) {
			uart_send_str((u8 *)"\r\nPS2_START is pressed!");
		}

		if (PS2_R_LEFT_UP) {
			uart_send_str((u8 *)"\r\nPS2_LEFT_UP is released!");
		} else if (PS2_R_LEFT_DOWN) {
			uart_send_str((u8 *)"\r\nPS2_LEFT_DOWN is released!");
		} else if (PS2_R_LEFT_LEFT) {
			uart_send_str((u8 *)"\r\nPS2_LEFT_LEFT is released!");
		} else if (PS2_R_LEFT_RIGHT) {
			uart_send_str((u8 *)"\r\nPS2_LEFT_RIGHT is released!");
		} else if (PS2_R_RIGHT_UP) {
			uart_send_str((u8 *)"\r\nPS2_RIGHT_UP is released!");
		} else if (PS2_R_RIGHT_DOWN) {
			uart_send_str((u8 *)"\r\nPS2_RIGHT_DOWN is released!");
		} else if (PS2_R_RIGHT_LEFT) {
			uart_send_str((u8 *)"\r\nPS2_RIGHT_LEFT is released!");
		} else if (PS2_R_RIGHT_RIGHT) {
			uart_send_str((u8 *)"\r\nPS2_RIGHT_RIGHT is released!");
		} else if (PS2_R_LEFT_1) {
			uart_send_str((u8 *)"\r\nPS2_LEFT_1 is released!");
		} else if (PS2_R_LEFT_2) {
			uart_send_str((u8 *)"\r\nPS2_LEFT_2 is released!");
		} else if (PS2_R_RIGHT_1) {
			uart_send_str((u8 *)"\r\nPS2_RIGHT_1 is released!");
		} else if (PS2_R_RIGHT_2) {
			uart_send_str((u8 *)"\r\nPS2_RIGHT_2 is released!");
		} else if (PS2_R_SELECT) {
			uart_send_str((u8 *)"\r\nPS2_SELECT is released!");
		} else if (PS2_R_START) {
			uart_send_str((u8 *)"\r\nPS2_START is released!");
		}
	}
	delay(20);
}
