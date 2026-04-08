/****************************************************************************
  @功能  : 存放初始化相关的函数
****************************************************************************/

extern bool g_ps2_link_ok;
extern u32 g_ps2_last_ok_ms;

/***********************************************
  函数名称: setup_uart()
  功能介绍: 初始化串口
 ***********************************************/
void setup_uart(void) {
  uart_init(115200);
  delay(200);
}

/***********************************************
  函数名称: setup_ps2()
  功能介绍: 初始化 PS2 手柄
 ***********************************************/
void setup_ps2(void) {
  u8 retry = 0;
  delay(50);

  // 某些手柄/接收器上电后第一次握手不稳定，失败时重试几次。
  for (retry = 0; retry < 5; retry++) {
    ps2_mode = ps2.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT, true, true);
    delay(30);
    if (ps2_mode == 0) break;
  }

  // 先做一次刷新，后续在 loop_ps2 中持续读取。
  ps2.read_gamepad(false, 0);
  g_ps2_link_ok = (ps2_mode == 0);
  g_ps2_last_ok_ms = millis();
}
