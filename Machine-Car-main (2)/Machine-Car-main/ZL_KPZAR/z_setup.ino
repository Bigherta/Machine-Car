/****************************************************************************
  初始化相关
****************************************************************************/

bool g_ps2_link_ok = false;

void setup_uart(void) {
  uart_init(115200);
  delay(100);
}

void setup_ps2(void) {
  delay(50);
  ps2_mode = ps2.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT);
  delay(50);
  ps2.read_gamepad();

  g_ps2_link_ok = (ps2_mode == 0);
}