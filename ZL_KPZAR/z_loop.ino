/****************************************************************************
  @功能  : 存放永久循环执行的函数
****************************************************************************/

bool g_ps2_link_ok = false;
u32 g_ps2_last_ok_ms = 0;
const u32 PS2_COMM_TIMEOUT_MS = 300;

/***********************************************
  函数名称: loop_ps2()
  功能介绍: 周期读取手柄状态
 ***********************************************/
void loop_ps2(void) {
  static u32 systick_ms_bak = 0;
  u32 now_ms = millis();

  if (now_ms - systick_ms_bak >= 20) {
    systick_ms_bak = now_ms;
    ps2.read_gamepad();

    if (ps2.readType() != 0) {
      g_ps2_link_ok = true;
      g_ps2_last_ok_ms = now_ms;
    }
  }

  if (ps2_mode != 0) {
    g_ps2_link_ok = false;
    return;
  }

  if (now_ms - g_ps2_last_ok_ms > PS2_COMM_TIMEOUT_MS) {
    g_ps2_link_ok = false;
  }
}
