/****************************************************************************
  循环读取 PS2
  注意：不要再往 Serial 打调试字符串，
  因为 Serial 正在用于总线马达控制
****************************************************************************/

void loop_ps2(void) {
  static u32 systick_ms_bak = 0;

  if (millis() - systick_ms_bak >= 20) {
    systick_ms_bak = millis();
    ps2.read_gamepad();
  }
}