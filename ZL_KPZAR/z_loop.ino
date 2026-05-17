/****************************************************************************
  循环读取 PS2
  注意：不要再往 Serial 打调试字符串，
  因为 Serial 正在用于总线马达控制
****************************************************************************/

unsigned long g_ps2_last_read_ms = 0;
bool g_ps2_frame_ready = false;

void loop_ps2(void) {
  static u32 systick_ms_bak = 0;

  // PS2 更新频率略提高，确保摇杆回中/换方向能尽快反映到电机命令。
  if (millis() - systick_ms_bak >= 15) {
    systick_ms_bak = millis();
    ps2.read_gamepad();
    g_ps2_last_read_ms = millis();
    g_ps2_frame_ready = true;
  }
}

// ================= 编码器实时速度监视 =================
// motor.ino 中 UPLOAD_DATA=3 时，驱动板会上传 $MSPD:...#，解析结果保存在 g_Speed[0..3]。
// 为避免调试字符串干扰电机串口，默认不向 Serial 输出。
// 如果你暂时需要在 Arduino 串口监视器看速度，可以把 0 改成 1；正常比赛/运行时建议保持 0。
#define SPEED_DEBUG_TO_SERIAL 0
const unsigned long SPEED_DEBUG_INTERVAL_MS = 200;

void loop_speed_debug(void) {
#if SPEED_DEBUG_TO_SERIAL
  static unsigned long last_debug_ms = 0;
  if (millis() - last_debug_ms < SPEED_DEBUG_INTERVAL_MS) return;
  last_debug_ms = millis();

  Serial.print("MSPD lf=");
  Serial.print(g_Speed[0]);
  Serial.print(" rf=");
  Serial.print(g_Speed[1]);
  Serial.print(" lr=");
  Serial.print(g_Speed[2]);
  Serial.print(" rr=");
  Serial.print(g_Speed[3]);
  Serial.print(" avg_abs=");
  Serial.println(motor_get_average_abs_speed());
#endif
}
