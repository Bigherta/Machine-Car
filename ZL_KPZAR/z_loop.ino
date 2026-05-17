/****************************************************************************
  循环读取 PS2 + 可选编码器速度显示
  注意：Serial 同时用于电机通信和电脑串口监视器；
  调试输出只在 SPEED_DEBUG_TO_SERIAL=1 时开启。
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

// ================= MTEP 实时速度反馈输出 =================
// v9 默认 motor.ino 中 UPLOAD_DATA=2，驱动板上传 $MTEP:...#。
// MTEP 是 10ms 编码器增量，程序已换算为 counts/s。
// 注意：Serial 同时接到电机驱动板，调试字符串也会发给驱动板；若控制异常请改为 0。
#define SPEED_DEBUG_TO_SERIAL 1
const unsigned long SPEED_DEBUG_INTERVAL_MS = 300;

void motor_get_wheel_mtep_cps(float *lf, float *rf, float *lr, float *rr);
float motor_get_average_abs_mtep_cps(void);
void motor_get_mtep_p_loop_debug(float target_cps[4], float feedback_cps[4],
                                 int base_pwm[4], int output_pwm[4],
                                 bool *loop_active, bool *feedback_fresh);
void motor_get_mtep_hold_debug(int hold_pwm[4], bool *hold_active, bool *feedback_fresh);

static const char *motor_rx_type_name(uint8_t t) {
  if (t == 1) return "MAll";
  if (t == 2) return "MTEP";
  if (t == 3) return "MSPD";
  return "NONE";
}

void loop_speed_debug(void) {
#if SPEED_DEBUG_TO_SERIAL
  static unsigned long last_debug_ms = 0;
  if (millis() - last_debug_ms < SPEED_DEBUG_INTERVAL_MS) return;
  last_debug_ms = millis();

  unsigned long age_ms = (g_motor_rx_packet_count == 0) ? 999999UL : (millis() - g_motor_rx_last_ms);

  Serial.print("RX cnt=");
  Serial.print(g_motor_rx_packet_count);
  Serial.print(" last=");
  Serial.print(motor_rx_type_name(g_motor_rx_last_type));
  Serial.print(" age_ms=");
  Serial.print(age_ms);

  Serial.print(" | MTEP=");
  Serial.print(Encoder_Offset[0]);
  Serial.print(',');
  Serial.print(Encoder_Offset[1]);
  Serial.print(',');
  Serial.print(Encoder_Offset[2]);
  Serial.print(',');
  Serial.print(Encoder_Offset[3]);

  float cps_lf, cps_rf, cps_lr, cps_rr;
  motor_get_wheel_mtep_cps(&cps_lf, &cps_rf, &cps_lr, &cps_rr);

  Serial.print(" | cps=");
  Serial.print(cps_lf);
  Serial.print(',');
  Serial.print(cps_rf);
  Serial.print(',');
  Serial.print(cps_lr);
  Serial.print(',');
  Serial.print(cps_rr);

  Serial.print(" | avg_abs_cps=");
  Serial.print(motor_get_average_abs_mtep_cps());

  float target_cps[4];
  float feedback_cps[4];
  int base_pwm[4];
  int output_pwm[4];
  bool loop_active = false;
  bool feedback_fresh = false;
  motor_get_mtep_p_loop_debug(target_cps, feedback_cps, base_pwm, output_pwm,
                              &loop_active, &feedback_fresh);

  Serial.print(" | Ploop=");
  Serial.print(loop_active ? "ON" : "OFF");
  Serial.print(feedback_fresh ? ",fresh" : ",stale");

  int hold_pwm[4];
  bool hold_active = false;
  bool hold_fresh = false;
  motor_get_mtep_hold_debug(hold_pwm, &hold_active, &hold_fresh);

  Serial.print(" | Hold=");
  Serial.print(hold_active ? "ON" : "OFF");
  Serial.print(hold_fresh ? ",fresh" : ",stale");
  Serial.print(" | hold_pwm=");
  Serial.print(hold_pwm[0]); Serial.print(',');
  Serial.print(hold_pwm[1]); Serial.print(',');
  Serial.print(hold_pwm[2]); Serial.print(',');
  Serial.print(hold_pwm[3]);

  Serial.print(" | base_pwm=");
  Serial.print(base_pwm[0]); Serial.print(',');
  Serial.print(base_pwm[1]); Serial.print(',');
  Serial.print(base_pwm[2]); Serial.print(',');
  Serial.print(base_pwm[3]);

  Serial.print(" | out_pwm=");
  Serial.print(output_pwm[0]); Serial.print(',');
  Serial.print(output_pwm[1]); Serial.print(',');
  Serial.print(output_pwm[2]); Serial.print(',');
  Serial.print(output_pwm[3]);

  Serial.print(" | target_cps=");
  Serial.print(target_cps[0]); Serial.print(',');
  Serial.print(target_cps[1]); Serial.print(',');
  Serial.print(target_cps[2]); Serial.print(',');
  Serial.print(target_cps[3]);

  Serial.print(" | raw=");
  Serial.println(g_motor_rx_last_packet);
#endif
}
