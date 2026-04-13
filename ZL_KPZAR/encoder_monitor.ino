const unsigned long ENCODER_MONITOR_INTERVAL_MS = 500;
const unsigned long ENCODER_MONITOR_SAMPLE_MS = 80;
const unsigned long ENCODER_MONITOR_MIN_TRANSITIONS = 4;

void loop_encoder_monitor(void) {
  static unsigned long last_print_ms = 0;
  if (millis() - last_print_ms < ENCODER_MONITOR_INTERVAL_MS) {
    return;
  }
  last_print_ms = millis();

  bool ok = encoder_check_encode_decode(ENCODER_MONITOR_PIN_A,
                                        ENCODER_MONITOR_PIN_B,
                                        ENCODER_MONITOR_SAMPLE_MS,
                                        ENCODER_MONITOR_MIN_TRANSITIONS);
  Serial.print(" ENC_OK=");
  Serial.println(ok ? 1 : 0);
}
