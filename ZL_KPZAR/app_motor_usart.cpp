#include "app_motor_usart.hpp"
#include "bsp_motor_usart.hpp"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// UNO SRAM 很小。驱动板回传的有效包通常很短：
// MTEP:-123,-123,-123,-123 这类长度远小于 80。
// 从 256 降到 80，可以直接节省 352B 全局 SRAM。
#define RXBUFF_LEN 80

#define MOTOR_TX_MONITOR_MODE 0

static uint8_t send_buff[50];

float g_Speed[4];
int Encoder_Offset[4];
int Encoder_Now[4];

float g_mtep_counts_per_second[4] = {0, 0, 0, 0};
unsigned long g_mtep_last_update_ms = 0;

uint8_t g_recv_flag;
uint8_t g_recv_buff[RXBUFF_LEN];
uint8_t g_recv_buff_deal[RXBUFF_LEN];

// 只保留很小的接收状态量；不再保存 raw packet 字符串，节省 64B SRAM。
uint32_t g_motor_rx_packet_count = 0;
uint8_t g_motor_rx_last_type = 0;  // 0=none, 1=MAll, 2=MTEP, 3=MSPD
unsigned long g_motor_rx_last_ms = 0;

static void motor_send_buffer(void) {
  Send_Motor_ArrayU8(send_buff, strlen((char *)send_buff));
}

void send_motor_type(motor_type_t data) {
  snprintf((char *)send_buff, sizeof(send_buff), "$mtype:%d#", data);
  motor_send_buffer();
}

void send_motor_deadzone(uint16_t data) {
  snprintf((char *)send_buff, sizeof(send_buff), "$deadzone:%u#", data);
  motor_send_buffer();
}

void send_pulse_line(uint16_t data) {
  snprintf((char *)send_buff, sizeof(send_buff), "$mline:%u#", data);
  motor_send_buffer();
}

void send_pulse_phase(uint16_t data) {
  snprintf((char *)send_buff, sizeof(send_buff), "$mphase:%u#", data);
  motor_send_buffer();
}

void send_wheel_diameter(float data) {
  char float_str[10];
  dtostrf(data, 6, 3, float_str);
  snprintf((char *)send_buff, sizeof(send_buff), "$wdiameter:%s#", float_str);
  motor_send_buffer();
}

void send_motor_PID(float P, float I, float D) {
  // 协议表中 PID 配置命令为大写 $MPID。
  char p_str[10], i_str[10], d_str[10];
  dtostrf(P, 1, 3, p_str);
  dtostrf(I, 1, 3, i_str);
  dtostrf(D, 1, 3, d_str);
  snprintf((char *)send_buff, sizeof(send_buff), "$MPID:%s,%s,%s#", p_str, i_str, d_str);
  motor_send_buffer();
}

void send_upload_data(bool all_encoder_switch, bool ten_encoder_switch,
                      bool speed_switch) {
  snprintf((char *)send_buff, sizeof(send_buff), "$upload:%d,%d,%d#",
           all_encoder_switch ? 1 : 0,
           ten_encoder_switch ? 1 : 0,
           speed_switch ? 1 : 0);
  motor_send_buffer();
}

void Contrl_Speed(int16_t M1_speed, int16_t M2_speed, int16_t M3_speed,
                  int16_t M4_speed) {
  snprintf((char *)send_buff, sizeof(send_buff), "$spd:%d,%d,%d,%d#",
           M1_speed, M2_speed, M3_speed, M4_speed);
#if MOTOR_TX_MONITOR_MODE
  Serial.println((char *)send_buff);
#else
  motor_send_buffer();
#endif
}

void Contrl_Pwm(int16_t M1_pwm, int16_t M2_pwm, int16_t M3_pwm,
                int16_t M4_pwm) {
  snprintf((char *)send_buff, sizeof(send_buff), "$pwm:%d,%d,%d,%d#",
           M1_pwm, M2_pwm, M3_pwm, M4_pwm);
  motor_send_buffer();
}

static void skip_spaces(const char **p) {
  while (**p == ' ' || **p == '\t' || **p == '\r' || **p == '\n') {
    (*p)++;
  }
}

static bool parse_int4(const char *p, int out[4]) {
  for (uint8_t i = 0; i < 4; i++) {
    skip_spaces(&p);
    char *end_ptr = NULL;
    long value = strtol(p, &end_ptr, 10);
    if (end_ptr == p) return false;
    out[i] = (int)value;
    p = end_ptr;
    skip_spaces(&p);
    if (i < 3) {
      if (*p != ',') return false;
      p++;
    }
  }
  return true;
}

static bool parse_float4(const char *p, float out[4]) {
  for (uint8_t i = 0; i < 4; i++) {
    skip_spaces(&p);
    char *end_ptr = NULL;
    double value = strtod(p, &end_ptr);
    if (end_ptr == p) return false;
    out[i] = (float)value;
    p = end_ptr;
    skip_spaces(&p);
    if (i < 3) {
      if (*p != ',') return false;
      p++;
    }
  }
  return true;
}

void Deal_Control_Rxtemp(uint8_t rxtemp) {
  static uint8_t step = 0;
  static uint8_t start_flag = 0;

  if (rxtemp == '$' && start_flag == 0) {
    start_flag = 1;
    step = 0;
    g_recv_buff[0] = '\0';
    return;
  }

  if (start_flag != 1) return;

  if (rxtemp == '#') {
    start_flag = 0;
    g_recv_buff[step] = '\0';

    uint8_t packet_type = 0;
    if (strncmp("MAll:", (char *)g_recv_buff, 5) == 0) {
      packet_type = 1;
    } else if (strncmp("MTEP:", (char *)g_recv_buff, 5) == 0) {
      packet_type = 2;
    } else if (strncmp("MSPD:", (char *)g_recv_buff, 5) == 0) {
      packet_type = 3;
    }

    if (packet_type != 0) {
      strncpy((char *)g_recv_buff_deal, (char *)g_recv_buff, RXBUFF_LEN - 1);
      g_recv_buff_deal[RXBUFF_LEN - 1] = '\0';
      g_motor_rx_packet_count++;
      g_motor_rx_last_type = packet_type;
      g_motor_rx_last_ms = millis();
      g_recv_flag = 1;
    }
    step = 0;
    return;
  }

  if (step >= RXBUFF_LEN - 1) {
    start_flag = 0;
    step = 0;
    g_recv_buff[0] = '\0';
  } else {
    g_recv_buff[step++] = rxtemp;
  }
}

void Deal_data_real(void) {
  const char *packet = (const char *)g_recv_buff_deal;
  const char *payload = packet + 5;

  if (strncmp("MAll", packet, 4) == 0) {
    int values[4];
    if (!parse_int4(payload, values)) return;
    for (uint8_t i = 0; i < 4; i++) {
      Encoder_Now[i] = values[i];
    }
  } else if (strncmp("MTEP", packet, 4) == 0) {
    int values[4];
    if (!parse_int4(payload, values)) return;
    for (uint8_t i = 0; i < 4; i++) {
      Encoder_Offset[i] = values[i];
      g_mtep_counts_per_second[i] = (float)values[i] * 100.0f;
    }
    g_mtep_last_update_ms = millis();
  } else if (strncmp("MSPD", packet, 4) == 0) {
    float values[4];
    if (!parse_float4(payload, values)) return;
    for (uint8_t i = 0; i < 4; i++) {
      g_Speed[i] = values[i];
    }
  }
}
