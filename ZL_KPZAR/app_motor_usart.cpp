#include "app_motor_usart.hpp"
#include "bsp_motor_usart.hpp"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// v5：默认使用原始协议格式发送，不再额外追加 \r\n。
// 串口监视器仍然能看到 Arduino 发出的 $spd/$pwm 文本，只是它们会连在一起显示。
// 这样可以排除 CR/LF 影响驱动板解析的可能。
#define MOTOR_TX_MONITOR_MODE 0

#define RXBUFF_LEN 256

static uint8_t send_buff[50];

float g_Speed[4];
int Encoder_Offset[4];
int Encoder_Now[4];

// MTEP 是驱动板上传的 10ms 编码器增量。
// 因此 counts/s ≈ MTEP * 100。
// 这里先保存“编码器计数速度”，不依赖轮径/减速比，最适合作为第一步速度反馈。
float g_mtep_counts_per_second[4] = {0, 0, 0, 0};
unsigned long g_mtep_last_update_ms = 0;

uint8_t g_recv_flag;
uint8_t g_recv_buff[RXBUFF_LEN];
uint8_t g_recv_buff_deal[RXBUFF_LEN];

// 编码器反馈诊断变量：用于判断 Arduino 是否真的收到了驱动板回传包。
// 0=none, 1=MAll, 2=MTEP, 3=MSPD
uint32_t g_motor_rx_packet_count = 0;
uint8_t g_motor_rx_last_type = 0;
unsigned long g_motor_rx_last_ms = 0;
char g_motor_rx_last_packet[64] = {0};

void send_motor_type(motor_type_t data) {
  sprintf((char *)send_buff, "$mtype:%d#", data);
  Send_Motor_ArrayU8(send_buff, strlen((char *)send_buff));
}

void send_motor_deadzone(uint16_t data) {
  sprintf((char *)send_buff, "$deadzone:%d#", data);
  Send_Motor_ArrayU8(send_buff, strlen((char *)send_buff));
}

void send_pulse_line(uint16_t data) {
  sprintf((char *)send_buff, "$mline:%d#", data);
  Send_Motor_ArrayU8(send_buff, strlen((char *)send_buff));
}

void send_pulse_phase(uint16_t data) {
  sprintf((char *)send_buff, "$mphase:%d#", data);
  Send_Motor_ArrayU8(send_buff, strlen((char *)send_buff));
}

void send_wheel_diameter(float data) {
  char float_str[10];
  dtostrf(data, 6, 3, float_str);
  sprintf((char *)send_buff, "$wdiameter:%s#", float_str);
  Send_Motor_ArrayU8(send_buff, strlen((char *)send_buff));
}

void send_motor_PID(float P, float I, float D) {
  // 协议表中 PID 配置命令为大写 $MPID。
  sprintf((char *)send_buff, "$MPID:%.3f,%.3f,%.3f#", P, I, D);
  Send_Motor_ArrayU8(send_buff, strlen((char *)send_buff));
}

void send_upload_data(bool all_encoder_switch, bool ten_encoder_switch,
                      bool speed_switch) {
  sprintf((char *)send_buff, "$upload:%d,%d,%d#", all_encoder_switch,
          ten_encoder_switch, speed_switch);
  Send_Motor_ArrayU8(send_buff, strlen((char *)send_buff));
}

void Contrl_Speed(int16_t M1_speed, int16_t M2_speed, int16_t M3_speed,
                  int16_t M4_speed) {
  sprintf((char *)send_buff, "$spd:%d,%d,%d,%d#", M1_speed, M2_speed,
          M3_speed, M4_speed);

#if MOTOR_TX_MONITOR_MODE
  Serial.println((char *)send_buff);
#else
  Send_Motor_ArrayU8(send_buff, strlen((char *)send_buff));
#endif
}

void Contrl_Pwm(int16_t M1_pwm, int16_t M2_pwm, int16_t M3_pwm,
                int16_t M4_pwm) {
  sprintf((char *)send_buff, "$pwm:%d,%d,%d,%d#", M1_pwm, M2_pwm, M3_pwm,
          M4_pwm);
  Send_Motor_ArrayU8(send_buff, strlen((char *)send_buff));
}

static bool isValidNumbers(const char *str) {
  // 允许数字前后有空格/回车/换行，兼容驱动板可能附带的格式字符。
  for (int i = 0; i < 4; i++) {
    while (*str == ' ' || *str == '\t' || *str == '\r' || *str == '\n') str++;

    if (*str == '-') {
      str++;
    }

    bool has_digit = false;
    while (isdigit(*str) || *str == '.') {
      if (isdigit(*str)) has_digit = true;
      str++;
    }
    if (!has_digit) {
      return false;
    }

    while (*str == ' ' || *str == '\t' || *str == '\r' || *str == '\n') str++;

    if (i < 3) {
      if (*str != ',') return false;
      str++;
    }
  }

  while (*str == ' ' || *str == '\t' || *str == '\r' || *str == '\n') str++;
  return *str == '\0';
}

static void splitString(char *mystrArray[], char *str, const char *delimiter) {
  char *token = strtok(str, delimiter);
  mystrArray[0] = token;
  int i = 1;

  while (token != NULL) {
    token = strtok(NULL, delimiter);
    mystrArray[i] = token;
    i++;
  }
}

void Deal_Control_Rxtemp(uint8_t rxtemp) {
  static uint8_t step = 0;
  static uint8_t start_flag = 0;

  if (rxtemp == '$' && start_flag == 0) {
    start_flag = 1;
    memset(g_recv_buff, 0, RXBUFF_LEN);

  } else if (start_flag == 1) {
    if (rxtemp == '#') {
      start_flag = 0;
      step = 0;

      // 只有收到完整且合法的编码器/速度反馈包，才置位 g_recv_flag。
      // 这样可以避免无效串口字符导致 Deal_data_real() 重复解析旧数据。
      uint8_t packet_type = 0;
      if (strncmp("MAll:", (char *)g_recv_buff, 5) == 0) {
        packet_type = 1;
      } else if (strncmp("MTEP:", (char *)g_recv_buff, 5) == 0) {
        packet_type = 2;
      } else if (strncmp("MSPD:", (char *)g_recv_buff, 5) == 0) {
        packet_type = 3;
      }

      if (packet_type != 0) {
        if (isValidNumbers((char *)g_recv_buff + 5)) {
          memcpy(g_recv_buff_deal, g_recv_buff, RXBUFF_LEN);

          g_motor_rx_packet_count++;
          g_motor_rx_last_type = packet_type;
          g_motor_rx_last_ms = millis();
          strncpy(g_motor_rx_last_packet, (char *)g_recv_buff, sizeof(g_motor_rx_last_packet) - 1);
          g_motor_rx_last_packet[sizeof(g_motor_rx_last_packet) - 1] = '\0';

          g_recv_flag = 1;
        }
      } else {
        memset(g_recv_buff, 0, RXBUFF_LEN);
      }
    } else {
      // 预留一个 '\0' 位置，防止接收异常长数据时越界。
      if (step >= RXBUFF_LEN - 1) {
        start_flag = 0;
        step = 0;
        memset(g_recv_buff, 0, RXBUFF_LEN);
      } else {
        g_recv_buff[step] = rxtemp;
        step++;
      }
    }
  }
}

void Deal_data_real(void) {
  static uint8_t data[RXBUFF_LEN];
  uint8_t length = 0;

  if ((strncmp("MAll", (char *)g_recv_buff_deal, 4) == 0)) {
    length = strlen((char *)g_recv_buff_deal) - 5;
    for (uint8_t i = 0; i < length; i++) {
      data[i] = g_recv_buff_deal[i + 5];
    }
    data[length] = '\0';

    char *strArray[10];
    char mystr_temp[4][10] = {'\0'};
    splitString(strArray, (char *)data, ", ");
    for (int i = 0; i < 4; i++) {
      if (strArray[i] == NULL) return;
      strcpy(mystr_temp[i], strArray[i]);
      Encoder_Now[i] = atoi(mystr_temp[i]);
    }
  } else if ((strncmp("MTEP", (char *)g_recv_buff_deal, 4) == 0)) {
    length = strlen((char *)g_recv_buff_deal) - 5;
    for (uint8_t i = 0; i < length; i++) {
      data[i] = g_recv_buff_deal[i + 5];
    }
    data[length] = '\0';

    char *strArray[10];
    char mystr_temp[4][10] = {'\0'};
    splitString(strArray, (char *)data, ", ");
    for (int i = 0; i < 4; i++) {
      if (strArray[i] == NULL) return;
      strcpy(mystr_temp[i], strArray[i]);
      Encoder_Offset[i] = atoi(mystr_temp[i]);
      g_mtep_counts_per_second[i] = (float)Encoder_Offset[i] * 100.0f;
    }
    g_mtep_last_update_ms = millis();
  } else if ((strncmp("MSPD", (char *)g_recv_buff_deal, 4) == 0)) {
    length = strlen((char *)g_recv_buff_deal) - 5;
    for (uint8_t i = 0; i < length; i++) {
      data[i] = g_recv_buff_deal[i + 5];
    }
    data[length] = '\0';

    char *strArray[10];
    char mystr_temp[4][10] = {'\0'};
    splitString(strArray, (char *)data, ", ");
    for (int i = 0; i < 4; i++) {
      if (strArray[i] == NULL) return;
      strcpy(mystr_temp[i], strArray[i]);
      g_Speed[i] = atof(mystr_temp[i]);
    }
  }
}
