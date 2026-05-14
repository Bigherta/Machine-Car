#include "app_motor_usart.hpp"
#include "bsp_motor_usart.hpp"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RXBUFF_LEN 256

static uint8_t send_buff[50];

float g_Speed[4];
int Encoder_Offset[4];
int Encoder_Now[4];

uint8_t g_recv_flag;
uint8_t g_recv_buff[RXBUFF_LEN];
uint8_t g_recv_buff_deal[RXBUFF_LEN];

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
  sprintf((char *)send_buff, "$mpid:%.3f,%.3f,%.3f#", P, I, D);
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
  Send_Motor_ArrayU8(send_buff, strlen((char *)send_buff));
}

void Contrl_Pwm(int16_t M1_pwm, int16_t M2_pwm, int16_t M3_pwm,
                int16_t M4_pwm) {
  sprintf((char *)send_buff, "$pwm:%d,%d,%d,%d#", M1_pwm, M2_pwm, M3_pwm,
          M4_pwm);
  Send_Motor_ArrayU8(send_buff, strlen((char *)send_buff));
}

static bool isValidNumbers(const char *str) {
  for (int i = 0; i < 4; i++) {
    if (*str == '-') {
      str++;
    }

    if (!isdigit(*str) && *str != '.') {
      return false;
    }

    while (isdigit(*str) || *str == '.') {
      str++;
    }

    if (i < 3 && *str != ',') {
      return false;
    }

    if (i < 3) {
      str++;
    }
  }
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
      g_recv_flag = 1;

      if (strncmp("MAll:", (char *)g_recv_buff, 5) == 0 ||
          strncmp("MTEP:", (char *)g_recv_buff, 5) == 0 ||
          strncmp("MSPD:", (char *)g_recv_buff, 5) == 0) {
        if (isValidNumbers((char *)g_recv_buff + 5)) {
          memcpy(g_recv_buff_deal, g_recv_buff, RXBUFF_LEN);
        }
      } else {
        memset(g_recv_buff, 0, RXBUFF_LEN);
      }
    } else {
      if (step > RXBUFF_LEN) {
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
      strcpy(mystr_temp[i], strArray[i]);
      Encoder_Offset[i] = atoi(mystr_temp[i]);
    }
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
      strcpy(mystr_temp[i], strArray[i]);
      g_Speed[i] = atof(mystr_temp[i]);
    }
  }
}
