#ifndef APP_MOTOR_USART_H
#define APP_MOTOR_USART_H

#include <Arduino.h>
#include <stdint.h>

typedef enum motor_type_t {
  MOTOR_TYPE_NONE = 0,
  MOTOR_520,
  MOTOR_310,
  MOTOR_TT_Encoder,
  MOTOR_TT,
  Motor_TYPE_MAX
} motor_type_t;

extern int Encoder_Offset[4];
extern int Encoder_Now[4];
extern float g_Speed[4];
extern float g_mtep_counts_per_second[4];
extern unsigned long g_mtep_last_update_ms;
extern uint8_t g_recv_flag;
extern uint8_t g_recv_buff_deal[];
extern uint32_t g_motor_rx_packet_count;
extern uint8_t g_motor_rx_last_type;  // 0=none, 1=MAll, 2=MTEP, 3=MSPD
extern unsigned long g_motor_rx_last_ms;

void send_motor_type(motor_type_t data);
void send_motor_deadzone(uint16_t data);
void send_pulse_line(uint16_t data);
void send_pulse_phase(uint16_t data);
void send_wheel_diameter(float data);
void send_motor_PID(float P, float I, float D);
void send_upload_data(bool all_encoder_switch, bool ten_encoder_switch,
                      bool speed_switch);
void Contrl_Speed(int16_t M1_speed, int16_t M2_speed, int16_t M3_speed,
                  int16_t M4_speed);
void Contrl_Pwm(int16_t M1_pwm, int16_t M2_pwm, int16_t M3_pwm,
                int16_t M4_pwm);

void Deal_Control_Rxtemp(uint8_t rxtemp);
void Deal_data_real(void);

#endif
