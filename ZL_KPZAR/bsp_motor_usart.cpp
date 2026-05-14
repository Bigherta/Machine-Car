#include "bsp_motor_usart.hpp"
#include "app_motor_usart.hpp"

#define MOTOR_SERIAL Serial

void Usart_init(void) { MOTOR_SERIAL.begin(115200); }

void Send_Motor_U8(uint8_t data) { MOTOR_SERIAL.write(data); }

void Send_Motor_ArrayU8(uint8_t *pData, uint16_t length) {
  while (length--) {
    Send_Motor_U8(*pData);
    pData++;
  }
}

void Motor_USART_Recieve(void) {
  if (MOTOR_SERIAL.available()) {
    char rx_temp = (char)MOTOR_SERIAL.read();
    Deal_Control_Rxtemp((uint8_t)rx_temp);
  }
}
