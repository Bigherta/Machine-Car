#ifndef BSP_MOTOR_USART_H
#define BSP_MOTOR_USART_H

#include <Arduino.h>
#include <stdint.h>

void Usart_init(void);
void Send_Motor_U8(uint8_t data);
void Send_Motor_ArrayU8(uint8_t *pData, uint16_t length);
void Motor_USART_Recieve(void);

#endif
