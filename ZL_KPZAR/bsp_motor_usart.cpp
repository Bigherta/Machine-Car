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
  // 重要：不要用 while(Serial.available()) 一口气读到空。
  // 编码器驱动板开启 MSPD 实时上传后，串口可能持续进数据；
  // 如果这里长期占用 CPU，PS2 手柄读取就会被饿死，电机会一直执行上一条速度命令。
  // 因此每次 loop 只处理有限字节，保证主循环能及时回到 PS2 控制逻辑。
  const uint8_t MOTOR_RX_MAX_BYTES_PER_CALL = 32;
  const unsigned long MOTOR_RX_TIME_BUDGET_US = 1500;

  uint8_t bytes_read = 0;
  unsigned long start_us = micros();

  while (MOTOR_SERIAL.available() &&
         bytes_read < MOTOR_RX_MAX_BYTES_PER_CALL &&
         (unsigned long)(micros() - start_us) < MOTOR_RX_TIME_BUDGET_US) {
    char rx_temp = (char)MOTOR_SERIAL.read();
    Deal_Control_Rxtemp((uint8_t)rx_temp);
    bytes_read++;
  }
}
