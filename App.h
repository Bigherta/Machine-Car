#ifndef _APP_H_
#define _APP_H_

#define KEY 		A7
#define ADC_BAT		A6		//电池电压的AD检测通道

extern u32 gSystemTickCount;	//系统从启动到现在的毫秒数

//void DelayMs(uint16 ms);
int InitPS2();
void InitADC(void);
void InitTimer2(void);
void InitUart2(void);
void Uart1SendData(BYTE dat);
uint16 GetBatteryVoltage(void);
void ServoSetPluseAndTime(uint8 id,uint16 p,uint16 time);
void TaskRun(int ps2_ok);

#endif
