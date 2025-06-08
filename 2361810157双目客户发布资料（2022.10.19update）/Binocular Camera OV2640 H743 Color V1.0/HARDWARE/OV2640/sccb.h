#ifndef __SCCB_H
#define __SCCB_H
#include "sys.h"

//IO方向设置
#define SCCB_SDA_IN()  {GPIOD->MODER&=~(3<<(7*2));GPIOD->MODER|=0<<7*2;}	//PB3输入模式
#define SCCB_SDA_OUT() {GPIOD->MODER&=~(3<<(7*2));GPIOD->MODER|=1<<7*2;}    //PB3输出模式
//IO操作
#define SCCB_SCL(n)  (n?HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,GPIO_PIN_RESET)) //SCL
#define SCCB_SDA(n)  (n?HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_RESET)) //SDA

#define SCCB_READ_SDA    HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_7)     //输入SDA
#define SCCB_ID         0X60                                    //OV2640的ID

///////////////////////////////////////////
void SCCB_Init(void);
void SCCB_Start(void);
void SCCB_Stop(void);
void SCCB_No_Ack(void);
u8 SCCB_WR_Byte(u8 dat);
u8 SCCB_RD_Byte(void);
u8 SCCB_WR_Reg(u8 reg,u8 data);
u8 SCCB_RD_Reg(u8 reg);
#endif

