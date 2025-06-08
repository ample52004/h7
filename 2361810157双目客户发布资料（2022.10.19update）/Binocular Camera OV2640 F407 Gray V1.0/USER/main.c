
//��Զ�ɵ��ӿƼ� QYF-NanoRiceF4������
//���ߣ�����ԭ�� @ALIENTEK 
//�ıࣺH.Q

#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "lcd.h" 
#include "timer.h" 
#include "ov2640.h" 
#include "dcmi.h" 
#include "sram.h"	

u16 Parallel_Data_Buffer1[38400] __attribute__((at(0X68000000)));//����������
u16 Parallel_Data_Buffer2[38400] __attribute__((at(0X68000000)));//����������
u16 display[76800] __attribute__((at(0X68000000)));//�Դ棬��4��dma�������

int zhencount=0;
volatile u32 jpeg_data_len=0; 			//buf�е�JPEG��Ч���ݳ��� 
 
//RGB565����
//RGB����ֱ����ʾ��LCD����
void rgb565_test(void)
{ 
	LCD_Clear(WHITE);
  POINT_COLOR=RED; 
	LCD_ShowString(30,50,200,16,16,"QiYuanFei NanoRiceF4");	
	delay_ms(1000);
	OV2640_RGB565_Mode();	//RGB565ģʽ
	OV2640_Special_Effects(2);
	OV2640_Light_Mode(1);
	OV2640_Auto_Exposure(4);
	My_DCMI_Init();			//DCMI����
	DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, ENABLE);
	DCMI_DMA_Init((u32)Parallel_Data_Buffer1,38400,DMA_MemoryDataSize_HalfWord,DMA_MemoryInc_Enable);//DCMI DMA����  
	OV2640_OutSize_Set(240,160);
	LCD_Set_Window(0,0,240,320);
	LCD_WriteRAM_Prepare();     		//��ʼд��GRAM	 	  
	DCMI_Start(); 		//��������
	delay_ms(10);		   
} 
int main(void)
{ 

	FSMC_SRAM_Init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);  //��ʼ����ʱ����
	uart_init(115200);		//��ʼ�����ڲ�����Ϊ115200
	LED_Init();					//��ʼ��LED 
 	LCD_Init();					//LCD��ʼ��  
 	POINT_COLOR=RED;//��������Ϊ��ɫ 
	LCD_ShowString(30,70,200,16,16,"QiYuanFei NanoRiceF4");	
	LCD_ShowString(30,90,200,16,16,"Binocular Camera");  
  LCD_ShowString(30,90,200,16,16,"2018.09.01");	
	delay_ms(2000);
	while(OV2640_Init())//��ʼ��OV2640
	{
		LCD_ShowString(30,130,240,16,16,"OV2640 ERR");
		delay_ms(200);
	  LCD_Fill(30,130,239,170,WHITE);
		delay_ms(200);
	}
	LCD_ShowString(30,130,200,16,16,"OV2640 OK");  
	rgb565_test();	  
	while(1)
	{
	}
	

}
