#include "sys.h"
#include "dcmi.h" 
#include "lcd.h" 
#include "ltdc.h"

extern DCMI_HandleTypeDef  hdcmi;           //DCMI���
extern DMA_HandleTypeDef   hdma_dcmi;        //DMA���
extern u16 Parallel_Data_Buffer1[38400] ;
extern u16 Parallel_Data_Buffer2[38400] ;

int zhencount;
extern u16 display[76800];



u8 ov_frame=0;  						//֡��
extern void jpeg_data_process(void);	//JPEG���ݴ�����


void DMA1_Stream1_IRQHandler(void)
{
	  long i=0;
    if(__HAL_DMA_GET_FLAG(&hdma_dcmi,DMA_FLAG_TCIF1_5)!=RESET)//DMA�������
    {
        __HAL_DMA_CLEAR_FLAG(&hdma_dcmi,DMA_FLAG_TCIF1_5);//���DMA��������жϱ�־λ
//        dcmi_rx_callback();	//ִ������ͷ���ջص�����,��ȡ���ݵȲ����������洦��
			zhencount++; //ͼ��ֶα�־������2��
		if(zhencount==1)
		{
			i=0;
			while(i< 38400)
			{
				display[i]=	Parallel_Data_Buffer1[i];
//				LCD_WriteRAM(display[i]);
				i++;				
			}
		}
		if(zhencount==2)
		{
			zhencount=0;
			i=0;
			while(i< 38400)
			{
				display[38400+i]=	Parallel_Data_Buffer2[i];	
//				LCD_WriteRAM(display[i]);
				i++;				
			}
		}
		//display�����ݿ�����ͼ����
		for(i=0;i<76800;i++)
		{
			LCD_WriteRAM(display[i]);
		} 	 
    } 
}


//DCMI��ʼ��
void DCMI_Init(void)
{
    hdcmi.Instance=DCMI;
    hdcmi.Init.SynchroMode=DCMI_SYNCHRO_HARDWARE;    //Ӳ��ͬ��HSYNC,VSYNC
    hdcmi.Init.PCKPolarity=DCMI_PCKPOLARITY_RISING;  //PCLK ��������Ч
    hdcmi.Init.VSPolarity=DCMI_VSPOLARITY_LOW;       //VSYNC �͵�ƽ��Ч
    hdcmi.Init.HSPolarity=DCMI_HSPOLARITY_LOW;       //HSYNC �͵�ƽ��Ч
    hdcmi.Init.CaptureRate=DCMI_CR_ALL_FRAME;        //ȫ֡����
    hdcmi.Init.ExtendedDataMode=DCMI_EXTEND_DATA_8B; //8λ���ݸ�ʽ 
    HAL_DCMI_Init(&hdcmi);                           //��ʼ��DCMI���˺����Ὺ��֡�ж�  

    //�ر����жϡ�VSYNC�жϡ�ͬ�������жϺ�����ж�
    //__HAL_DCMI_DISABLE_IT(&hdcmi,DCMI_IT_LINE|DCMI_IT_VSYNC|DCMI_IT_ERR|DCMI_IT_OVR);
	
	//�ر������жϣ�����HAL_DCMI_Init()��Ĭ�ϴ򿪺ܶ��жϣ�������Щ�ж�
	//�Ժ����Ǿ���Ҫ����Щ�ж�����Ӧ�Ĵ�������Ļ��ͻᵼ�¸��ָ��������⣬
	//������Щ�жϺܶ඼����Ҫ���������ｫ��ȫ���رյ���Ҳ���ǽ�IER�Ĵ������㡣
	//�ر��������ж��Ժ��ٸ����Լ���ʵ��������ʹ����Ӧ���жϡ�
	DCMI->IER=0x0;	
	
    __HAL_DCMI_ENABLE_IT(&hdcmi,DCMI_IT_FRAME);      //ʹ��֡�ж�
    __HAL_DCMI_ENABLE(&hdcmi);                       //ʹ��DCMI
}

//DCMI�ײ��������������ã�ʱ��ʹ�ܣ��ж�����
//�˺����ᱻHAL_DCMI_Init()����
//hdcmi:DCMI���
//void HAL_DCMI_MspInit(DCMI_HandleTypeDef* hdcmi)
//{
//    GPIO_InitTypeDef GPIO_Initure;
//    
//    __HAL_RCC_DCMI_CLK_ENABLE();                //ʹ��DCMIʱ��

//    __HAL_RCC_GPIOA_CLK_ENABLE();               //ʹ��GPIOAʱ��
//    __HAL_RCC_GPIOB_CLK_ENABLE();               //ʹ��GPIOBʱ��
//    __HAL_RCC_GPIOC_CLK_ENABLE();               //ʹ��GPIOCʱ��
//    __HAL_RCC_GPIOD_CLK_ENABLE();               //ʹ��GPIODʱ��
//    __HAL_RCC_GPIOH_CLK_ENABLE();               //ʹ��GPIOHʱ��
//    
//    //��ʼ��PA6
//    GPIO_Initure.Pin=GPIO_PIN_6;  
//    GPIO_Initure.Mode=GPIO_MODE_AF_PP;          //���츴��
//    GPIO_Initure.Pull=GPIO_PULLUP;              //����
//    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;       //����
//    GPIO_Initure.Alternate=GPIO_AF13_DCMI;      //����ΪDCMI   
//    HAL_GPIO_Init(GPIOA,&GPIO_Initure);         //��ʼ��
//    
//    //PB7,8,9
//    GPIO_Initure.Pin=GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;  
//    HAL_GPIO_Init(GPIOB,&GPIO_Initure);         //��ʼ��
//    
//    //PC6,7,8,9,11
//    GPIO_Initure.Pin=GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|\
//                     GPIO_PIN_9|GPIO_PIN_11;  
//    HAL_GPIO_Init(GPIOC,&GPIO_Initure);         //��ʼ��
//    
//    //PD3
//    GPIO_Initure.Pin=GPIO_PIN_3; 
//    HAL_GPIO_Init(GPIOD,&GPIO_Initure);         //��ʼ��
//    
//    //PH8
//    GPIO_Initure.Pin=GPIO_PIN_8; 
//    HAL_GPIO_Init(GPIOH,&GPIO_Initure);         //��ʼ��   
//    
//    HAL_NVIC_SetPriority(DCMI_IRQn,2,2);        //��ռ���ȼ�1�������ȼ�2
//    HAL_NVIC_EnableIRQ(DCMI_IRQn);              //ʹ��DCMI�ж�
//					
//}

//DCMI DMA����
//mem0addr:�洢����ַ0  ��Ҫ�洢����ͷ���ݵ��ڴ��ַ(Ҳ�����������ַ)
//mem1addr:�洢����ַ1  ��ֻʹ��mem0addr��ʱ��,��ֵ����Ϊ0
//memblen:�洢��λ��,����Ϊ:DMA_MDATAALIGN_BYTE/DMA_MDATAALIGN_HALFWORD/DMA_MDATAALIGN_WORD
//meminc:�洢��������ʽ,����Ϊ:DMA_MINC_ENABLE/DMA_MINC_DISABLE
void DCMI_DMA_Init(u32 mem0addr,u32 mem1addr,u16 memsize,u32 memblen,u32 meminc)
{ 
    __HAL_RCC_DMA1_CLK_ENABLE();                                    //ʹ��DMA1ʱ��
    __HAL_LINKDMA(&hdcmi,DMA_Handle,hdma_dcmi);        //��DMA��DCMI��ϵ����
	__HAL_DMA_DISABLE_IT(&hdma_dcmi,DMA_IT_TC);    			//�ȹر�DMA��������ж�(������ʹ��MCU����ʱ�����ֻ��������)
	
    hdma_dcmi.Instance=DMA1_Stream1;                          //DMA1������1    
	  hdma_dcmi.Init.Request=DMA_REQUEST_DCMI;					//DCMI��DMA����
    hdma_dcmi.Init.Direction=DMA_PERIPH_TO_MEMORY;            //���赽�洢��
    hdma_dcmi.Init.PeriphInc=DMA_PINC_DISABLE;                //���������ģʽ
    hdma_dcmi.Init.MemInc=meminc;                   			//�洢������ģʽ
    hdma_dcmi.Init.PeriphDataAlignment=DMA_PDATAALIGN_WORD;   //�������ݳ���:32λ
    hdma_dcmi.Init.MemDataAlignment=memblen;     				//�洢�����ݳ���:8/16/32λ
    hdma_dcmi.Init.Mode=DMA_CIRCULAR;                         //ʹ��ѭ��ģʽ 
    hdma_dcmi.Init.Priority=DMA_PRIORITY_HIGH;                //�����ȼ�
    hdma_dcmi.Init.FIFOMode=DMA_FIFOMODE_ENABLE;              //ʹ��FIFO
    hdma_dcmi.Init.FIFOThreshold=DMA_FIFO_THRESHOLD_HALFFULL; //ʹ��1/2��FIFO 
    hdma_dcmi.Init.MemBurst=DMA_MBURST_SINGLE;                //�洢��ͻ������
    hdma_dcmi.Init.PeriphBurst=DMA_PBURST_SINGLE;             //����ͻ�����δ��� 
    HAL_DMA_DeInit(&hdma_dcmi);                               //�������ǰ������
    HAL_DMA_Init(&hdma_dcmi);	                                //��ʼ��DMA
    
    //�ڿ���DMA֮ǰ��ʹ��__HAL_UNLOCK()����һ��DMA,��ΪHAL_DMA_Statrt()HAL_DMAEx_MultiBufferStart()
    //����������һ��ʼҪ��ʹ��__HAL_LOCK()����DMA,������__HAL_LOCK()���жϵ�ǰ��DMA״̬�Ƿ�Ϊ����״̬�������
    //����״̬�Ļ���ֱ�ӷ���HAL_BUSY�������ᵼ�º���HAL_DMA_Statrt()��HAL_DMAEx_MultiBufferStart()������DMA����
    //����ֱ�ӱ�������DMAҲ�Ͳ�������������Ϊ�˱���������������������DMA֮ǰ�ȵ���__HAL_UNLOC()�Ƚ���һ��DMA��
    __HAL_UNLOCK(&hdma_dcmi);
    if(mem1addr==0)    //����DMA����ʹ��˫����
    {
        HAL_DMA_Start(&hdma_dcmi,(u32)&DCMI->DR,mem0addr,memsize);
    }
    else                //ʹ��˫����
    {
        HAL_DMAEx_MultiBufferStart(&hdma_dcmi,(u32)&DCMI->DR,mem0addr,mem1addr,memsize);//����˫����
        __HAL_DMA_ENABLE_IT(&hdma_dcmi,DMA_IT_TC);    //������������ж�
        HAL_NVIC_SetPriority(DMA1_Stream1_IRQn,2,3);        //DMA�ж����ȼ�
        HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    }    
}

//DCMI,��������
void DCMI_Start(void)
{  
    LCD_SetCursor(0,0);  
	LCD_WriteRAM_Prepare();		        //��ʼд��GRAM
    __HAL_DMA_ENABLE(&hdma_dcmi); //ʹ��DMA
    DCMI->CR|=DCMI_CR_CAPTURE;          //DCMI����ʹ��
}

//DCMI,�رմ���
void DCMI_Stop(void)
{ 
    DCMI->CR&=~(DCMI_CR_CAPTURE);       //�رղ���
    while(DCMI->CR&0X01);               //�ȴ��������
    __HAL_DMA_DISABLE(&hdma_dcmi);//�ر�DMA
} 

//DCMI�жϷ�����
void DCMI_IRQHandler(void)
{
    HAL_DCMI_IRQHandler(&hdcmi);
}

//����һ֡ͼ������
//hdcmi:DCMI���
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
	//jpeg_data_process();//jpeg���ݴ���
	//LED1_Toggle;
	ov_frame++; 
    //����ʹ��֡�ж�,��ΪHAL_DCMI_IRQHandler()������ر�֡�ж�
  __HAL_DCMI_ENABLE_IT(hdcmi,DCMI_IT_FRAME);
}

void (*dcmi_rx_callback)(void);//DCMI DMA���ջص�����
//DMA1������1�жϷ�����

////////////////////////////////////////////////////////////////////////////////
//������������,��usmart����,���ڵ��Դ���

//DCMI������ʾ����
//sx,sy;LCD����ʼ����
//width,height:LCD��ʾ��Χ.
void DCMI_Set_Window(u16 sx,u16 sy,u16 width,u16 height)
{
	DCMI_Stop(); 
	LCD_Clear(WHITE);
	LCD_Set_Window(sx,sy,width,height);
//	OV5640_OutSize_Set(0,0,width,height);
    LCD_SetCursor(0,0);  
	LCD_WriteRAM_Prepare();		        //��ʼд��GRAM  
    __HAL_DMA_ENABLE(&hdma_dcmi); //����DMA1,Stream1  
    DCMI->CR|=DCMI_CR_CAPTURE;          //DCMI����ʹ��	
}
   
//ͨ��usmart����,����������.
//pclk/hsync/vsync:�����źŵ����޵�ƽ����
void DCMI_CR_Set(u8 pclk,u8 hsync,u8 vsync)
{
    HAL_DCMI_DeInit(&hdcmi);//���ԭ��������
    hdcmi.Instance=DCMI;
    hdcmi.Init.SynchroMode=DCMI_SYNCHRO_HARDWARE;//Ӳ��ͬ��HSYNC,VSYNC
    hdcmi.Init.PCKPolarity=pclk<<5;              //PCLK ��������Ч
    hdcmi.Init.VSPolarity=vsync<<7;              //VSYNC �͵�ƽ��Ч
    hdcmi.Init.HSPolarity=hsync<<6;              //HSYNC �͵�ƽ��Ч
    hdcmi.Init.CaptureRate=DCMI_CR_ALL_FRAME;    //ȫ֡����
    hdcmi.Init.ExtendedDataMode=DCMI_EXTEND_DATA_8B;//8λ���ݸ�ʽ 
    HAL_DCMI_Init(&hdcmi);                       //��ʼ��DCMI
    hdcmi.Instance->CR|=DCMI_MODE_CONTINUOUS;    //����ģʽ
}


