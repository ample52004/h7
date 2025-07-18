#include "sys.h"
#include "dcmi.h" 
#include "lcd.h" 
#include "ltdc.h"

extern DCMI_HandleTypeDef  hdcmi;           //DCMI句柄
extern DMA_HandleTypeDef   hdma_dcmi;        //DMA句柄
extern u16 Parallel_Data_Buffer1[38400] ;
extern u16 Parallel_Data_Buffer2[38400] ;

int zhencount;
extern u16 display[76800];



u8 ov_frame=0;  						//帧率
extern void jpeg_data_process(void);	//JPEG数据处理函数


void DMA1_Stream1_IRQHandler(void)
{
	  long i=0;
    if(__HAL_DMA_GET_FLAG(&hdma_dcmi,DMA_FLAG_TCIF1_5)!=RESET)//DMA传输完成
    {
        __HAL_DMA_CLEAR_FLAG(&hdma_dcmi,DMA_FLAG_TCIF1_5);//清除DMA传输完成中断标志位
//        dcmi_rx_callback();	//执行摄像头接收回调函数,读取数据等操作在这里面处理
			zhencount++; //图像分段标志，共计2段
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
		//display的数据可以做图像处理
		for(i=0;i<76800;i++)
		{
			LCD_WriteRAM(display[i]);
		} 	 
    } 
}


//DCMI初始化
void DCMI_Init(void)
{
    hdcmi.Instance=DCMI;
    hdcmi.Init.SynchroMode=DCMI_SYNCHRO_HARDWARE;    //硬件同步HSYNC,VSYNC
    hdcmi.Init.PCKPolarity=DCMI_PCKPOLARITY_RISING;  //PCLK 上升沿有效
    hdcmi.Init.VSPolarity=DCMI_VSPOLARITY_LOW;       //VSYNC 低电平有效
    hdcmi.Init.HSPolarity=DCMI_HSPOLARITY_LOW;       //HSYNC 低电平有效
    hdcmi.Init.CaptureRate=DCMI_CR_ALL_FRAME;        //全帧捕获
    hdcmi.Init.ExtendedDataMode=DCMI_EXTEND_DATA_8B; //8位数据格式 
    HAL_DCMI_Init(&hdcmi);                           //初始化DCMI，此函数会开启帧中断  

    //关闭行中断、VSYNC中断、同步错误中断和溢出中断
    //__HAL_DCMI_DISABLE_IT(&hdcmi,DCMI_IT_LINE|DCMI_IT_VSYNC|DCMI_IT_ERR|DCMI_IT_OVR);
	
	//关闭所有中断，函数HAL_DCMI_Init()会默认打开很多中断，开启这些中断
	//以后我们就需要对这些中断做相应的处理，否则的话就会导致各种各样的问题，
	//但是这些中断很多都不需要，所以这里将其全部关闭掉，也就是将IER寄存器清零。
	//关闭完所有中断以后再根据自己的实际需求来使能相应的中断。
	DCMI->IER=0x0;	
	
    __HAL_DCMI_ENABLE_IT(&hdcmi,DCMI_IT_FRAME);      //使能帧中断
    __HAL_DCMI_ENABLE(&hdcmi);                       //使能DCMI
}

//DCMI底层驱动，引脚配置，时钟使能，中断配置
//此函数会被HAL_DCMI_Init()调用
//hdcmi:DCMI句柄
//void HAL_DCMI_MspInit(DCMI_HandleTypeDef* hdcmi)
//{
//    GPIO_InitTypeDef GPIO_Initure;
//    
//    __HAL_RCC_DCMI_CLK_ENABLE();                //使能DCMI时钟

//    __HAL_RCC_GPIOA_CLK_ENABLE();               //使能GPIOA时钟
//    __HAL_RCC_GPIOB_CLK_ENABLE();               //使能GPIOB时钟
//    __HAL_RCC_GPIOC_CLK_ENABLE();               //使能GPIOC时钟
//    __HAL_RCC_GPIOD_CLK_ENABLE();               //使能GPIOD时钟
//    __HAL_RCC_GPIOH_CLK_ENABLE();               //使能GPIOH时钟
//    
//    //初始化PA6
//    GPIO_Initure.Pin=GPIO_PIN_6;  
//    GPIO_Initure.Mode=GPIO_MODE_AF_PP;          //推挽复用
//    GPIO_Initure.Pull=GPIO_PULLUP;              //上拉
//    GPIO_Initure.Speed=GPIO_SPEED_FREQ_VERY_HIGH;       //高速
//    GPIO_Initure.Alternate=GPIO_AF13_DCMI;      //复用为DCMI   
//    HAL_GPIO_Init(GPIOA,&GPIO_Initure);         //初始化
//    
//    //PB7,8,9
//    GPIO_Initure.Pin=GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;  
//    HAL_GPIO_Init(GPIOB,&GPIO_Initure);         //初始化
//    
//    //PC6,7,8,9,11
//    GPIO_Initure.Pin=GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|\
//                     GPIO_PIN_9|GPIO_PIN_11;  
//    HAL_GPIO_Init(GPIOC,&GPIO_Initure);         //初始化
//    
//    //PD3
//    GPIO_Initure.Pin=GPIO_PIN_3; 
//    HAL_GPIO_Init(GPIOD,&GPIO_Initure);         //初始化
//    
//    //PH8
//    GPIO_Initure.Pin=GPIO_PIN_8; 
//    HAL_GPIO_Init(GPIOH,&GPIO_Initure);         //初始化   
//    
//    HAL_NVIC_SetPriority(DCMI_IRQn,2,2);        //抢占优先级1，子优先级2
//    HAL_NVIC_EnableIRQ(DCMI_IRQn);              //使能DCMI中断
//					
//}

//DCMI DMA配置
//mem0addr:存储器地址0  将要存储摄像头数据的内存地址(也可以是外设地址)
//mem1addr:存储器地址1  当只使用mem0addr的时候,该值必须为0
//memblen:存储器位宽,可以为:DMA_MDATAALIGN_BYTE/DMA_MDATAALIGN_HALFWORD/DMA_MDATAALIGN_WORD
//meminc:存储器增长方式,可以为:DMA_MINC_ENABLE/DMA_MINC_DISABLE
void DCMI_DMA_Init(u32 mem0addr,u32 mem1addr,u16 memsize,u32 memblen,u32 meminc)
{ 
    __HAL_RCC_DMA1_CLK_ENABLE();                                    //使能DMA1时钟
    __HAL_LINKDMA(&hdcmi,DMA_Handle,hdma_dcmi);        //将DMA与DCMI联系起来
	__HAL_DMA_DISABLE_IT(&hdma_dcmi,DMA_IT_TC);    			//先关闭DMA传输完成中断(否则在使用MCU屏的时候会出现花屏的情况)
	
    hdma_dcmi.Instance=DMA1_Stream1;                          //DMA1数据流1    
	  hdma_dcmi.Init.Request=DMA_REQUEST_DCMI;					//DCMI的DMA请求
    hdma_dcmi.Init.Direction=DMA_PERIPH_TO_MEMORY;            //外设到存储器
    hdma_dcmi.Init.PeriphInc=DMA_PINC_DISABLE;                //外设非增量模式
    hdma_dcmi.Init.MemInc=meminc;                   			//存储器增量模式
    hdma_dcmi.Init.PeriphDataAlignment=DMA_PDATAALIGN_WORD;   //外设数据长度:32位
    hdma_dcmi.Init.MemDataAlignment=memblen;     				//存储器数据长度:8/16/32位
    hdma_dcmi.Init.Mode=DMA_CIRCULAR;                         //使用循环模式 
    hdma_dcmi.Init.Priority=DMA_PRIORITY_HIGH;                //高优先级
    hdma_dcmi.Init.FIFOMode=DMA_FIFOMODE_ENABLE;              //使能FIFO
    hdma_dcmi.Init.FIFOThreshold=DMA_FIFO_THRESHOLD_HALFFULL; //使用1/2的FIFO 
    hdma_dcmi.Init.MemBurst=DMA_MBURST_SINGLE;                //存储器突发传输
    hdma_dcmi.Init.PeriphBurst=DMA_PBURST_SINGLE;             //外设突发单次传输 
    HAL_DMA_DeInit(&hdma_dcmi);                               //先清除以前的设置
    HAL_DMA_Init(&hdma_dcmi);	                                //初始化DMA
    
    //在开启DMA之前先使用__HAL_UNLOCK()解锁一次DMA,因为HAL_DMA_Statrt()HAL_DMAEx_MultiBufferStart()
    //这两个函数一开始要先使用__HAL_LOCK()锁定DMA,而函数__HAL_LOCK()会判断当前的DMA状态是否为锁定状态，如果是
    //锁定状态的话就直接返回HAL_BUSY，这样会导致函数HAL_DMA_Statrt()和HAL_DMAEx_MultiBufferStart()后续的DMA配置
    //程序直接被跳过！DMA也就不能正常工作，为了避免这种现象，所以在启动DMA之前先调用__HAL_UNLOC()先解锁一次DMA。
    __HAL_UNLOCK(&hdma_dcmi);
    if(mem1addr==0)    //开启DMA，不使用双缓冲
    {
        HAL_DMA_Start(&hdma_dcmi,(u32)&DCMI->DR,mem0addr,memsize);
    }
    else                //使用双缓冲
    {
        HAL_DMAEx_MultiBufferStart(&hdma_dcmi,(u32)&DCMI->DR,mem0addr,mem1addr,memsize);//开启双缓冲
        __HAL_DMA_ENABLE_IT(&hdma_dcmi,DMA_IT_TC);    //开启传输完成中断
        HAL_NVIC_SetPriority(DMA1_Stream1_IRQn,2,3);        //DMA中断优先级
        HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    }    
}

//DCMI,启动传输
void DCMI_Start(void)
{  
    LCD_SetCursor(0,0);  
	LCD_WriteRAM_Prepare();		        //开始写入GRAM
    __HAL_DMA_ENABLE(&hdma_dcmi); //使能DMA
    DCMI->CR|=DCMI_CR_CAPTURE;          //DCMI捕获使能
}

//DCMI,关闭传输
void DCMI_Stop(void)
{ 
    DCMI->CR&=~(DCMI_CR_CAPTURE);       //关闭捕获
    while(DCMI->CR&0X01);               //等待传输完成
    __HAL_DMA_DISABLE(&hdma_dcmi);//关闭DMA
} 

//DCMI中断服务函数
void DCMI_IRQHandler(void)
{
    HAL_DCMI_IRQHandler(&hdcmi);
}

//捕获到一帧图像处理函数
//hdcmi:DCMI句柄
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
	//jpeg_data_process();//jpeg数据处理
	//LED1_Toggle;
	ov_frame++; 
    //重新使能帧中断,因为HAL_DCMI_IRQHandler()函数会关闭帧中断
  __HAL_DCMI_ENABLE_IT(hdcmi,DCMI_IT_FRAME);
}

void (*dcmi_rx_callback)(void);//DCMI DMA接收回调函数
//DMA1数据流1中断服务函数

////////////////////////////////////////////////////////////////////////////////
//以下两个函数,供usmart调用,用于调试代码

//DCMI设置显示窗口
//sx,sy;LCD的起始坐标
//width,height:LCD显示范围.
void DCMI_Set_Window(u16 sx,u16 sy,u16 width,u16 height)
{
	DCMI_Stop(); 
	LCD_Clear(WHITE);
	LCD_Set_Window(sx,sy,width,height);
//	OV5640_OutSize_Set(0,0,width,height);
    LCD_SetCursor(0,0);  
	LCD_WriteRAM_Prepare();		        //开始写入GRAM  
    __HAL_DMA_ENABLE(&hdma_dcmi); //开启DMA1,Stream1  
    DCMI->CR|=DCMI_CR_CAPTURE;          //DCMI捕获使能	
}
   
//通过usmart调试,辅助测试用.
//pclk/hsync/vsync:三个信号的有限电平设置
void DCMI_CR_Set(u8 pclk,u8 hsync,u8 vsync)
{
    HAL_DCMI_DeInit(&hdcmi);//清除原来的设置
    hdcmi.Instance=DCMI;
    hdcmi.Init.SynchroMode=DCMI_SYNCHRO_HARDWARE;//硬件同步HSYNC,VSYNC
    hdcmi.Init.PCKPolarity=pclk<<5;              //PCLK 上升沿有效
    hdcmi.Init.VSPolarity=vsync<<7;              //VSYNC 低电平有效
    hdcmi.Init.HSPolarity=hsync<<6;              //HSYNC 低电平有效
    hdcmi.Init.CaptureRate=DCMI_CR_ALL_FRAME;    //全帧捕获
    hdcmi.Init.ExtendedDataMode=DCMI_EXTEND_DATA_8B;//8位数据格式 
    HAL_DCMI_Init(&hdcmi);                       //初始化DCMI
    hdcmi.Instance->CR|=DCMI_MODE_CONTINUOUS;    //持续模式
}


