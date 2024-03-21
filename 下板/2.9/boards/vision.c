#include "vision.h"
#include "string.h"

#define RX_VISION_SIZE 18		//DMA接收缓冲区大小

 static u8 RXVISION[RX_VISION_SIZE];		//DMA接收缓冲区
  vision vis;
 
 static mpu*bmi;
 
 
void vision_init(void)
{
  	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;		//dma用到中断
	DMA_InitTypeDef  DMA_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);;//使能USART1时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); //DMA2时钟使能 ---********************
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_USART6); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource14, GPIO_AF_USART6); //GPIOA10复用为USART1
	
	//USART1端口配置
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;         
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;       
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;   
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;       
    GPIO_Init(GPIOG, &GPIO_InitStructure);  /* TXIO */  

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_14;                
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;   
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;       
    GPIO_Init(GPIOG, &GPIO_InitStructure);  /* RXIO */

  //USART1 初始化设置
	USART_DeInit(USART6);
	USART_InitStructure.USART_BaudRate = 115200;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无校验
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_Init(USART6, &USART_InitStructure); //初始化串口1
	
	USART_Cmd(USART6, ENABLE);  //使能串口1
	
	USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);  //使能串口1的DMA接收     
	
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;//DMA1 数据流5 中断通道*******
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
	
	
  /* 配置 DMA Stream */
	DMA_DeInit(DMA2_Stream1); 	
	while (DMA_GetCmdStatus(DMA2_Stream1) != DISABLE);//等待DMA可配置
	DMA_InitStructure.DMA_Channel = DMA_Channel_5;  //通道选择
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&(USART6->DR);//DMA外设地址****************************
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)RXVISION;//DMA 存储器0地址 内存地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//外设到存储器模式*******************************
	DMA_InitStructure.DMA_BufferSize = RX_VISION_SIZE;//数据传输量 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;//中等优先级**********
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;//***************
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输*****
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输

	DMA_Init(DMA2_Stream1, &DMA_InitStructure);//初始化DMA2 Stream5	
	DMA_ITConfig(DMA2_Stream1,DMA_IT_TC,ENABLE);
	DMA_Cmd(DMA2_Stream1,ENABLE);
}

float look_set_yaw = 0;
float look_set_pitch = 0;


void DMA2_Stream1_IRQHandler(void)
{
	 bmi = mpu_get_data();
	if(DMA_GetFlagStatus(DMA2_Stream1,DMA_FLAG_TCIF1)!=RESET)
	{
		DMA_Cmd(DMA2_Stream1, DISABLE); //关闭DMA,防止处理其间有数据		
		DMA_ClearFlag(DMA2_Stream1,DMA_FLAG_TCIF1 | DMA_FLAG_FEIF1 | DMA_FLAG_DMEIF1 | DMA_FLAG_TEIF1 | DMA_FLAG_HTIF1);//清除DMA2_Steam7传输完成标志
		DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);		

		 if( RXVISION[0] == 0xA5 && RXVISION[15] == 0xEE )
	 {
		
		 vis.mode = RXVISION[1];     //模式
		 ///////////yaw轴///////////
		 vis.set_yaw.data[0] = RXVISION[2] ;  
		 vis.set_yaw.data[1] = RXVISION[3] ;  
     vis.set_yaw.data[2] = RXVISION[4] ;  
		 vis.set_yaw.data[3] = RXVISION[5] ; 
		 if( vis.set_yaw.angle == 45 )
			 vis.set_yaw.angle = 0;
		 vis.set_yaw.angle =  -vis.set_yaw.angle;
		 
		 look_set_yaw = vis.set_yaw.angle;
		 
		 
		 ///////////pitch轴///////////
		 vis.set_pitch.data[0] =  RXVISION[6] ;  
		 vis.set_pitch.data[1] =  RXVISION[7] ;
		 vis.set_pitch.data[2] =  RXVISION[8] ;  
		 vis.set_pitch.data[3] =  RXVISION[9] ;
		 
		 if( vis.set_pitch.angle == 45 )
			 vis.set_pitch.angle = 0; 
		 vis.set_pitch.angle =  vis.set_pitch.angle;
		 look_set_pitch = vis.set_pitch.angle;
	
		 ///////   distance距离  ////////////
		 vis.distance.data[0]  =  RXVISION[10] ;  
		 vis.distance.data[1]  =  RXVISION[11] ;
		 vis.distance.data[2]  =  RXVISION[12] ;
		 vis.distance.data[3]  =  RXVISION[13] ;
		 
		 vis.command = RXVISION[14];   
		 
		 vis.Vision_Get_New_Data = 1;  //卡尔曼滤波更新数据标志位
	 }
		DMA_Cmd(DMA2_Stream1, ENABLE); 
	}
}

vision*get_vision_date(void)
{
	return &vis;
}
