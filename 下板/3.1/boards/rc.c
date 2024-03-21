#include "rc.h"
#include "sys.h"

#define RX_BUF_SIZE 18		//DMA接收缓冲区大小
 u8 RXBuff[RX_BUF_SIZE];		//DMA接收缓冲区
 
void rc_init()
{

	  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;		//dma用到中断
	DMA_InitTypeDef  DMA_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOC时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//使能USART3时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);  //DMA2时钟使能 ---********************
 
	//串口3对应引脚复用映射
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3); //GPIOC11复用为USART3
	
	//USART3端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; //GPIOC11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure); //初始化PC11

  //USART1 初始化设置
	USART_DeInit(USART3);
	USART_InitStructure.USART_BaudRate = 100000;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_Even;//偶校验
	USART_InitStructure.USART_Mode = USART_Mode_Rx;	//收模式
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
  USART_Init(USART3, &USART_InitStructure); //初始化串口2
	
  USART_Cmd(USART3, ENABLE);  //使能串口2 	
  USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);  //使能串口2的DMA接收     
	
	
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream1_IRQn;//DMA1 数据流5 中断通道*******
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
	
	
  /* 配置 DMA Stream */
	DMA_DeInit(DMA1_Stream1); 
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&(USART3->DR);//DMA外设地址****************************
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)RXBuff;//DMA 存储器0地址 内存地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//外设到存储器模式*******************************
  DMA_InitStructure.DMA_BufferSize = RX_BUF_SIZE;//数据传输量 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;// 使用普通模式 ********************************
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//中等优先级**********
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;//***************
  DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
	
  DMA_Init(DMA1_Stream1, &DMA_InitStructure);//初始化DMA1 Stream5	
	DMA_ITConfig(DMA1_Stream1,DMA_IT_TC,ENABLE);
	DMA_Cmd(DMA1_Stream1,ENABLE);
}

 RC rc;
 int ddd = 0;
void DMA1_Stream1_IRQHandler(void)
{
	ddd++;
	
	if(DMA_GetITStatus(DMA1_Stream1, DMA_IT_TCIF1))
	{
		DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1);
		DMA_ClearITPendingBit(DMA1_Stream1, DMA_IT_TCIF1);
		
		rc.cnts++;
		
		rc.R_x = -1024 + ((RXBuff[0]| (RXBuff[1] << 8)) & 0x07ff); //!< Channel 0
		rc.R_y = -1024 + (((RXBuff[1] >> 3) | (RXBuff[2] << 5)) & 0x07ff); //!< Channel 1
		rc.L_x = -1024 + (((RXBuff[2] >> 6) | (RXBuff[3] << 2) | (RXBuff[4] << 10))& 0x07ff);//!< Channel 2
		rc.L_y = -1024 + (((RXBuff[4] >> 1) | (RXBuff[5] << 7)) & 0x07ff); //!< Channel 3
		rc.sl = ((RXBuff[5] >> 4)& 0x000C) >> 2; //!< Switch left
		rc.sr = ((RXBuff[5] >> 4)& 0x0003); //!< Switch left
		

		//遥控器值抛出异常
		rc.R_x = rc.R_x > 660 ? 0 : rc.R_x;
		rc.R_x = rc.R_x < -660 ? 0 : rc.R_x;
		
		rc.R_y = rc.R_y > 660 ? 0 : rc.R_y;
		rc.R_y = rc.R_y < -660 ? 0 : rc.R_y;
		
		rc.L_x = rc.L_x > 660 ? 0 : rc.L_x;
		rc.L_x = rc.L_x < -660 ? 0 : rc.L_x;
		
		rc.L_y = rc.L_y > 660 ? 0 : rc.L_y;
		rc.L_y = rc.L_y < -660 ? 0 : rc.L_y;

	}
}

RC*get_rc_data(void)
{
	return &rc;
}
