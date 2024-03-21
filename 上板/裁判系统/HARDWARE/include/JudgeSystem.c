#include "communicate.h"
#include "judgement_info.h"
#include "data_fifo.h"
#include "protocol.h"
#include "JudgeSystem.h"

fifo_s_t* JudgeSystemUart3_RXFIFO;		//裁判系统数据接收队列
u8 RX6Buff[RX6_BUF_SIZE];		//DMA接收缓冲区
/**
  * @brief  串口6初始化
  * @param  None
  * @retval None
  * @note   使用DMA接收，开启串口空闲中断, 
  */
int JudgeSystem_init = 0;  //可删，用于查看裁判串口系统是否初始化

	
void usart6_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE); //
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE); 

	GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART6); //
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART6); //

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14; //
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //
	GPIO_Init(GPIOG,&GPIO_InitStructure); //

	USART_InitStructure.USART_BaudRate = 115200;//波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//
	USART_InitStructure.USART_Parity = USART_Parity_No;//
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//
  USART_Init(USART6, &USART_InitStructure); //
	
  USART_Cmd(USART6, ENABLE);  //
	
	USART_ClearFlag(USART6, USART_FLAG_TC);
	//USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);//空闲中断
	USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);
	USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);

	//Usart6 NVIC 
  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//
	NVIC_Init(&NVIC_InitStructure);	//
	
	//DMA_DeInit(DMA2_Stream1); 
  DMA_InitStructure.DMA_Channel = DMA_Channel_5;  //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&(USART6->DR);//DMA外设地址****************************
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)RX6Buff;//DMA 存储器0地址 内存地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//外设到存储器模式*******************************
  DMA_InitStructure.DMA_BufferSize = RX6_BUF_SIZE;//数据传输量 
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
	
  DMA_Init(DMA2_Stream1, &DMA_InitStructure);//初始化DMA2 Stream1	
	DMA_Cmd(DMA2_Stream1,ENABLE);
	
	JudgeSystemUart3_RXFIFO = fifo_s_create(140);		//裁判系统数据接收协议帧队列
	
	/*********************************/
	JudgeSystem_init = 1 ;
	
}

int num = 0;
int bbb = 0;


void USART6_IRQHandler(void)
{
	int i =0;
	unpack_data_t p_obj;	//协议帧队列数据解析结构体	
	
	
	if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)
	{
		
		
		num = USART6->SR;
		num = USART6->DR;	//清中断标志位
		
		DMA_Cmd(DMA2_Stream1,DISABLE);	//关DMA
		
	  num = RX6_BUF_SIZE - DMA_GetCurrDataCounter(DMA2_Stream1);	//获取数据长度	
		//DMA_GetCurrDataCounter  此函数读的是剩余dma大小
		
		for( i = 0; i < num; i++)	//所有数据入队
		{
    bbb++;
		fifo_s_put(JudgeSystemUart3_RXFIFO, RX6Buff[i]);		//入队
	  }
		
		DMA2_Stream1->NDTR = RX6_BUF_SIZE;//重新设置接收数据个数
		
		DMA_Cmd(DMA2_Stream1,ENABLE);		//开DMA，重新开始接收数据
		
		p_obj.data_fifo = JudgeSystemUart3_RXFIFO;	//准备解析协议帧
		
		
		unpack_fifo_data(&p_obj, DN_REG_ID);	//解析协议帧队列 判断数据类型（命令码）
	
	}
}

	
