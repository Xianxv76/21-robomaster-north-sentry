#include "communicate.h"
#include "judgement_info.h"
#include "data_fifo.h"
#include "protocol.h"
#include "JudgeSystem.h"

fifo_s_t* JudgeSystemUart3_RXFIFO;		//����ϵͳ���ݽ��ն���
u8 RX6Buff[RX6_BUF_SIZE];		//DMA���ջ�����
/**
  * @brief  ����6��ʼ��
  * @param  None
  * @retval None
  * @note   ʹ��DMA���գ��������ڿ����ж�, 
  */
int JudgeSystem_init = 0;  //��ɾ�����ڲ鿴���д���ϵͳ�Ƿ��ʼ��

	
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

	USART_InitStructure.USART_BaudRate = 115200;//������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//
	USART_InitStructure.USART_Parity = USART_Parity_No;//
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//
  USART_Init(USART6, &USART_InitStructure); //
	
  USART_Cmd(USART6, ENABLE);  //
	
	USART_ClearFlag(USART6, USART_FLAG_TC);
	//USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);//�����ж�
	USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);
	USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);

	//Usart6 NVIC 
  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//
	NVIC_Init(&NVIC_InitStructure);	//
	
	//DMA_DeInit(DMA2_Stream1); 
  DMA_InitStructure.DMA_Channel = DMA_Channel_5;  //ͨ��ѡ��
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&(USART6->DR);//DMA�����ַ****************************
  DMA_InitStructure.DMA_Memory0BaseAddr = (u32)RX6Buff;//DMA �洢��0��ַ �ڴ��ַ
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//���赽�洢��ģʽ*******************************
  DMA_InitStructure.DMA_BufferSize = RX6_BUF_SIZE;//���ݴ����� 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//���������ģʽ
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�洢������ģʽ
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݳ���:8λ
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�洢�����ݳ���:8λ
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;// ʹ����ͨģʽ ********************************
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//�е����ȼ�**********
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;//***************
  DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���
	
  DMA_Init(DMA2_Stream1, &DMA_InitStructure);//��ʼ��DMA2 Stream1	
	DMA_Cmd(DMA2_Stream1,ENABLE);
	
	JudgeSystemUart3_RXFIFO = fifo_s_create(140);		//����ϵͳ���ݽ���Э��֡����
	
	/*********************************/
	JudgeSystem_init = 1 ;
	
}

int num = 0;
int bbb = 0;


void USART6_IRQHandler(void)
{
	int i =0;
	unpack_data_t p_obj;	//Э��֡�������ݽ����ṹ��	
	
	
	if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)
	{
		
		
		num = USART6->SR;
		num = USART6->DR;	//���жϱ�־λ
		
		DMA_Cmd(DMA2_Stream1,DISABLE);	//��DMA
		
	  num = RX6_BUF_SIZE - DMA_GetCurrDataCounter(DMA2_Stream1);	//��ȡ���ݳ���	
		//DMA_GetCurrDataCounter  �˺���������ʣ��dma��С
		
		for( i = 0; i < num; i++)	//�����������
		{
    bbb++;
		fifo_s_put(JudgeSystemUart3_RXFIFO, RX6Buff[i]);		//���
	  }
		
		DMA2_Stream1->NDTR = RX6_BUF_SIZE;//�������ý������ݸ���
		
		DMA_Cmd(DMA2_Stream1,ENABLE);		//��DMA�����¿�ʼ��������
		
		p_obj.data_fifo = JudgeSystemUart3_RXFIFO;	//׼������Э��֡
		
		
		unpack_fifo_data(&p_obj, DN_REG_ID);	//����Э��֡���� �ж��������ͣ������룩
	
	}
}

	
