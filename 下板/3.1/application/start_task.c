#include "start_task.h"

//�������ȼ�
#define START_TASK_PRIO		1
//�����ջ��С	
#define START_STK_SIZE 		128  
//������
TaskHandle_t StartTask_Handler;
//������
void start_task(void *pvParameters);


//�������ȼ�
#define chassis_TASK_PRIO		2
//�����ջ��С	
#define chassis_STK_SIZE 		128  
//������
TaskHandle_t chassisTask_Handler;
//������
void chassis_task(void *pvParameters);  //���̿�������



//�������ȼ�
#define CAN1_TASK_PRIO		2
//�����ջ��С	
#define CAN1_STK_SIZE 		128  
//������
TaskHandle_t CAN1Task_Handler;
//������
void can1_send(void *pvParameters);      //can1����


//�������ȼ�
#define BUTTON_TASK_PRIO		2
//�����ջ��С	
#define BUTTON_STK_SIZE 		128  
//������
TaskHandle_t BUTTONTask_Handler;
//������
void button_get_task(void *pvParameters);      //button����

//�������ȼ�
#define ENCODER0_TASK_PRIO		2    
//�����ջ��С	
#define ENCODER0_STK_SIZE 		128  
//������
TaskHandle_t ENCODERTask_Handler;
//������
void encoder0_task(void *pvParameters);        //����������

//�������ȼ�
#define MODEL_TASK_PRIO		2    
//�����ջ��С	
#define MODEL_STK_SIZE 		128  
//������
TaskHandle_t MODELTask_Handler;
//������
void model_task(void *pvParameters);        //��̨����

//�������ȼ�
#define INS_TASK_PRIO		2
//�����ջ��С	
#define INS_STK_SIZE 		128  
//������
TaskHandle_t INSTask_Handler;
//������
void ins_task(void *pvParameters);

//�������ȼ�
#define  FIRE_TASK_PRIO		2
//�����ջ��С	
#define FIRE_STK_SIZE 		128  
//������
TaskHandle_t FIRETask_Handler;
//������
void fire_task(void *pvParameters);

//�������ȼ�
#define  PC_TASK_PRIO		2
//�����ջ��С	
#define PC_STK_SIZE 		128  
//������
TaskHandle_t PCTask_Handler;
//������
void pc_task(void *pvParameters);


void start(void){
	
		//������ʼ����
    xTaskCreate((TaskFunction_t )start_task,            //������
                (const char*    )"start_task",          //��������
                (uint16_t       )START_STK_SIZE,        //�����ջ��С
                (void*          )NULL,                  //���ݸ��������Ĳ���
                (UBaseType_t    )START_TASK_PRIO,       //�������ȼ�
                (TaskHandle_t*  )&StartTask_Handler);   //������              
    vTaskStartScheduler();          //�����������
}



//��ʼ����������
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //�����ٽ���
	

    //����can1��������
    xTaskCreate((TaskFunction_t )can1_send,     
                (const char*    )"can1_send",   
                (uint16_t       )CAN1_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )CAN1_TASK_PRIO,
                (TaskHandle_t*  )&CAN1Task_Handler);  
								
    //������������
    xTaskCreate((TaskFunction_t )chassis_task,     
                (const char*    )"chassis_task",   
                (uint16_t       )chassis_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )chassis_TASK_PRIO,
                (TaskHandle_t*  )&chassisTask_Handler);    
								
    //����button����
    xTaskCreate((TaskFunction_t )button_get_task,     
                (const char*    )"button_get_task",   
                (uint16_t       )BUTTON_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )BUTTON_TASK_PRIO,
                (TaskHandle_t*  )&BUTTONTask_Handler);
								
		//��������������
    xTaskCreate((TaskFunction_t )encoder0_task,     
                (const char*    )"encoder0_task",   
                (uint16_t       )ENCODER0_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )ENCODER0_TASK_PRIO,
                (TaskHandle_t*  )&ENCODERTask_Handler); 
								
		//������̨����
    xTaskCreate((TaskFunction_t )model_task,     
                (const char*    )"model_task",   
                (uint16_t       )MODEL_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )MODEL_TASK_PRIO,
                (TaskHandle_t*  )&MODELTask_Handler); 	
								
		//��������������						
	  xTaskCreate((TaskFunction_t )ins_task,     	
                (const char*    )"ins_task",   	
                (uint16_t       )INS_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )INS_TASK_PRIO,	
                (TaskHandle_t*  )&INSTask_Handler);  
						
		//������������			
	  xTaskCreate((TaskFunction_t )fire_task,     	
                (const char*    )"fire_task",   	
                (uint16_t       )FIRE_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )fire_task,	
                (TaskHandle_t*  )&FIRETask_Handler);  
					
		//����pc����			
	  xTaskCreate((TaskFunction_t )pc_task,     	
                (const char*    )"pc_task",   	
                (uint16_t       )PC_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )pc_task,	
                (TaskHandle_t*  )&PCTask_Handler);  					
								
    vTaskDelete(StartTask_Handler); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ���
}




