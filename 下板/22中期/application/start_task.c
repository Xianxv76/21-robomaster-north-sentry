#include "start_task.h"

//任务优先级
#define START_TASK_PRIO		1
//任务堆栈大小	
#define START_STK_SIZE 		128  
//任务句柄
TaskHandle_t StartTask_Handler;
//任务函数
void start_task(void *pvParameters);


//任务优先级
#define chassis_TASK_PRIO		2
//任务堆栈大小	
#define chassis_STK_SIZE 		128  
//任务句柄
TaskHandle_t chassisTask_Handler;
//任务函数
void chassis_task(void *pvParameters);  //底盘控制任务



//任务优先级
#define CAN1_TASK_PRIO		2
//任务堆栈大小	
#define CAN1_STK_SIZE 		128  
//任务句柄
TaskHandle_t CAN1Task_Handler;
//任务函数
void can1_send(void *pvParameters);      //can1发送


//任务优先级
#define BUTTON_TASK_PRIO		2
//任务堆栈大小	
#define BUTTON_STK_SIZE 		128  
//任务句柄
TaskHandle_t BUTTONTask_Handler;
//任务函数
void button_get_task(void *pvParameters);      //button任务

//任务优先级
#define ENCODER0_TASK_PRIO		2    
//任务堆栈大小	
#define ENCODER0_STK_SIZE 		128  
//任务句柄
TaskHandle_t ENCODERTask_Handler;
//任务函数
void encoder0_task(void *pvParameters);        //编码器任务

//任务优先级
#define MODEL_TASK_PRIO		2    
//任务堆栈大小	
#define MODEL_STK_SIZE 		128  
//任务句柄
TaskHandle_t MODELTask_Handler;
//任务函数
void model_task(void *pvParameters);        //云台任务

//任务优先级
#define INS_TASK_PRIO		2
//任务堆栈大小	
#define INS_STK_SIZE 		128  
//任务句柄
TaskHandle_t INSTask_Handler;
//任务函数
void ins_task(void *pvParameters);

//任务优先级
#define  FIRE_TASK_PRIO		2
//任务堆栈大小	
#define FIRE_STK_SIZE 		128  
//任务句柄
TaskHandle_t FIRETask_Handler;
//任务函数
void fire_task(void *pvParameters);

//任务优先级
#define  PC_TASK_PRIO		2
//任务堆栈大小	
#define PC_STK_SIZE 		128  
//任务句柄
TaskHandle_t PCTask_Handler;
//任务函数
void pc_task(void *pvParameters);


void start(void){
	
		//创建开始任务
    xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄              
    vTaskStartScheduler();          //开启任务调度
}



//开始任务任务函数
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //进入临界区
	

    //创建can1发送任务
    xTaskCreate((TaskFunction_t )can1_send,     
                (const char*    )"can1_send",   
                (uint16_t       )CAN1_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )CAN1_TASK_PRIO,
                (TaskHandle_t*  )&CAN1Task_Handler);  
								
    //创建底盘任务
    xTaskCreate((TaskFunction_t )chassis_task,     
                (const char*    )"chassis_task",   
                (uint16_t       )chassis_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )chassis_TASK_PRIO,
                (TaskHandle_t*  )&chassisTask_Handler);    
								
    //创建button任务
    xTaskCreate((TaskFunction_t )button_get_task,     
                (const char*    )"button_get_task",   
                (uint16_t       )BUTTON_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )BUTTON_TASK_PRIO,
                (TaskHandle_t*  )&BUTTONTask_Handler);
								
		//创建编码器任务
    xTaskCreate((TaskFunction_t )encoder0_task,     
                (const char*    )"encoder0_task",   
                (uint16_t       )ENCODER0_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )ENCODER0_TASK_PRIO,
                (TaskHandle_t*  )&ENCODERTask_Handler); 
								
		//创建云台任务
    xTaskCreate((TaskFunction_t )model_task,     
                (const char*    )"model_task",   
                (uint16_t       )MODEL_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )MODEL_TASK_PRIO,
                (TaskHandle_t*  )&MODELTask_Handler); 	
								
		//创建陀螺仪任务						
	  xTaskCreate((TaskFunction_t )ins_task,     	
                (const char*    )"ins_task",   	
                (uint16_t       )INS_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )INS_TASK_PRIO,	
                (TaskHandle_t*  )&INSTask_Handler);  
						
		//创建发射任务			
	  xTaskCreate((TaskFunction_t )fire_task,     	
                (const char*    )"fire_task",   	
                (uint16_t       )FIRE_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )fire_task,	
                (TaskHandle_t*  )&FIRETask_Handler);  
					
		//创建pc任务			
	  xTaskCreate((TaskFunction_t )pc_task,     	
                (const char*    )"pc_task",   	
                (uint16_t       )PC_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )pc_task,	
                (TaskHandle_t*  )&PCTask_Handler);  					
								
    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}




