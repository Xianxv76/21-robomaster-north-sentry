#include "communicate.h"
#include "judgement_info.h"
#include "data_fifo.h"
#include "protocol.h"
#include "JudgeSystem.h"
#include "string.h"


/* data send (forward) */
/* data receive */
receive_judge_t judge_rece_mesg;		//裁判系统接收到的数据

/**
  * @brief    get judgement system message
  */
//extern TaskHandle_t pc_unpack_task_t;
int copy_usatr =0;

void judgement_data_handler(uint8_t *p_frame)
{  
	frame_header_t *p_header = (frame_header_t*)p_frame;	//帧头
	uint16_t data_length = p_header->data_length;		//p_header是上面定义的局部变量
  uint16_t cmd_id      = *(uint16_t *)(p_frame + HEADER_LEN); //指针操作 一次性读取两个字节的命令码数据
  uint8_t *data_addr   = p_frame + HEADER_LEN + CMD_LEN;	//数据部分的指针
  uint8_t  invalid_cmd = 0;
 
  memcpy(p_header, p_frame, HEADER_LEN);
  
  switch (cmd_id)
  {
		 case GAME_STATUS_DATA_ID:
             memcpy(&judge_rece_mesg.game_rstatus_data, data_addr, data_length);
		  copy_usatr =1;
             break;

    case GAME_RESULT_ID:
             memcpy(&judge_rece_mesg.game_result_data, data_addr, data_length);
		 copy_usatr =2;
             break;

    case ROBO_EXIST_DATA_ID:
             memcpy(&judge_rece_mesg.robot_exis_data, data_addr, data_length);
		 copy_usatr =3;
             break;
		
	case COURT_EVENT_DATA_ID:
              memcpy(&judge_rece_mesg.court_event_data, data_addr, data_length);
	 copy_usatr =4;
	            GPIO_ResetBits(GPIOG,GPIO_Pin_3);//GPIOF9,F10设置高，灯灭***************TEST
		          break;

    case SUPPLY_STA_ACTION_DATA:
               memcpy(&judge_rece_mesg.supply_act_data, data_addr, data_length);
		 copy_usatr =5;
               break;

    case SUPPLY_STA_REQUEST_DATA:
                memcpy(&judge_rece_mesg.supply_request_data, data_addr, data_length);
		 copy_usatr =6;
                break;

    case ROBO_STATE_DATA:
                memcpy(&judge_rece_mesg.game_information, data_addr, data_length);
		 copy_usatr =7;
                break;
    
    case REAL_POWER_ID:
               memcpy(&judge_rece_mesg.power_heat_data, data_addr, data_length);
		 copy_usatr =8;
               break;

	case ROBO_POS_DATA_ID:
		            memcpy(&judge_rece_mesg.robot_pos_data, data_addr, data_length);
	 copy_usatr =9;
		             break;

	case ROBO_BUFF_DATA:
	            	memcpy(&judge_rece_mesg.get_buff_data, data_addr, data_length);
	 copy_usatr =10;
		           break;

	case SPACE_ROBO_ENERGY:
		           memcpy(&judge_rece_mesg.space_father_data, data_addr, data_length);
	 copy_usatr =11;
		           break;

	case HURT_DATA:
		           memcpy(&judge_rece_mesg.blood_changed_data, data_addr, data_length);
	 copy_usatr =12;
		            break;

	case REAL_SHOOT_DATA:
		
	
		          memcpy(&judge_rece_mesg.real_shoot_data, data_addr, data_length);
	 copy_usatr =13;
		          break;
 
//	case ROBO_CONNECTION_DATA:
//		memcpy(&judge_rece_mesg.robot_pos_data, data_addr, data_length);
//		break;
    
   default:
      invalid_cmd = 1;
    break;
//    case GAME_INFO_ID:
//											memcpy(&judge_rece_mesg.game_information, data_addr, data_length);//receive_judge_t judge_rece_mesg;		//裁判系统接收到的数据
//											break;

//    case REAL_BLOOD_DATA_ID:
//											      memcpy(&judge_rece_mesg.blood_changed_data, data_addr, data_length);
//											      break;

//    case REAL_SHOOT_DATA_ID:
//     								       memcpy(&judge_rece_mesg.real_shoot_data, data_addr, data_length);
//   								         break;
//		
//		case REAL_POWER_DATA_ID:
//      								     memcpy(&judge_rece_mesg.power_heat_data, data_addr, data_length);			
//											     GPIO_ResetBits(GPIOG,GPIO_Pin_3);//GPIOF9,F10设置高，灯灭***************TEST
//										        break;

//    case FIELD_RFID_DATA_ID:
//      								     memcpy(&judge_rece_mesg.rfid_data, data_addr, data_length);
//    								       break;

//    case GAME_RESULT_ID:
//      								    memcpy(&judge_rece_mesg.game_result_data, data_addr, data_length);
//   								        break;

//    case GAIN_BUFF_ID:
//     								     memcpy(&judge_rece_mesg.get_buff_data, data_addr, data_length);
//   								       break;
//    
//    case ROBOT_POS_DATA_ID:
//     								     memcpy(&judge_rece_mesg.robot_pos_data, data_addr, data_length);
//   								       break;
//    
//    default:
//      								invalid_cmd = 1;
//    								break;
  }
  
  /* valid forward data */
  if (!invalid_cmd)	//如果是有效的数据包，命令码能对上
  {
//    data_packet_pack(cmd_id, data_addr, data_length, UP_REG_ID);
//    osSignalSet(pc_unpack_task_t, PC_UART_TX_SIGNAL);
  }
}

