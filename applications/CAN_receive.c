/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"
#include "main.h"
#include "chassis_task.h"


extern chassis_move_t chassis_move;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

//rpm 速度测量值     give_current转矩电流值   
//position_value位置值     fault_value故障值     current_mode当前模式    
#define get_motor_measure_M15(ptr, data)                                \
    {                                                                   \
        (ptr)->rpm = (int16_t)((data)[0] << 8 | (data)[1]) / 100;      \
        (ptr)->given_current = (int16_t)((data)[2] << 8 | (data)[3]);   \
        (ptr)->position_value = (uint16_t)((data)[4] << 8 | (data)[5]); \
        (ptr)->fault_value = (data)[6]; 		                        \
		(ptr)->current_mode = (data)[7];                                \
    }
	
#define get_motor_mode_M15(ptr, data)            \
    {                                            \
        (ptr)->mode = (data)[0]; 	             \
    }                    

#define get_motor_feedback_mode_M15(ptr, data)   \
    {                                            \
        (ptr)->feedback_mode = (data)[0]; 	     \
    } 	
	
	
#define get_motor_measure_DJI(ptr, data)                           \
  {                                                                \
    (ptr)->last_ecd = (ptr)->ecd;                                  \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
    (ptr)->speed_rpm = (int16_t)((data)[2] << 8 | (data)[3]);      \
    (ptr)->given_current = (int16_t)((data)[4] << 8 | (data)[5]);  \
    (ptr)->temperate = (data)[6];                                  \
  }

motor_measure_t motor_chassis[10];

static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];

/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
uint8_t rx_data[8];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
  
	if(hcan==&hcan2)
		{
			  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		switch (rx_header.StdId)
			{
			case ROTATION_ID:                                  
				{
					get_motor_measure_DJI(&chassis_move.motor_rotation, rx_data);			
					break;
				}
			default:
				{
					break;
				}
			}
		}
	else if(hcan==&hcan1)
		{
			  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
			switch (rx_header.StdId)
				{
				case MOTOR_1_ID:
					{
						get_motor_measure_M15(&chassis_move.motor_1,rx_data);
						break;
					}
				case MOTOR_2_ID:
					{
						get_motor_measure_M15(&chassis_move.motor_2,rx_data);
						break;
					}
				case 0x201:					{
						get_motor_mode_M15(&chassis_move.motor_1,rx_data);
						break;
					}
				case 0x202:
					{
						get_motor_mode_M15(&chassis_move.motor_2,rx_data);
						break;
					}
				case 0x265:
					{
						get_motor_feedback_mode_M15(&chassis_move.motor_1,rx_data);
						break;
					}
				case 0x266:
					{
						get_motor_feedback_mode_M15(&chassis_move.motor_2,rx_data);
						break;
					}	
				default:
					{
						break;
					}
				}
		}
}

/**
  * @brief          发送电机控制电流(0x01,0x02)
  * @param[in]      motor_M15_1: (0x01) 1号电机控制电流, 范围 [-32767,32767]
  * @param[in]      motor_M15_2: (0x02) 2号电机控制电流, 范围 [-32767,32767]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor_M15_1, int16_t motor_M15_2)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = (motor_M15_1 >> 8);
    chassis_can_send_data[1] = motor_M15_1;
    chassis_can_send_data[2] = (motor_M15_2 >> 8);
    chassis_can_send_data[3] = motor_M15_2;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief          设置M15电机模式为电流环
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_set_mode(int16_t motor_1_mode, int16_t motor_2_mode)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x105;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor_1_mode;
    chassis_can_send_data[1] = motor_2_mode;
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief          设置M15电机反馈模式
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_set_feedback(int16_t motor_1_feedback, int16_t motor_2_feedback)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x106;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor_1_feedback;
    chassis_can_send_data[1] = motor_2_feedback;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief          发送电机控制电流(0x201)
  * @param[in]      motor_3508: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t motor_3508)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = 0x00;
    gimbal_can_send_data[1] = 0x00;
    gimbal_can_send_data[2] = 0x00;
    gimbal_can_send_data[3] = 0x00;
    gimbal_can_send_data[4] = motor_3508 >> 8;
    gimbal_can_send_data[5] = motor_3508;
    gimbal_can_send_data[6] = 0x00;
    gimbal_can_send_data[7] = 0x00;

    HAL_CAN_AddTxMessage(&hcan2, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
  * @brief          返回底盘M15 1号电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_1_measure_point(void)
{
    return &chassis_move.motor_1;
}

/**
  * @brief          返回底盘M15 2号电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_2_measure_point(void)
{
    return &chassis_move.motor_2;
}

/**
  * @brief          返回云台3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
const motor_measure_t_DJI *get_gimbal_motor_measure_point(void)
{
    return &chassis_move.motor_rotation;
}
