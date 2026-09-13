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

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2

/* CAN send and receive ID */
typedef enum
{
    CAN_GIMBAL_ALL_ID = 0x200,
    ROTATION_ID = 0x203,

	CAN_CHASSIS_ALL_ID = 0x32,
    MOTOR_1_ID = 0x97,
    MOTOR_2_ID = 0x98,
} can_msg_id_e;

//rm motor data
typedef struct
{	
    int16_t rpm;            //速度测量值
    int16_t given_current;  //转矩电流值
	int16_t position_value; //位置值
	int16_t fault_value;	//故障值
	int16_t current_mode;	//当前模式
	int16_t mode;           //设定的模式
	int16_t feedback_mode;  //电机反馈模式
} motor_measure_t;

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t_DJI;


/**
  * @brief          发送电机控制电流(0x01,0x02)
  * @param[in]      motor_M15_1: (0x01) 1号电机控制电流, 范围 [-32767,32767]
  * @param[in]      motor_M15_2: (0x02) 2号电机控制电流, 范围 [-32767,32767]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor_M15_1, int16_t motor_M15_2);

/**
  * @brief          设置M15电机模式为电流环
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_set_mode(int16_t motor_1_mode, int16_t motor_2_mode);

/**
  * @brief          设置M15电机反馈模式
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_set_feedback(int16_t motor_1_feedback, int16_t motor_2_feedback);

/**
  * @brief          发送电机控制电流(0x201)
  * @param[in]      motor_3508: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t motor_3508);

/**
  * @brief          返回云台3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t_DJI *get_gimbal_motor_measure_point(void);

/**
  * @brief          返回底盘M15 1号电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_chassis_motor_1_measure_point(void);

/**
  * @brief          返回底盘M15 2号电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_chassis_motor_2_measure_point(void);

#endif
