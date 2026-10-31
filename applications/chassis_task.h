#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "stm32f4xx_hal.h"
#include "bluetooth_receive.h"

#define PI 3.1415926535798932384626f   //圆周率
#define Circumference 0.6754424205196  //轮子周长
#define MODE_0 0X00
#define MODE_1 0x01
#define MODE_2 0x02
//目标速度 * 比例 = 实际发送rpm
#define RPM_FORWARD_RATIO 8883.0665912045f
#define RPM_BACK_RATIO -8883.0665912045f
#define RPM_RATIO  44.807f // 主动轮30，从动轮70，减速箱3591/187


//摇杆死区
#define CHASSIS_RC_DEADLINE 50

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.5f

//底盘M15最大can发送电流值
#define MAX_MOTOR_M15_CAN_CURRENT 32767.0f

//M15 rpm转化成底盘速度(m/s)的比例，  
#define M15_MOTOR_RPM_TO_VECTOR 0.0112573736753
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M15_MOTOR_RPM_TO_VECTOR

//M15 rpm转电流 比例，  
#define M15_MOTOR_RPM_TO_CURRENT 156
#define CHASSIS_MOTOR_RPM_TO_CURRENT_SEN M15_MOTOR_RPM_TO_CURRENT

//M15 速度转电流 比例，  
#define M15_MOTOR_VECTOR_TO_CURRENT 13860.544904474
#define CHASSIS_MOTOR_VECTOR_TO_CURRENT_SEN M15_MOTOR_VECTOR_TO_CURRENT

//M3508转化成云台速度(m/s)的比例，  
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define GIMBAL_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//运动过程速度
#define NORMAL_MAX_CHASSIS_SPEED_X 1000.0f
#define NORMAL_MAX_ROTATION_SPEED_Z 1000.0f


//底盘电机速度环PID
#define M15_MOTOR_SPEED_PID_KP_1 1000.0F//400.0f
#define M15_MOTOR_SPEED_PID_KI_1 0.5f
#define M15_MOTOR_SPEED_PID_KD_1 0.0f

#define M15_MOTOR_SPEED_PID_KP_2 1000.0F//400.0f
#define M15_MOTOR_SPEED_PID_KI_2 0.5f
#define M15_MOTOR_SPEED_PID_KD_2 0.0f

#define M15_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_M15_CAN_CURRENT
#define M15_MOTOR_SPEED_PID_MAX_IOUT 500.0f

// GM4310 云台速度环 PID，目标值和反馈值均为电机轴 rpm。
// 初始值按 3.5 kg 低速负载估算，需结合实测响应继续整定。
#define ROTATION_MOTOR_SPEED_PID_KP 10.0f
#define ROTATION_MOTOR_SPEED_PID_KI 0.05f
#define ROTATION_MOTOR_SPEED_PID_KD 0.0f
#define ROTATION_MOTOR_SPEED_PID_MAX_OUT 10000.0f
#define ROTATION_MOTOR_SPEED_PID_MAX_IOUT 10000.0f
#define ROTATION_MOTOR_SPEED_ZERO_DEADBAND_RPM 1.0f

typedef enum
{
	CHASSIS_NO_MOVE,
	HIGH_MOVE,
	MIDDLE_MOVE,
	LOW_MOVE,
	
	HIGH_ROTATION,
	MIDDLE_ROTATION,
	LOW_ROTATION,
} chassis_mode_e;

typedef struct
{
	const motor_measure_t *motor_measure;
	const motor_measure_t_DJI *motor_measure_DJI;
	int16_t rpm;
	fp32 speed;
	fp32 speed_set;
	int16_t give_current;//发送的值
	int16_t current;//返回的电流值
	
} chassis_motor_t;

typedef struct
{
  chassis_mode_e chassis_mode_1;                      //底盘控制状态机
  chassis_mode_e chassis_mode_2;                      //云台控制状态机
  chassis_mode_e last_chassis_mode;                   //底盘上次控制状态机
  chassis_motor_t motor_chassis_measure[2];           //底盘电机数据
  chassis_motor_t rotation_motor_measure;             //云台电机数据
  pid_type_def chassis_motor_pid[2];                  //底盘移动电机pid
  pid_type_def rotation_motor_pid;                    //云台旋转电机pid

  fp32 vx;                           //底盘速度 前进方向 前为正，单位 m/s
  fp32 wz;                           //云台输出轴转速，逆时针为正，单位 rpm
  fp32 v0_set;                      //底盘设定速度 前进方向 前为正，单位 m/s
  fp32 v1_set;                      //底盘设定速度 前进方向 前为正，单位 m/s
  fp32 wz_set;                       //云台电机目标转速，逆时针为正，单位 rpm

  fp32 vx_max_speed;  //前进方向最大速度 单位m/s
  fp32 vx_min_speed;  //后退方向最大速度 单位m/s
  fp32 wz_max_speed;  //旋转最大速度 单位m/s
  fp32 wz_min_speed;  //旋转最小速度 单位m/s

  
  motor_measure_t motor_1;
  motor_measure_t motor_2;
  motor_measure_t_DJI motor_rotation;  

} chassis_move_t;

typedef struct 
{
	float target_speed;      // 目标速度（最终要到达的速度）
	float last_speed;        // 变化前速度
	float current_speed;     // 当前输出速度
	float ramp_time_total;   // 斜坡总时长（ms，控制加速快慢，越大越平缓）
	float ramp_time_elapsed; // 已流逝时间（ms）
	uint8_t ramp_finish;     // 斜坡完成标志（0：未完成，1：完成）
} S_Curve_Ramp_t;

extern chassis_move_t chassis_move;

/**
  * @brief          底盘任务，间隔 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  * @param[out]     vy_set: 横向速度指针
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
  * @retval         none
  */
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);





typedef enum
{
	CHASSIS_ZERO_SPEED,                     //底盘不动
	CHASSIS_LOW_RATIO,                      //底盘低速移动
	CHASSIS_MIDDLE_RATIO,                   //底盘中速移动
	CHASSIS_HIGH_RATIO,                     //底盘高速移动
	
	GIMBAL_ZERO_SPEED,                      //云台不动
	GIMBAL_LOW_SPEED,                       //云台低速移动
	GIMBAL_MIDDLE_SPEED,                    //云台中速移动
	GIMBAL_HIGH_SPEED,                      //云台高速移动
} chassis_behaviour_e;


/**
  * @brief          通过逻辑判断，赋值"chassis_behaviour_mode"成哪种模式
  * @param[in]      chassis_move_mode: 底盘数据
  * @retval         none
  */
extern void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode);

/**
  * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
  * @param[out]     vx_set, 通常控制纵向移动.
  * @param[out]     vy_set, 通常控制横向移动.
  * @param[out]     wz_set, 通常控制旋转运动.
  * @param[in]      chassis_move_rc_to_vector,  包括底盘所有信息.
  * @retval         none
  */

extern void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

#endif
