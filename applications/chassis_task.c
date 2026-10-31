#include "chassis_task.h"
#include "cmsis_os.h"
#include "pid.h"
#include "CAN_receive.h"
#include "bsp_can.h"
#include <math.h>
#include "stm32f4xx_hal.h"
#include "bluetooth_receive.h"
#include "nrf24l01_recive.h"
#include "debug_printf.h"
#include "usart.h"

fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);
int16_t soft_start(uint32_t elapsed, uint32_t duration, int16_t target_speed);

void chassis_move_init(chassis_move_t *chassis_move_init);
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

void S_Curve_Ramp_Init(S_Curve_Ramp_t *ramp, float target_speed, float last_speed, float ramp_time_total);
float S_Curve_Ramp_Calc(S_Curve_Ramp_t *ramp, uint32_t dt_ms);


// 斜坡函数结构体
S_Curve_Ramp_t motor1_ramp;
S_Curve_Ramp_t motor2_ramp;
// 定时器中断周期ms
#define TIM_PERIOD_MS 1

chassis_move_t chassis_move;
chassis_behaviour_e chassis_behaviour_mode_1;   //底盘行为模式变量
chassis_behaviour_e chassis_behaviour_mode_2;   //云台行为模式变量

fp32 v0x_set = 0.0f;      // 电机0 X轴（纵向/前后）期望速度分量
fp32 v1x_set = 0.0f;      // 电机1 X轴（纵向/前后）期望速度分量
fp32 v0y_set = 0.0f;      // 电机0 Y轴（横向/左右）期望速度分量（经斜坡滤波后）
fp32 v0y[2] = {0.0f, 0.0f}; // 电机0 Y轴速度数组：[0]当前目标值, [1]上一周期目标值（用于检测变化以重置斜坡）
fp32 v1y_set = 0.0f;      // 电机1 Y轴（横向/左右）期望速度分量（经斜坡滤波后）
fp32 v1y[2] = {0.0f, 0.0f}; // 电机1 Y轴速度数组：[0]当前目标值, [1]上一周期目标值（用于检测变化以重置斜坡）
fp32 wz_set = 0.0f;       // 云台/底盘 Z轴（旋转）期望角速度
fp32 v0 = 0.0f;           // 电机0 最终合成输出速度 (v0x_set + v0y_set)
fp32 v1 = 0.0f;           // 电机1 最终合成输出速度 (v1x_set + v1y_set)
void chassis_task(void const *pvParameters)
{

	can_filter_init();
	chassis_move_init(&chassis_move);
	CAN_cmd_chassis_set_mode(MODE_2,MODE_2);
	CAN_cmd_chassis_set_feedback(0x01,0x01);		
  while (1)
  {
		nrf24l01_recive_process();	//处理接收数据
		nrf24l01_recive_link_lost_process();	//掉线处理

		//数据更新
		chassis_feedback_update(&chassis_move);   
		//控制量设置
		chassis_set_contorl(&chassis_move);		
		//pid计算
		chassis_control_loop(&chassis_move);
		{
			static uint32_t last_debug_tick = 0;
			uint32_t current_tick = HAL_GetTick();
			if ((uint32_t)(current_tick - last_debug_tick) >= 50U)
			{
				last_debug_tick = current_tick;
				debug_printf(huart1, "spd:%d,%.1f,%d,%.1f,%d,%.1f\n",
					chassis_move.motor_chassis_measure[0].rpm,
					chassis_move.motor_chassis_measure[0].speed_set,
					chassis_move.motor_chassis_measure[1].rpm,
					chassis_move.motor_chassis_measure[1].speed_set,
					chassis_move.rotation_motor_measure.speed,
					chassis_move.rotation_motor_measure.speed_set);
			}
		}
		// M15 CAN 控制帧发送的是电流，不能直接发送 rpm 设定值
		CAN_cmd_chassis(chassis_move.motor_chassis_measure[0].give_current,
						chassis_move.motor_chassis_measure[1].give_current);
//		CAN_cmd_gimbal(100);
		CAN_cmd_gimbal(chassis_move.rotation_motor_measure.give_current);
		vTaskDelay(2);
		
  }
}

const static fp32 motor_1_speed_pid[3] = {M15_MOTOR_SPEED_PID_KP_1, M15_MOTOR_SPEED_PID_KI_1, M15_MOTOR_SPEED_PID_KD_1};
const static fp32 motor_2_speed_pid[3] = {M15_MOTOR_SPEED_PID_KP_2, M15_MOTOR_SPEED_PID_KI_2, M15_MOTOR_SPEED_PID_KD_2};
const static fp32 rotation_speed_pid[3] = {ROTATION_MOTOR_SPEED_PID_KP, ROTATION_MOTOR_SPEED_PID_KI, ROTATION_MOTOR_SPEED_PID_KD};

void chassis_move_init(chassis_move_t *chassis_move_init)
{
	//底盘电机1
	chassis_move_init->motor_chassis_measure[0].motor_measure = get_chassis_motor_1_measure_point();
	PID_init(&chassis_move_init->chassis_motor_pid[0], PID_POSITION, motor_1_speed_pid, M15_MOTOR_SPEED_PID_MAX_OUT, M15_MOTOR_SPEED_PID_MAX_IOUT);
	//底盘电机2
	chassis_move_init->motor_chassis_measure[1].motor_measure = get_chassis_motor_2_measure_point();
	PID_init(&chassis_move_init->chassis_motor_pid[1], PID_POSITION, motor_2_speed_pid, M15_MOTOR_SPEED_PID_MAX_OUT, M15_MOTOR_SPEED_PID_MAX_IOUT);
	//云台电机
    chassis_move_init->rotation_motor_measure.motor_measure_DJI = get_gimbal_motor_measure_point();
	PID_init(&chassis_move_init->rotation_motor_pid, PID_POSITION, rotation_speed_pid, ROTATION_MOTOR_SPEED_PID_MAX_OUT, ROTATION_MOTOR_SPEED_PID_MAX_IOUT);
    
	
	chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
	chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
	
	chassis_move_init->wz_max_speed = NORMAL_MAX_ROTATION_SPEED_Z;
	chassis_move_init->wz_min_speed = -NORMAL_MAX_ROTATION_SPEED_Z;
}


void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
	//底盘
	chassis_move_update->motor_chassis_measure[0].rpm = chassis_move_update->motor_chassis_measure[0].motor_measure->rpm;
  chassis_move_update->motor_chassis_measure[1].rpm = chassis_move_update->motor_chassis_measure[1].motor_measure->rpm;
	
	chassis_move_update->motor_chassis_measure[0].speed = chassis_move_update->motor_chassis_measure[0].motor_measure->rpm * CHASSIS_MOTOR_RPM_TO_VECTOR_SEN;
  chassis_move_update->motor_chassis_measure[1].speed = chassis_move_update->motor_chassis_measure[1].motor_measure->rpm * CHASSIS_MOTOR_RPM_TO_VECTOR_SEN;

	chassis_move_update->motor_chassis_measure[0].current = chassis_move_update->motor_chassis_measure[0].motor_measure->given_current;
  chassis_move_update->motor_chassis_measure[1].current = chassis_move_update->motor_chassis_measure[1].motor_measure->given_current;
	
  chassis_move_update->vx = (chassis_move_update->motor_chassis_measure[0].speed + chassis_move_update->motor_chassis_measure[1].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
	
	// 云台
	// PID 统一使用 rpm，避免与 speed_set 的 rpm 单位不一致
	chassis_move_update->rotation_motor_measure.speed =
		chassis_move_update->rotation_motor_measure.motor_measure_DJI->speed_rpm;

	chassis_move_update->wz =
		chassis_move_update->rotation_motor_measure.speed / RPM_RATIO;
}


//底盘

static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
    if (chassis_move_control == NULL)
    {
        return;
    }
	 

  float rpm = RPM_FORWARD_RATIO * move_speed;
	//y轴速度
	if(cmd_move && !cmd_u_move)//前进
	{
		v0y[0] = rpm; v1y[0] = -rpm;	
	}
	else if(cmd_u_move && !cmd_move)//后退
	{
		v0y[0] = -rpm; v1y[0] = rpm;
	}
	else
	{		
		v0y[0] = 0; v1y[0] = 0;
	}

	
	//x轴速度   可以优化控制器分辨逻辑，懒得弄了
	if(lr_joystick >= 100 || lr_joystick <= -100)
	{
		v0x_set = lr_joystick /660.0f * rpm * lr_move_sensitivity;
		v1x_set = lr_joystick /660.0f * rpm * lr_move_sensitivity;
	}
	else
	{
		v0x_set = 0; v1x_set = 0;
	}
	
	if(v0y[1] != v0y[0] || v1y[1] != v1y[0])
	{
		S_Curve_Ramp_Init(&motor1_ramp, v0y[0], motor1_ramp.current_speed, 5);
		S_Curve_Ramp_Init(&motor2_ramp, v1y[0], motor2_ramp.current_speed, 5);
		v0y[1] = v0y[0]; v1y[1] = v1y[0];		
	}
	
	v0y_set = S_Curve_Ramp_Calc(&motor1_ramp, 2);
	v1y_set = S_Curve_Ramp_Calc(&motor2_ramp, 2);
	//	v0y_set = v0y[0];
	//	v1y_set = v1y[0];
	//	v0y_set = v0y[0]; v1y_set = v1y[0];
	
	v0 = v0x_set + v0y_set; v1 = v1x_set + v1y_set;

	chassis_move_control->v0_set = v0;
	chassis_move_control->v1_set = v1;

	chassis_move_control->motor_chassis_measure[0].speed_set = chassis_move_control->v0_set;
	chassis_move_control->motor_chassis_measure[1].speed_set = chassis_move_control->v1_set;			

	float rotation_rpm = RPM_RATIO * rotation_speed;		
	//云台

	  if(cmd_clockwise && !cmd_counterclockwise)//顺时针旋转
		{
			wz_set = rotation_rpm;
		}
		else if(!cmd_clockwise && cmd_counterclockwise)//逆时针旋转
		{
			wz_set = -rotation_rpm;
		}
		else
		{
			wz_set = 0;
		}
		chassis_move_control->wz_set = wz_set;
		chassis_move_control->rotation_motor_measure.speed_set = chassis_move_control->wz_set;				

}

static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
//    PID_calc(&chassis_move_control_loop->chassis_motor_pid[0], chassis_move_control_loop->motor_chassis_measure[0].speed, chassis_move_control_loop->motor_chassis_measure[0].speed_set);
//    PID_calc(&chassis_move_control_loop->chassis_motor_pid[1], chassis_move_control_loop->motor_chassis_measure[1].speed, chassis_move_control_loop->motor_chassis_measure[1].speed_set);
	
	if (chassis_move_control_loop->motor_chassis_measure[0].speed_set == 0.0f)
	{
		PID_clear(&chassis_move_control_loop->chassis_motor_pid[0]);
		chassis_move_control_loop->motor_chassis_measure[0].give_current = 0;
	}
	else
	{
		PID_calc(&chassis_move_control_loop->chassis_motor_pid[0],
			chassis_move_control_loop->motor_chassis_measure[0].rpm,
			chassis_move_control_loop->motor_chassis_measure[0].speed_set);
		chassis_move_control_loop->motor_chassis_measure[0].give_current =
			(int16_t)(chassis_move_control_loop->chassis_motor_pid[0].out);
	}

	if (chassis_move_control_loop->motor_chassis_measure[1].speed_set == 0.0f)
	{
		PID_clear(&chassis_move_control_loop->chassis_motor_pid[1]);
		chassis_move_control_loop->motor_chassis_measure[1].give_current = 0;
	}
	else
	{
		PID_calc(&chassis_move_control_loop->chassis_motor_pid[1],
			chassis_move_control_loop->motor_chassis_measure[1].rpm,
			chassis_move_control_loop->motor_chassis_measure[1].speed_set);
		chassis_move_control_loop->motor_chassis_measure[1].give_current =
			(int16_t)(chassis_move_control_loop->chassis_motor_pid[1].out);
	}
	
	// 云台零速时滤除 GM4310 速度反馈的量化噪声，避免上电后电流正负抖动。
	if (chassis_move_control_loop->rotation_motor_measure.speed_set == 0.0f &&
		fabsf(chassis_move_control_loop->rotation_motor_measure.speed) <= ROTATION_MOTOR_SPEED_ZERO_DEADBAND_RPM)
	{
		PID_clear(&chassis_move_control_loop->rotation_motor_pid);
		chassis_move_control_loop->rotation_motor_measure.give_current = 0;
	}
	else
	{
		PID_calc(&chassis_move_control_loop->rotation_motor_pid,
			chassis_move_control_loop->rotation_motor_measure.speed,
			chassis_move_control_loop->rotation_motor_measure.speed_set);
		chassis_move_control_loop->rotation_motor_measure.give_current =
			(int16_t)(chassis_move_control_loop->rotation_motor_pid.out);
	}
}

fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue)
{
	if(Value < minValue){
		return minValue;
	}
    else if(Value > maxValue){
        return maxValue;
	}
    else{
        return Value;
	}
}

/**
 * @brief  初始化S型斜坡参数
 * @param  ramp: 斜坡控制结构体指针
 * @param  target_speed: 目标速度
 * @param  ramp_time_total: 斜坡总时长 ms
 * @retval 无
 */
void S_Curve_Ramp_Init(S_Curve_Ramp_t *ramp, float target_speed,float last_speed, float ramp_time_total)
{
	ramp->target_speed = target_speed;
	ramp->last_speed = last_speed;
	ramp->current_speed = last_speed;
	ramp->ramp_time_total = ramp_time_total;
	ramp->ramp_time_elapsed = 0.0f;
	ramp->ramp_finish = (target_speed == last_speed || ramp_time_total <= 0.0f);
}

/**
 * @brief  计算S型斜坡输出速度
 * @param  ramp: 斜坡控制结构体指针
 * @param  dt_ms: 调用周期 ms
 * @retval 当前输出速度
 */
float S_Curve_Ramp_Calc(S_Curve_Ramp_t *ramp, uint32_t dt_ms)
{
	if (ramp->ramp_finish || ramp->ramp_time_total <= 0.0f) {
		ramp->ramp_finish = 1;
		ramp->current_speed = ramp->target_speed;
		return ramp->current_speed;
	}

	ramp->ramp_time_elapsed += (float)dt_ms;

	float t_norm = ramp->ramp_time_elapsed / ramp->ramp_time_total;	//时间归一化

	if (t_norm >= 1.0f) {
		t_norm = 1.0f;
		ramp->ramp_finish = 1;
	}

	//s型斜坡核心计算
	float progress = (1.0f - cosf(t_norm * PI)) * 0.5f;
	ramp->current_speed = ramp->last_speed +
						  (ramp->target_speed - ramp->last_speed) * progress;

	if (fabsf(ramp->current_speed) < 0.01f) {
		ramp->current_speed = 0.0f;
	}

	if (ramp->ramp_finish) {
		ramp->current_speed = ramp->target_speed;
	}

	return ramp->current_speed;
}
