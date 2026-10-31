/////////////////////////////////////////////遥控器主任务///////////////////////////////////////////////////////////////////////////////////
                                   
//2026.9.12 v1.00                   Han          //通过中断读取按键状态,在循环任务处理后发送，按键数据处理：调速按键进行计数，控制移动按键仅读取电平
//2026.9.13 v1.01                   Han          //添加oled输出目标速度

//2026.9.13 v1.10                   Han          //遥控器可正常发送数据包
//2026.9.20 v1.20                   Han          //由中断改为轮询，保留中断读取接口，按键检测稳定

//2026.9.21 v1.30                   Han          //功能完备，OLED屏幕能使用，数据说明显示不全

//2026.10.25 v1.31                  Han          //将旋转方法改为按下切换

//2026.10.31 v1.40                  Han          //启用预留按键【9】,按下时为输出目标速度乘一随时间变化正弦
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include "remoter.h"
#include "main.h"
#include <math.h>
#include <string.h>
#include "cmsis_os.h"


void Remote_Init(void);         //遥控器初始化
void Remote_Send(void);         //发送数据包
void date_pack_process(void);   //打包数据包

/////////////////////全局变量////////////////////////////
uint8_t tx_buffer[NRF24L01_TX_PACKET_WIDTH]={0};


#define MOVE_SPEED_STEP       0.05f     //线速度加减的步长（rpm)
#define ROTATION_SPEED_STEP   5.0f      //角速度加减的步长（rpm)
#define MOVE_SPEED_MAX        1.0f      //线速度最大值（rpm）
#define ROTATION_SPEED_MAX    100.0f    //角速度最大值（rpm）
#define ROTATION_MODULATION_ANGULAR_FREQUENCY 0.3f  //正弦调制角频率（rad/s）
#define ROTATION_MODULATION_TWO_PI 6.28318530718f

static float clamp_float(float value, float min_value, float max_value)         //限幅函数
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static void put_int16_le(uint8_t *buffer, int16_t value)
{
    uint16_t unsigned_value = (uint16_t)value;

    buffer[0] = (uint8_t)(unsigned_value & 0xFFU);
    buffer[1] = (uint8_t)(unsigned_value >> 8);
}
void Remote_Init(void){
    Key_Init();
    nrf24l01_send_init();
}

void date_pack_process(void)
{
    uint16_t move_up_count;
    uint16_t move_down_count;
    uint16_t rotation_up_count;
    uint16_t rotation_down_count;
    static uint32_t rotation_modulation_last_tick = 0U;
    static float rotation_modulation_phase = 0.0f;
    static uint8_t rotation_modulation_was_enabled = 0U;
    uint32_t current_tick;

    Key_process();

    move_up_count = key_data[5];
    move_down_count = key_data[4];
    rotation_up_count = key_data[6];
    rotation_down_count = key_data[7];

    move_speed = ((float)move_up_count - (float)move_down_count) * MOVE_SPEED_STEP;
    rotation_speed = ((float)rotation_up_count - (float)rotation_down_count) * ROTATION_SPEED_STEP;
    move_speed = clamp_float(move_speed, 0.0f, MOVE_SPEED_MAX);
    rotation_speed = clamp_float(rotation_speed, 0.0f, ROTATION_SPEED_MAX);

    if (move_speed == 0.0f ) {
        if (move_speed_zero_flag != 0U) {
            Key_ResetMoveSpeedCounts();
        }
        move_speed_zero_flag = 0U;
    } else {
        move_speed_zero_flag = 1U;
    }

    if (rotation_speed == 0.0f ) {
        if (rotation_speed_zero_flag != 0U) {
            Key_ResetRotationSpeedCounts();
        }
        rotation_speed_zero_flag = 0U;
    } else {
        rotation_speed_zero_flag = 1U;
    }

    current_tick = osKernelSysTick();
    if (key_data[KEY_E] != 0U) {
        if (rotation_modulation_was_enabled == 0U) {
            rotation_modulation_phase = 0.0f;
            rotation_modulation_last_tick = current_tick;
            rotation_modulation_was_enabled = 1U;
        } else {
            uint32_t elapsed_ticks = current_tick - rotation_modulation_last_tick;

            rotation_modulation_phase += ROTATION_MODULATION_ANGULAR_FREQUENCY *
                                         (float)elapsed_ticks /
                                         (float)osKernelSysTickFrequency;
            while (rotation_modulation_phase >= ROTATION_MODULATION_TWO_PI) {
                rotation_modulation_phase -= ROTATION_MODULATION_TWO_PI;
            }
            rotation_modulation_last_tick = current_tick;
        }
        rotation_speed = fabsf(rotation_speed * sinf(rotation_modulation_phase));
    } else {
        rotation_modulation_was_enabled = 0U;
    }

    memset(tx_buffer, 0, sizeof(tx_buffer));
    tx_buffer[0] = 0xA5;
    tx_buffer[1] = (uint8_t)((key_data[0] << 0) |
                             (key_data[1] << 1) |
                             (key_data[2] << 2) |
                             (key_data[3] << 3));
    put_int16_le(&tx_buffer[2], lr_joystick);
    memcpy(&tx_buffer[4], &move_speed, sizeof(move_speed));
    memcpy(&tx_buffer[8], &rotation_speed, sizeof(rotation_speed));
    memcpy(&tx_buffer[12], &lr_move_sensitivity, sizeof(lr_move_sensitivity));
    tx_buffer[17] = 0x5A;
}

void Remote_Send(void)
{
    memcpy(NRF24L01_TxPacket, tx_buffer, NRF24L01_TX_PACKET_WIDTH);
    SendFlag=NRF24L01_Send();
}

void remote_task(void const * argument)                 //遥控器任务
{
    Remote_Init();
    osDelay(1000);                                      //等待按键初始化及OLED初始化完成
    while(1)
    {
        date_pack_process();   
        Remote_Send();
        osDelay(10);
    }
}
