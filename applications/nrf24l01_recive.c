#include "nrf24l01_recive.h"
#include <string.h>

// ==================== 全局变量 ====================
uint8_t cmd_move             = 0;  // bit 0 : 前进
uint8_t cmd_u_move        = 0;  // bit 1 : 后退
uint8_t cmd_clockwise        = 0;  // bit 2 : 顺时针旋转
uint8_t cmd_counterclockwise = 0;  // bit 3 : 逆时针旋转

int16_t lr_joystick     = 0;       // short : 左右移动拨杆值

float   move_speed      = 0.0f;    // float 0 : 移动速度
float   rotation_speed  = 0.0f;    // float 1 : 旋转速度
float   lr_move_sensitivity  = 0.0f;    // float 2 : 左右移动灵敏度

// 接收缓冲
static uint8_t rx_buffer[NRF24L01_RX_PACKET_WIDTH];
static volatile uint8_t rx_pending = 0;
static uint32_t last_valid_rx_tick = 0;
static uint8_t link_lost_latched = 0;
uint8_t ReceiveFlag;

void nrf24l01_recive_init(void) { 
    NRF24L01_Init();
    rx_pending = 0;
    last_valid_rx_tick = 0;
    link_lost_latched = 0;
    ReceiveFlag=0;
}

//基于中断的数据接收触发的数据处理
void nrf24l01_recive_process(void)
{
    if (!rx_pending) {
        return;
    }

    rx_pending = 0;
    ReceiveFlag=NRF24L01_Receive();
    if ( ReceiveFlag!= 1) {
        return;
    }

    if (NRF24L01_RxPacket[0] != 0xA5) {
        return;
    }

    memcpy(rx_buffer, NRF24L01_RxPacket, NRF24L01_RX_PACKET_WIDTH);
    if (rx_buffer[NRF24L01_RX_PACKET_WIDTH - 1] != 0x5A) {
        return;
    }
    //建议添加CRC校验，此处未添加
    uint8_t bool_byte = rx_buffer[1];
    cmd_move = (bool_byte >> 0) & 1;
    cmd_u_move = (bool_byte >> 1) & 1;
    cmd_clockwise = (bool_byte >> 2) & 1;
    cmd_counterclockwise = (bool_byte >> 3) & 1;
    lr_joystick = (int16_t)((rx_buffer[3] << 8) | rx_buffer[2]);
    memcpy(&move_speed, &rx_buffer[4], 4);
    memcpy(&rotation_speed, &rx_buffer[8], 4);
    memcpy(&lr_move_sensitivity, &rx_buffer[12], 4);
    last_valid_rx_tick = HAL_GetTick();
    link_lost_latched = 0;
}

uint8_t nrf24l01_recive_link_alive(void)
{
    if (last_valid_rx_tick == 0U) {
        return 0U;
    }

    return ((uint32_t)(HAL_GetTick() - last_valid_rx_tick) <= NRF24L01_LINK_TIMEOUT_MS);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != GPIO_PIN_0) {
        return;
    }

    rx_pending = 1;
}

void nrf24l01_recive_link_lost_process(void)
{
    if (nrf24l01_recive_link_alive()) {
        return;
    }

    if (!link_lost_latched) {
        NRF24L01_Rx();
        link_lost_latched = 1;
    }

    cmd_move = 0;
    cmd_u_move = 0;
    cmd_clockwise = 0;
    cmd_counterclockwise = 0;
    lr_joystick = 0;
    move_speed = 0.0f;
    rotation_speed = 0.0f;
    lr_move_sensitivity = 0.0f;

    
}
