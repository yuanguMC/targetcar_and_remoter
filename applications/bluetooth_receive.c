// #include "bluetooth_receive.h"
// #include "usart.h"
// #include <stdio.h>
// #include <string.h>

// // 发送完成数据的函数
// void Bluetooth_Send_BigRune_Result(uint8_t hits, float average)
// {
//     char tx_buffer[32];
//     int len = snprintf(tx_buffer, sizeof(tx_buffer), "H:%d,A:%.1f\n", hits, average);
//     HAL_UART_Transmit(&huart1, (uint8_t*)tx_buffer, len, HAL_MAX_DELAY);
// }

// // ==================== 全局变量 ====================
// uint8_t cmd_move             = 0;  // bit 0 : 前进
// uint8_t cmd_stop_move        = 0;  // bit 1 : 后退
// uint8_t cmd_clockwise        = 0;  // bit 2 : 顺时针旋转
// uint8_t cmd_counterclockwise = 0;  // bit 3 : 逆时针旋转

// int16_t lr_joystick     = 0;       // short : 左右移动拨杆值

// float   move_speed      = 0.0f;    // float 0 : 移动速度
// float   rotation_speed  = 0.0f;    // float 1 : 旋转速度
// float   lr_move_sensitivity  = 0.0f;    // float 2 : 左右移动灵敏度

// // 接收缓冲
// static uint8_t rx_byte;
// static uint8_t rx_buffer[18];
// static uint8_t rx_index = 0;

// 初始化函数
// void Bluetooth_Init(void)
// {
//     HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
// }

// // UART 接收完成回调
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//     if (huart == &huart1)
//     {
//         if (rx_index == 0)
//         {
//             if (rx_byte == 0xA5)  // 包头
//             {
//                 rx_buffer[0] = rx_byte;
//                 rx_index = 1;
//             }
//         }
//         else
//         {
//             rx_buffer[rx_index++] = rx_byte;

//             if (rx_index == 18)  // 收到完整 18 字节
//             {
//                 if (rx_buffer[17] == 0x5A)  // 包尾
//                 {
//                     uint8_t checksum = 0;
//                     for (uint8_t i = 1; i < 16; i++)  
//                     {
//                         checksum += rx_buffer[i];
//                     }

//                     if (checksum == rx_buffer[16])  // 校验
//                     {
//                         uint8_t bool_byte = rx_buffer[1];
//                         cmd_move          = (bool_byte >> 0) & 1;
//                         cmd_stop_move         = (bool_byte >> 1) & 1;
//                         cmd_clockwise        = (bool_byte >> 2) & 1;
//                         cmd_counterclockwise = (bool_byte >> 3) & 1;
//                         lr_joystick = (int16_t)((rx_buffer[3] << 8) | rx_buffer[2]);
//                         memcpy(&move_speed,     &rx_buffer[4],  4);   // float 0：移动速度
//                         memcpy(&rotation_speed, &rx_buffer[8],  4);   // float 1：旋转速度
//                         memcpy(&lr_move_sensitivity, &rx_buffer[12], 4);   // float 2：左右移动灵敏度
//                     }
//                 }
//                 rx_index = 0;  // 复位，准备接收下一包
//             }
//         }

//         HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
//     }
// }

// // 主循环处理函数（示例）
// void Bluetooth_Process(void)
// {

// }
