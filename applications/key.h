#ifndef KEY_H
#define KEY_H

#include <stdint.h>
#include "main.h"

#define KEY_COUNT 9

typedef struct {
    GPIO_TypeDef * key_sys;
    int16_t    key_id;
} KeyInfo;                                      //中断方法预留


extern uint8_t key[KEY_COUNT];
extern uint8_t key_data[KEY_COUNT];
extern KeyInfo keyinfo;
extern volatile uint8_t move_speed_zero_flag;
extern volatile uint8_t rotation_speed_zero_flag;

void Key_Init(void);
void Key_process(void);
void Key_ResetMoveSpeedCounts(void);
void Key_ResetRotationSpeedCounts(void);



enum Key_Code           // 按键编号
{
    SPEED_UP,
    SPEED_DOWN,
    WSPEED_UP,
    WSPEED_DOWN,
    Clockwise,
    move,
    counterclockwise,
    inmove,
    KEY_E,
};

#endif
