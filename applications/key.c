#include "key.h"
#include "main.h"
#include "nrf24l01_recive.h"
#include "cmsis_os.h"

uint8_t key_data[KEY_COUNT] = {0};
KeyInfo keyinfo={0};                                     //中断方法预留    
static volatile uint8_t key_state[KEY_COUNT] = {0};      //记录按键功能状态
static volatile uint8_t key_count[KEY_COUNT] = {0};      //记录按键按下次数
volatile uint8_t move_speed_zero_flag = 0U;
volatile uint8_t rotation_speed_zero_flag = 0U;

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_PinState pressed_state;
} KeyPinConfig;

static const KeyPinConfig key_pin_config[KEY_COUNT] = {         //按键注册表
    {GPIOA, K1_Pin, GPIO_PIN_RESET},
    {GPIOA, K2_Pin, GPIO_PIN_RESET},
    {GPIOB, K3_Pin, GPIO_PIN_RESET},
    {GPIOB, K4_Pin, GPIO_PIN_RESET},
    {GPIOB, K5_Pin, GPIO_PIN_SET},
    {GPIOB, K6_Pin, GPIO_PIN_SET},
    {GPIOA, K7_Pin, GPIO_PIN_SET},
    {GPIOA, K8_Pin, GPIO_PIN_SET},
    {GPIOB, K9_Pin, GPIO_PIN_RESET}
};

static int8_t key_index_from_pin(uint16_t gpio_pin)             //读取按键编号
{
    uint8_t index;

    for (index = 0; index < KEY_COUNT; index++) {
        if (key_pin_config[index].pin == gpio_pin) {
            return (int8_t)index;
        }
	}

    return -1;
}

/**
 * @brief  根据GPIO引脚获取按键配置信息
 * 
 * 该函数通过GPIO引脚号查找对应的按键索引，并返回该按键的配置结构体指针。
 * 如果找不到对应的按键配置，则返回NULL。
 * 
 * @param gpio_pin GPIO引脚号（如K1_Pin, K2_Pin等）
 * @return const KeyPinConfig* 按键配置结构体指针，如果未找到对应配置则返回NULL
 */
static const KeyPinConfig *key_config_from_pin(uint16_t gpio_pin)
{
    int8_t key_index = key_index_from_pin(gpio_pin);

    if (key_index < 0) {
        return NULL;
    }

    return &key_pin_config[(uint8_t)key_index];
}


uint8_t Key_GPIO_Read(uint16_t gpio_pin)
{
    const KeyPinConfig *config = key_config_from_pin(gpio_pin);

    if (config == NULL) {
        return 0;
    }

    return (HAL_GPIO_ReadPin(config->port, config->pin) == config->pressed_state) ? 1U : 0U;
}


void Key_Init(void)
{
	keyinfo.key_id=0;
    move_speed_zero_flag = 0U;
    rotation_speed_zero_flag = 0U;
    key_state[Clockwise] = GPIO_PIN_RESET;
    key_state[counterclockwise] = GPIO_PIN_RESET;
    key_data[2] = GPIO_PIN_RESET;
    key_data[3] = GPIO_PIN_RESET;
    key_state[KEY_E] = GPIO_PIN_RESET;
    key_data[KEY_E] = GPIO_PIN_RESET;
}

void Key_ResetMoveSpeedCounts(void)
{
    key_count[SPEED_UP] = 0U;
    key_count[SPEED_DOWN] = 0U;
}

void Key_ResetRotationSpeedCounts(void)
{
    key_count[WSPEED_UP] = 0U;
    key_count[WSPEED_DOWN] = 0U;
}

static void key_sync_initial_state(uint8_t counter_state[KEY_COUNT],
                                   uint8_t last_state[KEY_COUNT])
{
    counter_state[Clockwise] = Key_GPIO_Read(key_pin_config[Clockwise].pin);
    last_state[Clockwise] = counter_state[Clockwise];
    counter_state[counterclockwise] =
        Key_GPIO_Read(key_pin_config[counterclockwise].pin);
    last_state[counterclockwise] = counter_state[counterclockwise];
    counter_state[KEY_E] = Key_GPIO_Read(key_pin_config[KEY_E].pin);
    last_state[KEY_E] = counter_state[KEY_E];
}

/**
 * @brief  按键扫描处理线程函数
 * 
 * @details 该函数作为RTOS线程运行，持续扫描所有按键的状态。采用两次采样去抖算法，
 *          在消抖延时前后读取所有按键，只有两次读数一致时才认为状态有效。
 *          检测到按键状态变化时，更新全局按键状态数组key_state[]，并对前4个按键
 *          (index 0-3)的按下次数进行计数。
 * 
 * @param  argument  线程参数指针（未使用）
 * @return 无返回值
 * 
 * @note   该函数包含无限循环，不应直接调用，应作为线程启动
 * @note   消抖延迟时间由KEY_DEBOUNCE_DELAY_TICKS配置
 * @note   key_count[]仅对前4个按键(index 0-3)进行计数
 */
void readkey (void const * argument)    
{
    uint8_t counter_state[KEY_COUNT] = {0};
    uint8_t last_state[KEY_COUNT] = {0};

    key_sync_initial_state(counter_state, last_state);

    while (1) {

        uint8_t index;
        uint8_t key_state_first[KEY_COUNT];
        uint8_t key_state_second[KEY_COUNT];

        for (index = 0; index < KEY_COUNT; index++) {
            key_state_first[index] = Key_GPIO_Read(key_pin_config[index].pin);
        }

        osDelay(KEY_DEBOUNCE_DELAY_TICKS);

        for (index = 0; index < KEY_COUNT; index++) {
            key_state_second[index] = Key_GPIO_Read(key_pin_config[index].pin);

            if (key_state_first[index] == key_state_second[index]) {
                counter_state[index] = key_state_first[index];
            } else {
                counter_state[index] = last_state[index];
            }
        }


        for (index = 0; index < KEY_COUNT ; index++) {
            if (counter_state[index] != last_state[index]) {
                if (counter_state[index] == GPIO_PIN_SET) {
                    if (index < 4U) {
                        if ((index == SPEED_DOWN && move_speed_zero_flag == 0U) ||
                            (index == WSPEED_DOWN && rotation_speed_zero_flag == 0U)) {
                            key_count[index] = 0U;
                        } else {
                            key_count[index]++;
                        }
                    }
                    if (index == Clockwise || index == counterclockwise) {
                        key_state[(index == Clockwise) ? counterclockwise : Clockwise] =
                            GPIO_PIN_RESET;
                        key_state[index] = (key_state[index] == GPIO_PIN_SET) ?
                                           GPIO_PIN_RESET : GPIO_PIN_SET;
                    } else if (index == KEY_E) {
                        key_state[index] = (key_state[index] == GPIO_PIN_SET) ?
                                           GPIO_PIN_RESET : GPIO_PIN_SET;
                    } else {
                        key_state[index] = GPIO_PIN_RESET;
                    }
                } else {
                    if (index != Clockwise && index != counterclockwise &&
                        index != KEY_E) {
                        key_state[index] = GPIO_PIN_SET;
                    }
                }
            }

            last_state[index] = counter_state[index];
        }
    }
}




//按键数据处理，将数据存入数据数组中
void Key_process(void)
{

    key_data[0] = (uint8_t)key_state[move];            
    key_data[1] = (uint8_t)key_state[inmove];
    key_data[2] = (uint8_t)key_state[Clockwise];
    key_data[3] = (uint8_t)key_state[counterclockwise];

    key_data[4] = key_count[SPEED_DOWN];                
    key_data[5] = key_count[SPEED_UP];
    key_data[6] = key_count[WSPEED_UP];
    key_data[7] = key_count[WSPEED_DOWN];
    key_data[KEY_E] = (uint8_t)key_state[KEY_E];
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//int8_t key_index;
	if (GPIO_Pin == IRQ_Pin) {
		nrf24l01_irq_callback();
		return;
	}

    //已停用的中断接收按键状态接口，不稳定，通道会串

	// key_index = key_index_from_pin(GPIO_Pin);
	// if (key_index < 0 || key_index >= KEY_COUNT) {
	// 	return;
	// }

	// key_buffer[(uint8_t)key_index] ++;

}
