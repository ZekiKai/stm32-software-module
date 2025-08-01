/**
 * @file drv8833.c
 * @brief DRV8833双H桥电机驱动器控制实现
 */

#include "drv8833.h"
#include "tim.h"

/** @brief IN1引脚的定时器 */
#define IN1_TIM &htim3
/** @brief IN2引脚的定时器 */
#define IN2_TIM &htim3

/** @brief IN1引脚的定时器通道 */
#define IN1_CH TIM_CHANNEL_1
/** @brief IN2引脚的定时器通道 */
#define IN2_CH TIM_CHANNEL_2

/** @brief 最大速度率与最小速度率*/
#define MAX_SPEED_RATE 100
#define MIN_SPEED_RATE 0


/** @brief 默认衰减模式 */
static DecayMode currentDecayMode = SLOW_DECAY;

/**
 * @brief 设置IN1引脚的PWM占空比
 * @param speed_rate 速度率
 */
static inline void __SetIn1SPEED(uint8_t speed_rate)
{
	uint8_t duty = (int)(99 * speed_rate / 100);
    __HAL_TIM_SET_COMPARE(IN1_TIM, IN1_CH, duty);
}

/**
 * @brief 设置IN2引脚的PWM占空比
 * @param speed_rate 速度率
 */
static inline void __SetIn2SPEED(uint8_t speed_rate)
{
	uint8_t duty = (int)(99 * speed_rate / 100);
    __HAL_TIM_SET_COMPARE(IN2_TIM, IN2_CH, duty);
}

/**
 * @brief 初始化DRV8833
 */
void DRV8833_Init(void)
{
    HAL_TIM_PWM_Start(IN1_TIM, IN1_CH);
    HAL_TIM_PWM_Start(IN2_TIM, IN2_CH);
}

/**
 * @brief 设置衰减模式
 * @param mode 衰减模式
 */
void DRV8833_SetDecayMode(DecayMode mode)
{
    currentDecayMode = mode;
}

/**
 * @brief 控制电机前进
 * @param speed 速度值（0-100）
 */
void DRV8833_Forward(uint8_t speed_rate)
{
    if (speed_rate > MAX_SPEED_RATE)
        speed_rate = MAX_SPEED_RATE;
    
    if (currentDecayMode == FAST_DECAY) {
        __SetIn1SPEED(speed_rate);
        __SetIn2SPEED(MIN_SPEED_RATE);
    } else {
        __SetIn1SPEED(MAX_SPEED_RATE);
        __SetIn2SPEED(MAX_SPEED_RATE - speed_rate);
    }
}

/**
 * @brief 控制电机后退
 * @param speed 速度值（0-100）
 */
void DRV8833_Backward(uint8_t speed_rate)
{
    if (speed_rate > MAX_SPEED_RATE)
    	speed_rate = MAX_SPEED_RATE;
    
    if (currentDecayMode == FAST_DECAY) {
        __SetIn1SPEED(MIN_SPEED_RATE);
        __SetIn2SPEED(speed_rate);
    } else {
        __SetIn1SPEED(MAX_SPEED_RATE - speed_rate);
        __SetIn2SPEED(MAX_SPEED_RATE);
    }
}

/**
 * @brief 电机刹车
 */
void DRV8833_Brake(void)
{
    __SetIn1SPEED(MAX_SPEED_RATE);
    __SetIn2SPEED(MAX_SPEED_RATE);
}

/**
 * @brief 电机滑行
 */
void DRV8833_Coast(void)
{
    __SetIn1SPEED(MIN_SPEED_RATE);
    __SetIn2SPEED(MIN_SPEED_RATE);
}
