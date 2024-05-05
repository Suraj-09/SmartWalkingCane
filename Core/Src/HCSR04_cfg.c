/*
 * HCSR04_cfg.c
 */


#include "HCSR04.h"

const HCSR04_CfgType HCSR04_CfgParam[HCSR04_UNITS] =
{
	// HC-SR04 Sensor Unit 1 Configurations
    {
		GPIOB,
		GPIO_PIN_12,
		TIM15,
		TIM_CHANNEL_1,
		72
	},
	{
		GPIOB,
		GPIO_PIN_13,
		TIM15,
		TIM_CHANNEL_2,
		72
	}
};
