/*
 * HCSR04.h
 */

#ifndef INC_HCSR04_H_
#define INC_HCSR04_H_

#include "stm32l4xx_hal.h"

#define HCSR04_UNITS  2

typedef struct {
	GPIO_TypeDef * TRIG_GPIO;
	uint16_t       TRIG_PIN;
	TIM_TypeDef*   TIM_Instance;
	uint32_t       IC_TIM_CH;
	uint32_t       TIM_CLK_MHz;
} HCSR04_CfgType;

void HCSR04_Init(uint8_t au8_HCSR04_Instance, TIM_HandleTypeDef* TMR_Handle);
void HCSR04_Trigger(uint8_t au8_HCSR04_Instance);
void HCSR04_TMR_OVF_ISR(TIM_HandleTypeDef* htim);
void HCSR04_TMR_IC_ISR(TIM_HandleTypeDef* htim);
float HCSR04_Read(uint8_t au8_HCSR04_Instance);

#endif /* INC_HCSR04_H_ */
