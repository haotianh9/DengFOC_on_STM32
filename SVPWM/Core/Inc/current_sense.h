/*
 * current_sense.h
 *
 *  Created on: Dec 24, 2023
 *      Author: haotian
 */

#ifndef INC_CURRENT_SENSE_H_
#define INC_CURRENT_SENSE_H_

#include "stm32f1xx_hal.h"
#include <math.h>






void ADC_Select_CH0(ADC_HandleTypeDef);
void ADC_Select_CH1(ADC_HandleTypeDef);
void ADC_Select_CH2(ADC_HandleTypeDef);
void read_ADC_voltage(ADC_HandleTypeDef ,uint16_t *);
void calibrateOffsets(ADC_HandleTypeDef,uint16_t *);

#endif /* INC_CURRENT_SENSE_H_ */
