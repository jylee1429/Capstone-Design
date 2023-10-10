#ifndef _CURRENT_MEASURE_H_
#define _CURRENT_MEASURE_H_

#include <stdint.h>

#define ADC_PIN1 A0               // ADC pin 
#define ADC_PIN2 A2               // ADC pin
#define SHUNT_RESISTANCE 0.005    // shunt register = 0.005 Ohm
#define AMPLIFICATION 50          // 50 amplification
#define RESOLUTUION 1023          // resolution
#define INPUT_VOLTAGE 5           // offset input voltage
#define OFFSET 2.5                // offset voltage
#define VOL_PER_RESO 0.00489      // INPUT_VOLTAGE / RESOLUTUION
#define AMP_PER_RES 2.5           // AMPLIFICATION * SHUNT_RESISTANCE
#define TIMES 1                   // time count

float GetVoltage(uint16_t pin, uint16_t times);
float GetCurrent(uint16_t pin, uint16_t times);
float LPF_Current(float prev_filter_val, float theta, uint16_t pin);

#endif