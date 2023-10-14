#ifndef _MOTOR_CONTROL_SETTING_H_
#define _MOTOR_CONTROL_SETTING_H_

#include "FND.h"
#include "LoadCell.h"
#include "Current_Measure.h"
#include "UltrasonicSensor.h"
#include "Arduino.h"

#define TRUE 1
#define FALSE 0
#define SYS_CLK 84e+6                         // system clock
#define PWM_RIGHT_PIN 43            	        // pin number
#define PWM_LEFT_PIN 40		                    // pin number
#define TOP 8400                              // 10KHZ
#define VAL_500MS 21000000                    // 500ms
#define VAL_100MS 4200000                     // 100ms
#define VAL_50MS 2100000                      // 50ms
#define VAL_10MS 420000                       // 10ms
#define VAL_1MS 42000                         // 1ms
#define VAL_01MS 4200                         // 0.1ms
#define FILTER_WEIGHT 0.1                     // weight
#define SW_PIN 2                              // switch pin number
#define LED_PIN 12                            // test LED
#define DIR 4                                 // direction
#define PWM_VALUE_RIGHT REG_PWM_CDTY2
#define PWM_VALUE_LEFT REG_PWM_CDTY3

// Controller
#define Kt 0.03
#define Kb 0.03
#define R 1.2
#define L 1e-3
#define L2 5 // 수정 필요
#define Bm 1e-4
#define Jm 1e-5
#define TIMEBASE 0.001

#endif
