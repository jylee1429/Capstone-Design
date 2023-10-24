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
#define VAL_40MS 1680000                      // 40ms
#define VAL_15MS 630000                       // 15ms
#define VAL_10MS 420000                       // 10ms
#define VAL_2MS 84000                         // 2ms
#define VAL_1MS 42000                         // 1ms
#define VAL_01MS 4200                         // 0.1ms
#define FILTER_WEIGHT 0.1                     // weight
#define SW_PIN 13                             // switch pin
#define LED_PIN 12                            // test LED
#define FG_PIN  11                            // velocity pin
#define POLE 4                                // Motor pole
#define TIME_INTERVAL 42e+6                   // clock time interval
#define PWM_VALUE_RIGHT REG_PWM_CDTY2
#define PWM_VALUE_LEFT REG_PWM_CDTY3   

#endif
