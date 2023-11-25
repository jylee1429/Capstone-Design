#ifndef _MOTOR_CONTROL_SETTING_H_
#define _MOTOR_CONTROL_SETTING_H_

#include "FND.h"
#include "LoadCell.h"
#include "UltrasonicSensor.h"
#include "Controller.h"
#include "Arduino.h"
#include "Wire.h"
#include "INA219.h"

#define OVF_CNT                                       // overflow cnt
#define SYS_CLK         84e+6                         // system clock
#define PWM_RIGHT_PIN   43            	              // pin number
#define PWM_LEFT_PIN    41		                        // pin number
#define ON_OFF_LED      22                            // on-off led
#define TOP             8400                          // 10KHZ
#define VAL_500MS       21000000                      // 500ms
#define VAL_100MS       4200000                       // 100ms
#define VAL_50MS        2100000                       // 50ms
#define VAL_40MS        1680000                       // 40ms
#define VAL_30MS        1260000                       // 30ms
#define VAL_20MS        840000                        // 20ms
#define VAL_15MS        630000                        // 15ms
#define VAL_10MS        420000                        // 10ms
#define VAL_5MS         210000                        // 5ms
#define VAL_2MS         84000                         // 2ms
#define VAL_1MS         42000                         // 1ms
#define VAL_01MS        4200                          // 0.1ms
#define SW_PIN          15                            // switch pin
#define OVF_VAL         4294967295                    // overflow value

#endif
