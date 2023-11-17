#ifndef _MOTOR_CONTROL_SETTING_H_
#define _MOTOR_CONTROL_SETTING_H_

#include "FND.h"
#include "LoadCell.h"
#include "UltrasonicSensor.h"
#include "Controller.h"
#include "Arduino.h"
#include "Wire.h"
#include "INA219.h"

#define SYS_CLK 84e+6                         // system clock
#define PWM_RIGHT_PIN 43            	        // pin number
#define PWM_LEFT_PIN 40		                    // pin number
#define ON_OFF_LED 22                         // on-off led
#define TEST_LED_1 10                         // test led(red)
#define TEST_LED_2 11                         // test led(yellow)
#define TEST_LED_3 12                         // test led(white)
#define TEST_LED_4 13                         // test_led(blue)
#define CUR_RIGHT_U 24                        // Hall sensor U(right)
#define CUR_RIGHT_V 26                        // Hall sensor V(right)
#define CUR_RIGHT_W 28                        // Hall sensor W(right)
#define CUR_LEFT_U 25                         // Hall sensor U(left)
#define CUR_LEFT_V 27                         // Hall sensor V(left)
#define CUR_LEFT_W 29                         // Hall sensor W(left)
#define TOP 8400                              // 10KHZ
#define VAL_500MS 21000000                    // 500ms
#define VAL_100MS 4200000                     // 100ms
#define VAL_50MS 2100000                      // 50ms
#define VAL_40MS 1680000                      // 40ms
#define VAL_30MS 1260000                      // 30ms
#define VAL_20MS 840000                       // 20ms
#define VAL_15MS 630000                       // 15ms
#define VAL_10MS 420000                       // 10ms
#define VAL_5MS 210000                        // 5ms
#define VAL_2MS 84000                         // 2ms
#define VAL_1MS 42000                         // 1ms
#define VAL_01MS 4200                         // 0.1ms
#define FILTER_WEIGHT 0.1                     // weight
#define SW_PIN 2                              // switch pin
#define POLE 6                                // Motor pole
#define TIME_INTERVAL 42e+6                   // clock time interval
#define PWM_VALUE_RIGHT REG_PWM_CDTY2
#define PWM_VALUE_LEFT REG_PWM_CDTY3   
#define OVF_VAL 4294967295
#define THETA 0.1
#define DEVICE_NUM 4                          // current devices num

#endif
