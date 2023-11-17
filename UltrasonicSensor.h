#ifndef _ULTRASONIC_SENSOR_H_
#define _ULTRASONIC_SENSOR_H_

#include <stdint.h>

#define TRIGGER_PIN 7
#define ECHO_PIN 8
#define LIMIT_DISTANCE 10

uint16_t GetDistance(void);
uint16_t LPF_Distance(uint16_t prev_filter_val, float theta);

#endif