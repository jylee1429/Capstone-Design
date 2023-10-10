#ifndef _ULTRASONIC_SENSOR_H_
#define _ULTRASONIC_SENSOR_H_

#include <stdint.h>

#define TRIGGER_PIN 4
#define ECHO_PIN 3
#define LIMIT_DISTANCE 5

uint16_t GetDistance(void);
uint16_t LPF_Distance(uint16_t prev_filter_val, float theta);

#endif