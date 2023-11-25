#ifndef _ULTRASONIC_SENSOR_H_
#define _ULTRASONIC_SENSOR_H_

#include <stdint.h>

#define TRIGGER_PIN         24
#define ECHO_PIN            26
#define LIMIT_DISTANCE      20
#define THETA               0.85
void Init(void);
void Distance_Sensor_Setting(void);
uint16_t GetDistance(void);
uint16_t LPF_Distance(uint16_t prev_estimated_value);

#endif