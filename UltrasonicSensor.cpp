#include "UltrasonicSensor.h"
#include "Arduino.h"

uint16_t GetDistance(void){
  uint16_t timeout, distance;

  digitalWrite(TRIGGER_PIN, LOW); 
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN,  HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW); // send the ultrasonic burst

  timeout = pulseIn(ECHO_PIN, HIGH);
  distance = (340 * timeout / 10000) / 2; // [cm]
  
  return distance;
}

uint16_t LPF_Distance(uint16_t prev_filter_val, float theta){
  uint16_t filter_val;
  filter_val = prev_filter_val * (1 - theta) + GetDistance() * theta;
  return filter_val;
}
