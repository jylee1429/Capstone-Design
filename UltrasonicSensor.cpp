#include "UltrasonicSensor.h"
#include "Arduino.h"

uint16_t GetDistance(void){
  uint16_t timeout;
  uint16_t distance;

  digitalWrite(TRIGGER_PIN, LOW); 
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN,  HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);                                                     // send the ultrasonic burst

  timeout = pulseIn(ECHO_PIN, HIGH);
  distance = (340 * timeout / 10000) / 2;                                             // [cm]

  return distance;
}

uint16_t LPF_Distance(uint16_t prev_estimated_value, float theta){
  uint16_t estimated_value;
  estimated_value = (prev_estimated_value * theta) + (GetDistance() * (1 - theta));   // 0 < theta < 1
  return estimated_value;
}
