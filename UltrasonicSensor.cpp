#include "UltrasonicSensor.h"
#include "Arduino.h"

void Init(void){
  pinMode(TRIGGER_PIN, OUTPUT);                 // ultrasonic sensor-trigger pin
  pinMode(ECHO_PIN, INPUT);                     // ultrasonic sensor-echo pin     
}


void Distance_Sensor_Setting(void){
  Init();
}

uint16_t GetDistance(void){
  uint16_t pulse_time;
  uint16_t distance;

  // ultrasonic start signal
  digitalWrite(TRIGGER_PIN, LOW); 
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN,  HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);                                                     
  // send the ultrasonic burst
  pulse_time = pulseIn(ECHO_PIN, HIGH);
  distance = (340 * pulse_time / 10000) / 2;                                              // [cm]

  return distance;

}

uint16_t LPF_Distance(uint16_t prev_estimated_value){
  uint16_t estimated_value;
  estimated_value = (prev_estimated_value * THETA) + (GetDistance() * (1 - THETA));   // 0 < theta < 1
  return estimated_value;
}
