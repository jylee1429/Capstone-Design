#include "Current_Measure.h"
#include <Arduino.h>

float GetVoltage(uint16_t pin, uint16_t times) {
  float vol = 0;
  float v_out;
  uint16_t i;

  for (i = 0; i < times; i++)
    vol += analogRead(pin); // convert the signal ranging from 0 to 5 volts input through an analog pin into an integer value within the range of 0 to 1023
    // average : vol/64
  v_out = (vol / (float)times) * VOL_PER_RESO;
  return v_out;
}

float GetCurrent(uint16_t pin, uint16_t times) {
  float vol;
  float current;
  
  vol = GetVoltage(pin, times) - OFFSET; // v_out - offset
  current = vol / AMP_PER_RES;
  return current;
}

// Low-Pass Fillter Current
float LPF_Current(float prev_filter_val, float theta, uint16_t pin){
  float filter_val;
  filter_val = prev_filter_val * (1 - theta) + GetCurrent(pin, TIMES) * theta;
  return filter_val;
}