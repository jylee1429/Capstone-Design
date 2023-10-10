#include "MotorControl.h"
// #define SERIAL_PRINT 
// #define TEST_LED

HX711 Loadcell;

uint8_t status = FALSE;

// distance variable
uint16_t distance;
uint16_t filtered_distance;

// current variable
float cur_u;
float cur_v;
float filtered_cur_u;
float filtered_cur_v;

// loadcell variable
uint16_t scale_factor = 4782;
float input_force = 0;
float filtered_input_force = 0;

void setup(){
  Serial.begin(460800);
  GPIO_Setting();
  PWM_Setting();
  Interrupt_Setting();
  Loadcell.LoadCell_Setting(scale_factor);
  
  #ifdef SERIAL_PRINT
  Serial.println("Setting finish");
  #endif
}

void loop(){

}
// loadcell
void TC6_Handler(){
  REG_TC2_SR0;  // flag clear
  input_force = Loadcell.Get_Input();
  filtered_input_force = Loadcell.LPF_Input(filtered_input_force, 0.1); // theta = 0.1
  
  if(status == TRUE){
    PWM_VALUE_RIGHT = input_force;
    PWM_VALUE_LEFT = input_force;
  }

  #ifdef SERIAL_PRINT
  Serial.print(input_force);
  Serial.print("\t");
  Serial.println(filtered_input_force);
  #endif
}

// current measure
void TC7_Handler(){           // 10ms
  REG_TC2_SR1;                // flag clear
  // row data
  cur_u = GetCurrent(ADC_PIN1, TIMES);
  cur_v = GetCurrent(ADC_PIN2, TIMES);
  // filtered data
  filtered_cur_u = LPF_Current(filtered_cur_u, 0.1, ADC_PIN1);
  filtered_cur_v = LPF_Current(filtered_cur_v, 0.1, ADC_PIN2);

  #ifdef SERIAL_PRINT
  digitalWrite(3, HIGH);   // test led 
  Serial.print(cur_u);
  Serial.print("\t");
  Serial.println(cur_v);
  Serial.print(filtered_cur_u);
  Serial.print("\t");
  Serial.println(filtered_cur_v);
  #endif
}

// ultrasonic sensor
void TC8_Handler(){           // 100ms
  REG_TC2_SR2;                // flag clear
  // row data
  distance = GetDistance();
  // filtered data
  filtered_distance = LPF_Distance(filtered_distance, 0.1); // theta = 0.1

  if(filtered_distance < LIMIT_DISTANCE){
    System_Off();
  }
  
  #ifdef SERIAL_PRINT
  Serial.print(distance);
  Serial.print("\t");
  Serial.println(filtered_distance);
  #endif
}
