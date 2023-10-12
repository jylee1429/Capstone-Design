#include "MotorControlSetting.h"
//#define SERIAL_PRINT

uint8_t status = FALSE;

// distance variable
uint16_t distance = 0;
uint16_t filtered_distance = 0;

// current variable
float cur_u;
float cur_v;
float filtered_cur_u;
float filtered_cur_v;

// loadcell variable
float input_force = 0;
float filtered_input_force = 0;
int i;

void PWM_Setting(void);
void Interrupt_Setting(void);

void setup(){
  Serial.begin(460800);
  GPIO_Setting();
  LoadCell_Setting();
  PWM_Setting();
  Interrupt_Setting();
  Serial.println("setup finish");
}

void loop(){

}


// loadcell
void TC6_Handler(){                 // 100ms
  REG_TC2_SR0;                      // flag clear
  input_force = Get_Force();      
  if(status == TRUE){               // system on
    PWM_VALUE_RIGHT = input_force;  
    PWM_VALUE_LEFT = input_force;
  }
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
}

// ultrasonic sensor
void TC8_Handler(){           // 0.5[sec]
  REG_TC2_SR2;                // flag clear
  // row data
  distance = GetDistance();
  // filtered data
  filtered_distance = LPF_Distance(filtered_distance, 0.1); 
  if(filtered_distance < LIMIT_DISTANCE){       // waring situlation
    System_Off();                               // system off
  }
}
