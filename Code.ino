#include "MotorControlSetting.h"

//velocity
const uint32_t _F_CPU = F_CPU / 2;
volatile uint32_t period_cnt0, period_cnt7, capture_cnt0, capture_cnt7;
uint8_t ovf_flag = false;
// current
float current_right;
float current_left;

// distance variable
uint16_t distance = 0;
uint16_t prev_distance = 0;
// loadcell
int32_t input_force_right = 0;
int32_t input_force_left = 0;

// pwm value
uint16_t output_right = 0;
uint16_t output_left = 0;
extern float pwm_percentage_right;
extern float pwm_percentage_left;
// print flag
uint8_t print_flag = false;

void setup(){
  Serial.begin(250000);
  GPIO_Setting();
  Current_Sensor_Setting();
  Distance_Sensor_Setting();
  FND_Setting();
  LoadCell_Setting();
  PWM_Setting();
  Interrupt_Setting();
  Serial.println("setup finish");
}

void loop(){
  if(print_flag == true){
    print_flag = false;
    Serial.println();
  }
}

// velocity (pin2)
void TC0_Handler() {
  static uint32_t prev_capture_cnt0;
  uint32_t status = REG_TC0_SR0;

  if (status & TC_SR_LDRAS) {
    capture_cnt0 = (uint32_t)REG_TC0_RA0;
    if(ovf_flag == true){
      ovf_flag = false;
      period_cnt0 = capture_cnt0 + OVF_CNT - prev_capture_cnt0;
    }
    else
      period_cnt0 = capture_cnt0 - prev_capture_cnt0;
    prev_capture_cnt0 = capture_cnt0;
  }
  else if(status & TC_SR_COVFS){
    ovf_flag = true;
  }
}

//current
void TC1_Handler(){
  REG_TC0_SR1;  
  current_right = Calculate_Current_Right();   // get current
  current_left = Calculate_Current_Left();      
}

// loadcell
void TC3_Handler(){                            // 100ms        
  REG_TC1_SR0;
  input_force_right = Get_Force_Right();
  input_force_left = Get_Force_Left();        
}

// Controller
void TC4_Handler(){                            // 5ms
  REG_TC1_SR1; 
  output_right = Controller(input_force_right);
  output_left = Controller(input_force_left);
  
  if(Is_OnOff() == true) {                        
    pwm_percentage_left = output_left;
    pwm_percentage_right = output_right;
  }
}

// Print
void TC5_Handler(){                           // 20ms
  REG_TC1_SR2;                                // flag clear
  print_flag = true;
}
// velocity (pin3)
void TC7_Handler(){                                  
  static uint32_t prev_capture_cnt7;
  uint32_t status = REG_TC2_SR1;

  if (status & TC_SR_LDRAS) {
    capture_cnt7 = (uint32_t)REG_TC2_RA1;
    if(ovf_flag == true){
      ovf_flag = false;
      period_cnt7 = capture_cnt7 + OVF_CNT - prev_capture_cnt7;
    }
    else
      period_cnt7 = capture_cnt7 - prev_capture_cnt7;
    prev_capture_cnt7 = capture_cnt7;
  }
  else if(status & TC_SR_COVFS){
    ovf_flag = true;
  }
}

// ultrasonic sensor
void TC8_Handler(){                           // 200ms
  REG_TC2_SR2;                                // flag clear
  distance = LPF_Distance(prev_distance);
  // Ensure Safety Distance
  if(distance < LIMIT_DISTANCE){
    System_Off();
  }
  prev_distance = distance;
}