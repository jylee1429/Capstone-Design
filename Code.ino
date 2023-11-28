#include "MotorControlSetting.h"

//velocity
volatile uint32_t period_cnt0, period_cnt7, capture_cnt0, capture_cnt7;
uint8_t ovf_flag0 = false;
uint8_t ovf_flag7 = false;
float vel_right = 0;
float vel_left = 0;
uint8_t vel_flag0 = false;
uint8_t vel_flag7 = false;
uint8_t ovf0 = 0;
uint8_t ovf7 = 0;
// current
float current_right = 0;
float current_left = 0;   
extern uint8_t current_flag;

// distance variable
uint16_t distance = 0;
uint16_t prev_distance = 0;

// loadcell
float input_force_right = 0;
float input_force_left = 0;

// pwm value
float val_right = 0.10;
float val_left = 0.05;

extern float pwm_percentage_right;
extern float pwm_percentage_left;

// print flag
uint8_t print_flag = false;

void setup(){
  Serial.begin(250000);
  PWM_Setting();                                              // pwm setting
  GPIO_Setting();                                             // GPIO pin mode setting
  // Current_Sensor_Setting();                                // INA219 current sensor setting
  Distance_Sensor_Setting();                                  // HC-SR04  ultrasonic sensor setting 
  LoadCell_Setting();                                         // HX711 loadcell setting
  Interrupt_Setting();                                        // interrupt setting
  Serial.println("setup finish");           
}

void loop(){
  if(print_flag == true){
    print_flag = false;
    Serial.print("right : ");
    Serial.print(input_force_right);
    Serial.print("\t");
    Serial.print("left : ");
    Serial.print(input_force_left);
    // Serial.print(distance);
    Serial.print("\t");
    Serial.println(Is_OnOff());
  }
}

// velocity-right (pin2)
void TC0_Handler() {
  static uint32_t prev_capture_cnt0;
  uint32_t status = REG_TC0_SR0;

  if (status & TC_SR_LDRAS) {
    capture_cnt0 = (uint32_t)REG_TC0_RA0;
    if(ovf_flag0 == true){
      ovf_flag0 = false;
      period_cnt0 = (ovf0 * OVF_VAL) + capture_cnt0 - prev_capture_cnt0;
      ovf0 = 0;
    }
    else{
      period_cnt0 = capture_cnt0 - prev_capture_cnt0;
    }
    prev_capture_cnt0 = capture_cnt0;
    vel_flag0 = true;
  }
  else if (status & TC_SR_COVFS){
    ovf_flag0 = true;
    ovf0++;
  }
}

// loadcell
void TC3_Handler(){                                           // 100ms        
  REG_TC1_SR0;
  input_force_right = Get_Force_Right();
  input_force_left = Get_Force_Left();        
}

// Controller
void TC4_Handler(){                                                          // 5ms
  REG_TC1_SR1; 
  
  if(Is_OnOff() == true) {                        
    // pwm_percentage_right = Controller(4, vel_right);
    // pwm_percentage_left = Controller(4, vel_left);

    PWM_Right_Value_Set(input_force_right * val_right);
    PWM_Left_Value_Set(input_force_left * val_left);
  }
  else{
    PWM_Right_Value_Set(0);
    PWM_Left_Value_Set(0);    
  }
}

// Print
void TC5_Handler(){                                             // 50ms
  REG_TC1_SR2;                                                  // flag clear
  print_flag = true;
}

// velocity-left (pin3)
void TC7_Handler(){                                  
  static uint32_t prev_capture_cnt7;
  uint32_t status = REG_TC2_SR1;
  
  if (status & TC_SR_LDRAS) {
    capture_cnt7 = (uint32_t)REG_TC2_RA1;
    if(ovf_flag7 == true){
      ovf_flag7 = false;
      period_cnt7 = (ovf7 * OVF_VAL) + capture_cnt7 - prev_capture_cnt7;
      ovf7 = 0;
    }
    else{
      period_cnt7 = capture_cnt7 - prev_capture_cnt7;
    }

    prev_capture_cnt7 = capture_cnt7;
    vel_flag7 = true;
  }
  else if (status & TC_SR_COVFS){
    ovf_flag7 = true;
    ovf7++;
  }
}

// ultrasonic sensor
void TC8_Handler(){                                                  // 200ms
  REG_TC2_SR2;                                                       // flag clear
  distance = LPF_Distance(prev_distance);
  // Ensure Safety Distance
  if(distance < LIMIT_DISTANCE){
    System_Off();
  }
  prev_distance = distance;
}