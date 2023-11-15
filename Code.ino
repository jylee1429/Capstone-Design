#include "MotorControlSetting.h"

uint16_t cnt = 0;
// velocity
uint16_t vel = 0;

// distance variable
uint16_t distance = 0;

// current variable
float cur_u;
float cur_v;

// loadcell
int32_t input_force_right = 0;
int32_t input_force_left = 0;
// pwm value
uint16_t output_right = 0;
uint16_t output_left = 0;

void setup(){
  Serial.begin(250000);
  Wire.begin();                         // I2C Master
  GPIO_Setting();
  // LED_TEST();
  FND_Setting();
  LoadCell_Setting();
  PWM_Setting();
  Interrupt_Setting();
  Serial.println("setup finish");
}

void loop(){
}

// velocity
void TC0_Handler(){                            // 40ms        
  REG_TC0_SR0;  
  digitalWrite(TEST_LED_1, HIGH);              // red
  cnt++;
  digitalWrite(TEST_LED_1, LOW);
}

// loadcell
void TC3_Handler(){                            // 50ms        
  REG_TC1_SR0;
  digitalWrite(TEST_LED_2, HIGH);              // yellow
  input_force_right = Get_Force_Right();
  input_force_left = Get_Force_Left();
  Serial.println(input_force_right);
  // cnt++;
  digitalWrite(TEST_LED_2, LOW);               
}

// Controller
void TC4_Handler(){                            // 5ms
  REG_TC1_SR1; 
  // digitalWrite(TEST_LED_3, HIGH);           // white
  output_right = Controller(input_force_right);
  output_left = Controller(input_force_left);
  
  if(Is_OnOff() == true) {                        
    PWM_VALUE_LEFT = output_left;
    PWM_VALUE_RIGHT = output_right;
  }
  // cnt++;
  // digitalWrite(TEST_LED_3, LOW);
}

// FND
void TC5_Handler(){                           // 20ms
  REG_TC1_SR2;                                // flag clear
  // Display_FND(ABS(input_force_right) * 10);
  Display_FND(cnt);
}

void TC8_Handler(){                           // 100ms
  REG_TC2_SR2;                                // flag clear
  digitalWrite(TEST_LED_4, HIGH);             // blue
  distance = GetDistance();
  if(distance < LIMIT_DISTANCE){
    System_Off();
  }
  // cnt++;
  digitalWrite(TEST_LED_4, LOW);
}