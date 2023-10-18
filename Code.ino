#include "MotorControlSetting.h"
//#define SERIAL_PRINT

extern uint8_t state;

uint32_t velocity = 0;
// distance variable
uint16_t distance = 0;
uint16_t filtered_distance = 0;

// frequency variable
extern float frequency;
extern uint8_t iteration;
extern uint8_t ovf_cnt;
extern uint32_t cnt;

// current variable
float cur_u;
float cur_v;
float filtered_cur_u;
float filtered_cur_v;

// loadcell variable
extern float input_force;
float filtered_input_force = 0;

// contorller variable
float i = 0;
float prev_i = 0;
float i_hat = 0;
float w_hat = 0;
float d_hat = 0;
float prev_i_hat = 0;
float prev_w_hat = 0;
float prev_d_hat = 0;
float u = 0;   
float prev_u = 0;
float err = 0; 
float prev_err = 0; 
float err_hat = 0;
float prev_err_hat = 0;
float pwm = 0;
float input = 0;
// PI controller gain
float Kp = 10;
float Ki = 850; //((Ra*Bm+Kt*Kb)/(Ra*Jm))*Kp;


// Controller
const float Kt = 0.03;
const float Kb = 0.03;
const float R = 1.2;
const float L = 1e-3;
const float L2 = 5; // 수정 필요
const float Bm = 1e-4;
const float Jm = 1e-5;
const float timebase = 0.001;

void setup(){
  Serial.begin(460800);
  GPIO_Setting();
  FND_Setting();
  LoadCell_Setting();
  PWM_Setting();
  Interrupt_Setting();
  Serial.println("setup finish");
}

void loop(){

}

void TC0_Handler(){
  REG_TC0_SR0;
  velocity = Get_Velocity(Get_Frequency());
}

// Controller
void TC4_Handler(){                   // 10ms
  REG_TC1_SR1;                        // flag clear
  // ref = Set_Input();
  prev_i = i;
  prev_i_hat = i_hat;
  prev_w_hat = w_hat;
  prev_d_hat = d_hat;

  prev_err = err;
  err = input_force - i;
  prev_d_hat = d_hat;
  d_hat = i - i_hat;                 // err_hat = current - current_hat;
  prev_u = u;

  // PI controller 
  u = prev_u + ((Kp + (Ki * timebase / 2)) * err) + ((-Kp + (Ki * timebase / 2)) * prev_err) - ((L2 * timebase / 2) * (prev_err_hat + err_hat));
  // u = prev_u + ((Kp + (Ki * timebase / 2)) * err) + ((-Kp + (Ki * timebase / 2)) * prev_err);
  
  // OBS 극점 -25, timestep=0.1ms
  i_hat = (( -0.0686 * prev_i) + ( 0.9935 * prev_i_hat) + (-0.0019 * prev_w_hat) + (0.0623 * prev_d_hat)) + (0.0311 * (u + prev_u));
  w_hat = (( 0.3017 * prev_i) + ( -0.0018 * prev_i_hat) + ( 0.9990 * prev_w_hat) + (-0.0001 * prev_d_hat)) + (0.0000 * (u + prev_u)); 
  d_hat = (( 0.0003 * prev_i) + ( -0.0002 * prev_i_hat) + ( 0.0000 * prev_w_hat) + (1.0000 * prev_d_hat)) + (0.0000 * (u + prev_u)); 

  // Saturation
  if(u > 4.8)
    u = 4.8;
  else if(u < 0) // 반대방향으로 굴러가는 수레로 수정필요
    u = 0;

  pwm = u / 5;
  input = TOP * pwm;
}

// FND
void TC5_Handler(){                 // 10ms
  REG_TC1_SR2;                      // flag clear
  Display_FND(distance);
}

// loadcell
void TC6_Handler(){                 // 100ms
  REG_TC2_SR0;                      // flag clear
  input_force = Get_Force();      
  if(state == TRUE){                // system on
    PWM_VALUE_RIGHT = input_force * 3000;
    PWM_VALUE_LEFT = input_force * 3000;
    // PWM_VALUE_RIGHT = input;
    // PWM_VALUE_LEFT = input;
  }
  else{
    System_Off();
  }
}

// current measurement
void TC7_Handler(){                  // 10ms
  REG_TC2_SR1;                       // flag clear
  // row data
  // cur_u = GetCurrent(ADC_PIN1, TIMES);
  // cur_v = GetCurrent(ADC_PIN2, TIMES);
  // // filtered data
  // filtered_cur_u = LPF_Current(filtered_cur_u, 0.1, ADC_PIN1);
  // filtered_cur_v = LPF_Current(filtered_cur_v, 0.1, ADC_PIN2);
}

// ultrasonic sensor
void TC8_Handler(){                                         // 500ms
  REG_TC2_SR2;                                              // flag clear
  distance = GetDistance();                                 // row data
  filtered_distance = LPF_Distance(filtered_distance, 0.1); // filtered data
  if(distance < LIMIT_DISTANCE){                            // waring situlation
    System_Off();                                           // system off
  }
}
