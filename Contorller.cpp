#include "Controller.h"
#include "MotorControlSetting.h"

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
float w = 0;   
float prev_u = 0;
float prev_w = 0;
float err = 0; 
float prev_err = 0; 
float err_hat = 0;
float prev_err_hat = 0;
float pwm = 0;
float input = 0;

// PI controller gain
float Kp = 10;
float Ki = 850;                         // ((Ra*Bm+Kt*Kb)/(Ra*Jm))*Kp;

// Controller
const float Kt = 0.03;
const float Kb = 0.03;
const float R = 1.2;
const float L = 1e-3;
const float L2 = 5;                     // need change
const float Bm = 1e-4;
const float Jm = 1e-5;
const float timebase = 0.0001;          // 0.1ms


uint16_t Controller(float input){
  uint16_t output;
   // ref = Set_Input();
  prev_i = i;
  prev_w = w;
  prev_i_hat = i_hat;
  prev_w_hat = w_hat;
  prev_d_hat = d_hat;

  prev_err = err;
  err = input - i;
  prev_d_hat = d_hat;
  d_hat = i - i_hat;                           // err_hat = current - current_hat;
  prev_u = u;

  // PI controller 
  u = prev_u + ((Kp + (Ki * timebase / 2)) * err) + ((-Kp + (Ki * timebase / 2)) * prev_err) - ((L2 * timebase / 2) * (prev_err_hat + err_hat));
  
  // OBS pole  = -25, timestep=0.1ms
  i_hat = (( -0.0696 * prev_i) +(-0.0019 * prev_w)+ ( 0.9948 * prev_i_hat) + (0.0000 * prev_w_hat) + (0.0623 * prev_d_hat)) + (0.0312 * (u + prev_u));
  w_hat = (( 0.2996 * prev_i) +(0.0020 * prev_w)+ ( 0.0000 * prev_i_hat) + ( 0.9990 * prev_w_hat) + (-0.0001 * prev_d_hat)) + (0.0000 * (u + prev_u)); 
  d_hat = (( 0.0001 * prev_i) +(0.0000 * prev_w)+ ( -0.0001 * prev_i_hat) + ( 0.0000 * prev_w_hat) + (1.0000 * prev_d_hat)) + (0.0000 * (u + prev_u)); 

  // Saturation
  if(u > 4.8)
    u = 4.8;
  else if(u < 0) // 반대방향으로 굴러가는 수레로 수정필요
    u = 0;

  pwm = u / 5;
  output = TOP * pwm;
  
  return output;
}