//모터 전류제어

/*
#define Kt = 0.03
#define Kb = 0.03
#define R = 1.2
#define L = 1e-3
#define Bm = 1e-4
#define Jm = 1e-5
*/

// float i = 0;
// float i_hat = 0;
// float w_hat = 0;
// float d_hat = 0;
// float prev_i_hat = 0;
// float prev_w_hat = 0;
// float prev_d_hat = 0;
// float u = 0;   
// float prev_u = 0;
// float err = 0;  
// float err_hat = 0;
// float pwm = 0;
// float force = 0;

// //PI controller
// float Kp = 10;
// float Ki = 850; //((Ra*Bm+Kt*Kb)/(Ra*Jm))*Kp;

// void Reset_X(void){
//   prev_i = i;
//   prev_i_hat = i_hat;
//   prev_w_hat = w_hat;
//   prev_d_hat = d_hat;
// }


// void TC8_Handler() {            // Interrupt occurs every 0.1ms      
//   REG_TC2_SR2;                  // flag clear                        
//   // ref = Set_Input();
//   Reset_X();
//   force = Get_force(); // loadcell input
//   prev_err = err;
//   err = force - i;
//   prev_d_hat = d_hat;
//   d_hat = i - i_hat;      // err_hat = current - current_hat;
//   prev_u = u;

//   // PI controller 
//   u = prev_u + ((Kp + (Ki * timebase / 2)) * err) + ((-Kp + (Ki * timebase / 2)) * prev_err) - ((L2 * timebase / 2) * (prev_err_hat + err_hat));
//   // u = prev_u + ((Kp + (Ki * timebase / 2)) * err) + ((-Kp + (Ki * timebase / 2)) * prev_err);
  
//   // OBS
//   i_hat = (( -0.1183 * prev_i) + ( 0.9980 * prev_i_hat) + (-0.0030 * prev_w_hat) + (0.0999 * prev_d_hat)) + (0.0500 * (u + prev_u));
//   w_hat = (( 0.2999 * prev_i) + ( 0.0000 * prev_i_hat) + ( 0.9990 * prev_w_hat) + (0.0000 * prev_d_hat)) + (0.0000 * (u + prev_u)); 
//   d_hat = (( 0.00001 * prev_i) + ( -0.00001 * prev_i_hat) + ( 0.0000 * prev_w_hat) + (0.99999 * prev_d_hat)) + (-0.0000003 * (u + prev_u)); 

//   // Saturation
//   if(u > 4.8)
//     u = 4.8;
//   else if(u < 0) // 반대방향으로 굴러가는 수레로 수정필요
//     u = 0;

//   pwm = u / 5;
//   REG_PWM_CDTY2 = TOP * pwm;
// } 
