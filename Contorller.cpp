#include "Controller.h"

//모터 전류제어

/* BLDC 인휠 모터
#define Kt = 0.03 추정값
#define Kb = 0.03 추정값
#define R = 1.2
#define L = 1e-3
#define Bm = 1e-4 추정값
#define Jm = 1e-5 추정값
*/

float i = 0;
float prev_i=0;
float w = 0;
float prev_w = 0;
float u = 0;
float prev_u = 0;
float err = 0;
float prev_err = 0;
float pwm = 0;
float force = 0;
float timebase = 0.005;

/*
A=[-Ra/La -Kb/La 1/La; 
    Kt/Jm -Bm/Jm 0; 
    0 0 0];
B=[1/La; 0; 0];
C=[1 0 0]; % 관측기 이득을 구하기 위한 행렬
L=acker(A',C',[-1 -1 -1]*10)';

A=[-Ra/La -Kb/La 0 0 0; 
    Kt/Jm -Bm/Jm 0 0 0; 
    L(1) 0 -Ra/La-L(1) -Kb/La 1/La; 
    L(2) 0 Kt/Jm-L(2) -Bm/Jm 0; 
    L(3) 0 -L(3) 0 0];
B=[1/La; 0; 1/La; 0; 0];
C=[1 0 0 0 0];

Kp = La * 100 ;
Ki = Ra * 100 ;

//이산시간 행렬
A_size = eye(size(A)); % A 행렬의 크기에 따라 변경
Ad = (A_size+A*T/2)*inv(A_size-A*T/2);
Bd = T/2*inv(A_size-A*T/2)*B;
*/

float A[3][3] = {
   {-0.3785,   -0.0284,    0.9711},
    {4.5476 ,   0.7432,    7.1056},
     {    0  ,       0,    1.0000}
} ;
float B[3][1] = {   
      {0.4856},
    {3.5528},
    {0}
   };

// float C[5][1] = { { 1 },
//                   { 0 },
//                   { 0 },
//                   {0},
//                   {0} };

//PI controller
float Kp = 0.016;
float Ki = 12; 

void Reset_X(void) {
  prev_i = i;
  prev_w = w;
}

float Controller(float input, float vel){  
  // ref = Set_Input();

  // force = Get_force();  // loadcell input
  force = input;
  prev_err = err;
  // err = force - i; // 전류 측정이 가능할 때
  err = force - i; // 전류 추정으로 제어
  prev_u = u;

  // PI controller
  u = prev_u + ((Kp + (Ki * timebase / 2)) * err) + ((-Kp + (Ki * timebase / 2)) * prev_err);
  // u = prev_u + ((Kp + (Ki * timebase / 2)) * err) + ((-Kp + (Ki * timebase / 2)) * prev_err);

  i = A[0][0]*prev_i + A[0][1]*prev_w + B[0][0]*(u+prev_u); 
  // w = A[1][0]*force + A[1][1]*w + A[1][2]*i_hat + A[1][3]*w_hat + A[1][4]*d_hat + B[1][0]*(u+prev_u); 
  
  w = vel;


  Reset_X(); // 값 초기화

  // Saturation
  if (u > 12)
    u = 12;
  else if (u < 0) 
    u = 0;

  pwm = u / 32;
  
  return pwm;
}