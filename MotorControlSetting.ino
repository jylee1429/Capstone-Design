#include "MotorControlSetting.h"

#define REVERSE_DIR

uint8_t state = false;
uint8_t current_flag = false;

float pwm_percentage_right = 0;
float pwm_percentage_left = 0;

const uint32_t _F_CPU = F_CPU / 2;
extern uint8_t vel_flag0;
extern uint8_t vel_flag7;

void System_On(void){
  LoadCell_Setting();                                        // loadcell reset
  state = true; 
}

void System_Off(void){
  state = false;
  PWM_Right_Value_Set(0);
  PWM_Left_Value_Set(0);   
}

uint8_t Is_OnOff(void){
  return state;   
}

// Motor On/Off function
void Motor_OnOff_Switch(void){
  uint8_t sw_state = digitalRead(SW_PIN);
  if(sw_state == HIGH){                               // switch Off
    System_Off();                                     // if switch off, system off
    Serial.println("System Off");
  }
  else{                                               // switch On
    System_On();                                      // if switch on, system on
    Serial.println("System On");
  }
}


void GPIO_Setting(void){
  pinMode(SW_PIN, INPUT);                             // On/Off switch                   
  pinMode(DIR_RIGHT_PIN, OUTPUT);
  pinMode(DIR_LEFT_PIN, OUTPUT);
  #ifdef REVERSE_DIR
  digitalWrite(DIR_LEFT_PIN, LOW);
  #endif
  #ifdef FORWARD_DIR
  digitalWrite(DIR_LEFT_PIN, HIGH);
  #endif
  digitalWrite(DIR_RIGHT_PIN, HIGH);                  // forword dir

  // Debouncing filter
  REG_PIOD_IFER |= 1 << 5;
  REG_PIOD_DIFSR |= 1 << 5;
  REG_PIOD_SCDR |= 0xFF;                              // 16ms
}

float Get_Velocity_Right(uint32_t period_cnt){
  float frequency, vel; 
  if(vel_flag0 == true){
    vel_flag0 = false;
    frequency= _F_CPU / period_cnt;
    vel = frequency / POLE * 60;
  }
  else{
    vel = 0;
  }

  return vel;
}

float Get_Velocity_Left(uint32_t period_cnt){
  float frequency, vel; 
  if(vel_flag7 == true){
    vel_flag7 = false;
    frequency= _F_CPU / period_cnt;
    vel = frequency / POLE * 60;
  }
  else{
    vel = 0;
  }

  return vel;
}

void PWM_Right_Value_Set(float pwm_val){
  REG_PWM_CDTY2 = TOP * pwm_val;        
}

void PWM_Left_Value_Set(float pwm_val){
  REG_PWM_CDTY3 = TOP * pwm_val;    
}

void PWM_Setting(void){
  REG_PMC_PCER1 |= PMC_PCER1_PID36;                   // ID-36 PWM
  
  // PWM channel 2
  // PWML2(PA20) - 43 pin(right)
  REG_PWM_DIS = PWM_DIS_CHID2;                        // Disable PWM channel 2
  REG_PIOA_PDR = PIO_PDR_P20;                         // the pin is controlled by the corresponding on-chip peripheral selected in the PIO_ABSR
  REG_PIOA_ABSR = PIO_PA20B_PWML2;                    // Set PWM pin perhipheral type B
  REG_PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(100);  // Set PWM CLKA prescale = 1 
  REG_PWM_CMR2 = PWM_CMR_CPRE_CLKA;                   // CLock A, single slope
  REG_PWM_CPRD2 = TOP;                                // TOP value
  REG_PWM_CDTY2 = 0;                                  // OC value
  REG_PWM_ENA = PWM_ENA_CHID2;                        // channel2 PWM enable

  // PWM channel 3
  // PWMH3(PC9) - 41 pin(left)
  REG_PWM_DIS = PWM_DIS_CHID3;                        // Disable PWM channel 3
  REG_PIOC_PDR = PIO_PDR_P9;                          // the pin is controlled by the corresponding on-chip peripheral selected in the PIO_ABSR
  REG_PIOC_ABSR = PIO_PC9B_PWMH3;                     // Set PWM pin perhipheral type B
  REG_PWM_CMR3 = PWM_CMR_CPRE_CLKA                    // CLock B, single slope
               | PWM_CMR_CPOL;                        // OC output waveform starts at a high level
  REG_PWM_CPRD3 = TOP;                                // TOP value
  REG_PWM_CDTY3 = 0;                                  // OC value
  REG_PWM_ENA = PWM_ENA_CHID3;                        // channel2 PWM enable
}

void Interrupt_Setting(void){
  REG_PMC_PCER0 |= PMC_PCER0_PID27;                   // ID-27 TC0 channel(TC0)
  REG_PMC_PCER0 |= PMC_PCER0_PID30;                   // ID-30 TC1 channel(TC3)
  REG_PMC_PCER0 |= PMC_PCER0_PID31;                   // ID-31 TC1 channel(TC4)
  REG_PMC_PCER1 |= PMC_PCER1_PID32;                   // ID-32 TC1 channel(TC5)
  REG_PMC_PCER1 |= PMC_PCER1_PID34;                   // ID-34 TC2 channel(TC7)
  REG_PMC_PCER1 |= PMC_PCER1_PID35;                   // ID-35 TC2 channel(TC8)

  // OC interrupt - Ch0
  REG_TC0_CMR0 = TC_CMR_TCCLKS_TIMER_CLOCK1
               | TC_CMR_ABETRG                        // TIOA is used as the external trigger
               | TC_CMR_LDRA_RISING                   // load RA on rising edge of trigger input
               | TC_CMR_LDRB_FALLING;                 // load RB on falling edge of trigger input
  REG_TC0_IER0 |= TC_IER_LDRAS | TC_IER_LDRBS;
  
  // OC interrupt - Ch3
  REG_TC1_CMR0 = TC_CMR_TCCLKS_TIMER_CLOCK1           // Internal MCK/2 clock signal
               | TC_CMR_WAVE                          // waveform mode
               | TC_CMR_WAVSEL_UP_RC;                 // up count until count = RC     
  REG_TC1_RC0 = VAL_100MS;                            // set RC2 value
  REG_TC1_IER0 = TC_IER_CPCS;                         // RC compare interrupt enable

  // OC interrupt - Ch4
  REG_TC1_CMR1 = TC_CMR_TCCLKS_TIMER_CLOCK1           // Internal MCK/2 clock signal
               | TC_CMR_WAVE                          // waveform mode
               | TC_CMR_WAVSEL_UP_RC;                 // up count until count = RC     
  REG_TC1_RC1 = VAL_5MS;                              // set RC2 value
  REG_TC1_IER1 = TC_IER_CPCS;                         // RC compare interrupt enable
  
  // OC interrupt - Ch5
  REG_TC1_CMR2 = TC_CMR_TCCLKS_TIMER_CLOCK1           // Internal MCK/2 clock signal
               | TC_CMR_WAVE                          // waveform mode
               | TC_CMR_WAVSEL_UP_RC;                 // up count until count = RC     
  REG_TC1_RC2 = VAL_50MS;                             // set RC2 value
  REG_TC1_IER2 = TC_IER_CPCS;                         // RC compare interrupt enable

  // OC interrupt - Ch7
  REG_TC2_CMR1 = TC_CMR_TCCLKS_TIMER_CLOCK1
               | TC_CMR_ABETRG                        // TIOA is used as the external trigger
               | TC_CMR_LDRA_RISING                   // load RA on rising edge of trigger input
               | TC_CMR_LDRB_FALLING;                 // load RB on falling edge of trigger input
  REG_TC2_IER1 |= TC_IER_LDRAS | TC_IER_LDRBS;

  // OC interrupt - Ch8
  REG_TC2_CMR2 = TC_CMR_TCCLKS_TIMER_CLOCK1           // Internal MCK/2 clock signal
               | TC_CMR_WAVE                          // waveform mode
               | TC_CMR_WAVSEL_UP_RC;                 // up count until count = RC     
  REG_TC2_RC2 = VAL_100MS;                            // set RC2 value
  REG_TC2_IER2 = TC_IER_CPCS;                         // RC compare interrupt enable
  
  NVIC_EnableIRQ(TC0_IRQn);                           // velocity 
  NVIC_EnableIRQ(TC3_IRQn);                           // loadcell
  NVIC_EnableIRQ(TC4_IRQn);                           // controller
  NVIC_EnableIRQ(TC5_IRQn);                           // print
  NVIC_EnableIRQ(TC7_IRQn);                           // velocity
  NVIC_EnableIRQ(TC8_IRQn);                           // ultrasonic sensor


  REG_TC0_CCR0 = TC_CCR_CLKEN | TC_CCR_SWTRG;         // TC0
  REG_TC1_CCR0 = TC_CCR_CLKEN | TC_CCR_SWTRG;         // TC3
  REG_TC1_CCR1 = TC_CCR_CLKEN | TC_CCR_SWTRG;         // TC4
  REG_TC1_CCR2 = TC_CCR_CLKEN | TC_CCR_SWTRG;         // TC5
  REG_TC2_CCR1 = TC_CCR_CLKEN | TC_CCR_SWTRG;         // TC7
  REG_TC2_CCR2 = TC_CCR_CLKEN | TC_CCR_SWTRG;         // TC8
  
  // Externel Interrut(PB25 - pin2)
  // On/Off switch
  attachInterrupt(SW_PIN, Motor_OnOff_Switch, CHANGE);
}