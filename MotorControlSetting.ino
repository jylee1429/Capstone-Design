#include "MotorControlSetting.h"
#include "Arduino.h"
#include "MotorControlSetting.h"
#include "Arduino.h"

const uint32_t freq_cpu = F_CPU / 2;
volatile uint32_t prev_cnt = 0;
float scale_factor = 4782;
int32_t input_force = 0;
float frequency = 0;
uint8_t state = FALSE;

LoadCell loadcell;
FND fnd;

void System_On(void){
  state = TRUE; 
  digitalWrite(LED_PIN, HIGH);            // led on
}

void System_Off(void){
  state = FALSE;
  digitalWrite(LED_PIN, LOW);             // led off
  PWM_VALUE_RIGHT = 0;
  PWM_VALUE_LEFT = 0;
}

// Motor On/Off function
void Motor_OnOff_Switch(void){
  uint8_t sw_state = digitalRead(SW_PIN);
  if(sw_state == HIGH){                   // switch Off
    System_Off();                         // if switch off, system off
  }
  else{                                   // switch On
    System_On();                          // if switch on, system on
  }
}

void FND_Setting(void){
  SPI_Init();
  FND_Init(&fnd, RCLK_PIN);
}

void Display_FND(int val){
  digitalWrite(SS, LOW);
  Digit4_Zero(&fnd, val, 10);
}

void LoadCell_Setting(void){
  Init(&loadcell, DOUT_PIN, CLK_PIN);
  Tare(&loadcell, TIMES);
  Set_Scale(&loadcell, scale_factor);
}

int32_t Get_Force(void){
  return Get_Units(&loadcell, 1) * 0.453592;      // [kg]
}

float Get_Frequency(void){
  uint32_t period_cnt = 0;
  uint32_t cnt;
  cnt = (uint32_t)REG_TC0_RA0;
  period_cnt = cnt - prev_cnt;
  prev_cnt = cnt;

  frequency = freq_cpu / period_cnt;
  return frequency;
} 

uint16_t Get_Velocity(float frequence){
  uint16_t vel;
  vel = frequence / POLE * 60;
  return vel;
}

void GPIO_Setting(void){
  pinMode(SW_PIN, INPUT);                       // On/Off switch
  pinMode(LED_PIN, OUTPUT);                     // test LED 
  pinMode(TRIGGER_PIN, OUTPUT);                 // ultrasonic sensor-trigger pin
  pinMode(ECHO_PIN, INPUT);                     // ultrasonic sensor-echo pin         
  pinMode(FG_PIN, INPUT);                   
  
  // Debouncing filter
  REG_PIOB_IFER |= 1 << 25;
  REG_PIOB_DIFSR |= 1 << 25;
  REG_PIOB_SCDR |= 0xFF;                        // 16 ms
}

void PWM_Setting(void){
  REG_PMC_PCER1 |= PMC_PCER1_PID36;                   // ID-36 PWM
  // PWM channel 2
  // PWML2(PA20)
  REG_PWM_DIS = PWM_DIS_CHID2;                        // Disable PWM channel 2
  REG_PIOA_PDR = PIO_PDR_P20;                         // the pin is controlled by the corresponding on-chip peripheral selected in the PIO_ABSR
  REG_PIOA_ABSR = PIO_PA20B_PWML2;                    // Set PWM pin perhipheral type B
  REG_PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(1);    // Set PWM CLKA prescale = 1 
  REG_PWM_CMR2 = PWM_CMR_CPRE_CLKA;                   // CLock A, single slope
  REG_PWM_CPRD2 = TOP;                                // TOP value
  REG_PWM_CDTY2 = PWM_VALUE_RIGHT;                    // OC value
  REG_PWM_ENA = PWM_ENA_CHID2;                        // channel2 PWM enable
  // PWM channel 3
  // PWML3(PC8)
  REG_PWM_DIS = PWM_DIS_CHID3;                        // Disable PWM channel 3
  REG_PIOC_PDR = PIO_PDR_P8;                          // the pin is controlled by the corresponding on-chip peripheral selected in the PIO_ABSR
  REG_PIOC_ABSR = PIO_PC8B_PWML3;                     // Set PWM pin perhipheral type B
  REG_PWM_CMR3 = PWM_CMR_CPRE_CLKA;                   // CLock A, single slope
  REG_PWM_CPRD3 = TOP;                                // TOP value
  REG_PWM_CDTY3 = PWM_VALUE_LEFT;                     // OC value
  REG_PWM_ENA = PWM_ENA_CHID3;                        // channel2 PWM enable
}

void Interrupt_Setting(void){
  // PMC Peripheral Clock Enable Register0
  REG_PMC_PCER0 |= PMC_PCER0_PID27;               // ID-27 TC1 channel(TC0)
  REG_PMC_PCER0 |= PMC_PCER0_PID31;               // ID-31 TC1 channel(TC4)
  REG_PMC_PCER1 |= PMC_PCER1_PID32;               // ID-32 TC1 channel(TC5)
  REG_PMC_PCER1 |= PMC_PCER1_PID34;               // ID-34 TC2 channel(TC7)
  
  // External interrupt - Ch0
  REG_TC0_CMR0 = TC_CMR_TCCLKS_TIMER_CLOCK1
               | TC_CMR_ABETRG                    // TIOA is used as the external trigger
               | TC_CMR_LDRA_RISING;              // load RA on rising edge of trigger input
  REG_TC0_IER0 = TC_IER_LDRAS;                    // enable the RA Load interrupt

  // OC interrupt - Ch4
  REG_TC1_CMR1 = TC_CMR_TCCLKS_TIMER_CLOCK1       // Internal MCK/2 clock signal
               | TC_CMR_WAVE                      // waveform mode
               | TC_CMR_ACPC_TOGGLE               // toggle TIOA output
               | TC_CMR_WAVSEL_UP_RC;             // up count until count = RC     
  REG_TC1_RC1 = VAL_01MS;                         // set RC2 value
  REG_TC1_IER1 = TC_IER_CPCS;                     // RC compare interrupt enable
  
  // OC interrupt - Ch5
  REG_TC1_CMR2 = TC_CMR_TCCLKS_TIMER_CLOCK1       // Internal MCK/2 clock signal
               | TC_CMR_WAVE                      // waveform mode
               | TC_CMR_ACPC_TOGGLE               // toggle TIOA output
               | TC_CMR_WAVSEL_UP_RC;             // up count until count = RC     
  REG_TC1_RC2 = VAL_10MS;                         // set RC2 value
  REG_TC1_IER2 = TC_IER_CPCS;                     // RC compare interrupt enable
  
  // OC interrupt - Ch7
  REG_TC2_CMR1 = TC_CMR_TCCLKS_TIMER_CLOCK1       // Internal MCK/2 clock signal
               | TC_CMR_WAVE                      // waveform mode
               | TC_CMR_ACPC_TOGGLE               // toggle TIOA output
               | TC_CMR_WAVSEL_UP_RC;             // up count until count = RC     
  REG_TC2_RC1 = VAL_2MS;                          // set RC2 value
  REG_TC2_IER1 = TC_IER_CPCS;                     // RC compare interrupt enable

  // OC interrupt - Ch8
  REG_TC2_CMR2 = TC_CMR_TCCLKS_TIMER_CLOCK1       // Internal MCK/2 clock signal
               | TC_CMR_WAVE                      // waveform mode
               | TC_CMR_ACPC_TOGGLE               // toggle TIOA output
               | TC_CMR_WAVSEL_UP_RC;             // up count until count = RC     
  REG_TC2_RC1 = VAL_10MS;                          // set RC2 value
  REG_TC2_IER1 = TC_IER_CPCS;                     // RC compare interrupt enable
  
  NVIC_EnableIRQ(TC0_IRQn);
  NVIC_EnableIRQ(TC4_IRQn); 
  NVIC_EnableIRQ(TC5_IRQn); 
  NVIC_EnableIRQ(TC7_IRQn);
  NVIC_EnableIRQ(TC8_IRQn);  

  // setting interrupt priority
  NVIC_SetPriority(TC4_IRQn, 0);                   // controller
  NVIC_SetPriority(TC5_IRQn, 1);                   // FND
  NVIC_SetPriority(TC7_IRQn, 0);                   // current measurement
  NVIC_SetPriority(TC8_IRQn, 2);

  REG_TC0_CCR0 = TC_CCR_CLKEN | TC_CCR_SWTRG;      // TC0
  REG_TC1_CCR1 = TC_CCR_CLKEN | TC_CCR_SWTRG;      // TC4
  REG_TC1_CCR2 = TC_CCR_CLKEN | TC_CCR_SWTRG;      // TC5
  REG_TC2_CCR1 = TC_CCR_CLKEN | TC_CCR_SWTRG;      // TC7
  REG_TC2_CCR2 = TC_CCR_CLKEN | TC_CCR_SWTRG;      // TC8
  
  // Externel Interrut(PB25 - pin2)
  attachInterrupt(SW_PIN, Motor_OnOff_Switch, CHANGE);
}