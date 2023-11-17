#include "MotorControlSetting.h"
#include "Arduino.h"

uint8_t state = false;

void System_On(void){
  state = true; 
  digitalWrite(ON_OFF_LED, HIGH);             // if switch on, led on
}

void System_Off(void){
  state = false;
  PWM_VALUE_RIGHT = 0;
  PWM_VALUE_LEFT = 0;
  digitalWrite(ON_OFF_LED, LOW);              // if switch off, led off
}

uint8_t Is_OnOff(void){
  return state;
}

// Motor On/Off function
void Motor_OnOff_Switch(void){
  uint8_t sw_state = digitalRead(SW_PIN);
  if(sw_state == HIGH){                         // switch Off
    System_Off();                               // if switch off, system off
  }
  else{                                         // switch On
    System_On();                                // if switch on, system on
  }
}

uint32_t ABS(int32_t x){
  if(x < 0){
    return x * (-1);
  }
  else
    return x;
}

void LED_TEST(void){
  Serial.println("LED1");
  digitalWrite(TEST_LED_1, HIGH);
  delay(100);
  digitalWrite(TEST_LED_1, LOW);
  delay(100);
  Serial.println("LED2");  
  digitalWrite(TEST_LED_2, HIGH);
  delay(100);
  digitalWrite(TEST_LED_2, LOW);
  delay(100);

  Serial.println("LED3");
  digitalWrite(TEST_LED_3, HIGH);
  delay(100);
  digitalWrite(TEST_LED_3, LOW);  
  delay(100);

  Serial.println("LED4");  
  digitalWrite(TEST_LED_4, HIGH);
  delay(100);
  digitalWrite(TEST_LED_4, LOW);
  delay(100);
}

void GPIO_Setting(void){
  pinMode(SW_PIN, INPUT);                       // On/Off switch
  pinMode(TRIGGER_PIN, OUTPUT);                 // ultrasonic sensor-trigger pin
  pinMode(ECHO_PIN, INPUT);                     // ultrasonic sensor-echo pin                        
  pinMode(CUR_RIGHT_U, INPUT);                  // right hall sensor u
  pinMode(CUR_RIGHT_V, INPUT);                  // right hall sensor v
  pinMode(CUR_RIGHT_W, INPUT);                  // right hall sensor w
  pinMode(CUR_LEFT_U, INPUT);                   // left hall sensor u
  pinMode(CUR_LEFT_V, INPUT);                   // left hall sensor u
  pinMode(CUR_LEFT_W, INPUT);                   // left hall sensor u
  pinMode(TEST_LED_1, OUTPUT);                  // test led
  pinMode(TEST_LED_2, OUTPUT);                  // test led
  pinMode(TEST_LED_3, OUTPUT);                  // test led
  pinMode(TEST_LED_4, OUTPUT);                  // test led
  pinMode(ON_OFF_LED, OUTPUT);                  // on-off led
  
  // Debouncing filter
  REG_PIOB_IFER |= 1 << 25;
  REG_PIOB_DIFSR |= 1 << 25;
  REG_PIOB_SCDR |= 0xFF;                        // 16 ms
}

void PWM_Setting(void){
  REG_PMC_PCER1 |= PMC_PCER1_PID36;                   // ID-36 PWM
  
  REG_PWM_CLK = PWM_CLK_PREA(0) | PWM_CLK_DIVA(1);    // Set PWM CLKA prescale = 1 
  // PWM channel 2
  // PWML2(PA20)
  REG_PWM_DIS = PWM_DIS_CHID2;                        // Disable PWM channel 2
  REG_PIOA_PDR = PIO_PDR_P20;                         // the pin is controlled by the corresponding on-chip peripheral selected in the PIO_ABSR
  REG_PIOA_ABSR = PIO_PA20B_PWML2;                    // Set PWM pin perhipheral type B
  REG_PWM_CMR2 = PWM_CMR_CPRE_CLKA;                   // CLock A, single slope
  REG_PWM_CPRD2 = TOP;                                // TOP value(8400)
  REG_PWM_CDTY2 = TOP * PWM_VALUE_RIGHT;              // OC value
  REG_PWM_ENA = PWM_ENA_CHID2;                        // channel2 PWM enable
  // PWM channel 3
  // PWML3(PC8)
  REG_PWM_DIS = PWM_DIS_CHID3;                        // Disable PWM channel 3
  REG_PIOC_PDR = PIO_PDR_P8;                          // the pin is controlled by the corresponding on-chip peripheral selected in the PIO_ABSR
  REG_PIOC_ABSR = PIO_PC8B_PWML3;                     // Set PWM pin perhipheral type B
  REG_PWM_CMR3 = PWM_CMR_CPRE_CLKA;                   // CLock A, single slope
  REG_PWM_CPRD3 = TOP;                                // TOP value
  REG_PWM_CDTY3 = TOP * PWM_VALUE_LEFT;               // OC value
  REG_PWM_ENA = PWM_ENA_CHID3;                        // channel2 PWM enable
}

void Interrupt_Setting(void){
  // PMC Peripheral Clock Enable Register0
  REG_PMC_PCER0 |= PMC_PCER0_PID27;               // ID-27 TC0 channel(TC0)
  REG_PMC_PCER0 |= PMC_PCER0_PID30;               // ID-30 TC1 channel(TC3)
  REG_PMC_PCER0 |= PMC_PCER0_PID31;               // ID-31 TC1 channel(TC4)
  REG_PMC_PCER1 |= PMC_PCER1_PID32;               // ID-32 TC1 channel(TC5)
  REG_PMC_PCER1 |= PMC_PCER1_PID35;               // ID-35 TC2 channel(TC8)

  // OC interrupt - Ch0
  REG_TC0_CMR0 = TC_CMR_TCCLKS_TIMER_CLOCK1       // Internal MCK/2 clock signal
               | TC_CMR_WAVE                      // waveform mode
               | TC_CMR_WAVSEL_UP_RC;             // up count until count = RC     
  REG_TC0_RC0 = VAL_40MS;                         // set RC2 value
  REG_TC0_IER0 = TC_IER_CPCS;                     // RC compare interrupt enable

  // OC interrupt - Ch3
  REG_TC1_CMR0 = TC_CMR_TCCLKS_TIMER_CLOCK1       // Internal MCK/2 clock signal
               | TC_CMR_WAVE                      // waveform mode
               | TC_CMR_WAVSEL_UP_RC;             // up count until count = RC     
  REG_TC1_RC0 = VAL_100MS;                        // set RC2 value
  REG_TC1_IER0 = TC_IER_CPCS;                     // RC compare interrupt enable

  // OC interrupt - Ch4
  REG_TC1_CMR1 = TC_CMR_TCCLKS_TIMER_CLOCK1       // Internal MCK/2 clock signal
               | TC_CMR_WAVE                      // waveform mode
               | TC_CMR_WAVSEL_UP_RC;             // up count until count = RC     
  REG_TC1_RC1 = VAL_5MS;                          // set RC2 value
  REG_TC1_IER1 = TC_IER_CPCS;                     // RC compare interrupt enable
  
  // OC interrupt - Ch5
  REG_TC1_CMR2 = TC_CMR_TCCLKS_TIMER_CLOCK1       // Internal MCK/2 clock signal
               | TC_CMR_WAVE                      // waveform mode
               | TC_CMR_WAVSEL_UP_RC;             // up count until count = RC     
  REG_TC1_RC2 = VAL_20MS;                         // set RC2 value
  REG_TC1_IER2 = TC_IER_CPCS;                     // RC compare interrupt enable

  // OC interrupt - Ch8
  REG_TC2_CMR2 = TC_CMR_TCCLKS_TIMER_CLOCK1       // Internal MCK/2 clock signal
               | TC_CMR_WAVE                      // waveform mode
               | TC_CMR_WAVSEL_UP_RC;             // up count until count = RC     
  REG_TC2_RC2 = VAL_100MS;                        // set RC2 value
  REG_TC2_IER2 = TC_IER_CPCS;                     // RC compare interrupt enable
  
  NVIC_EnableIRQ(TC0_IRQn);                       // velocity 
  NVIC_EnableIRQ(TC3_IRQn);                       // loadcell
  NVIC_EnableIRQ(TC4_IRQn);                       // controller
  NVIC_EnableIRQ(TC5_IRQn);                       // FND
  NVIC_EnableIRQ(TC8_IRQn);                       // ultrasonic sensor

  // // setting interrupt priority
  // NVIC_SetPriority(TC0_IRQn, 2);                   // velocity
  // NVIC_SetPriority(TC3_IRQn, 1);                   // loadcell
  // NVIC_SetPriority(TC4_IRQn, 3);                   // controller
  // NVIC_SetPriority(TC5_IRQn, 2);                   // FND
  // NVIC_SetPriority(TC8_IRQn, 2);                   // ultrasonic sensor

  REG_TC0_CCR0 = TC_CCR_CLKEN | TC_CCR_SWTRG;      // TC0
  REG_TC1_CCR0 = TC_CCR_CLKEN | TC_CCR_SWTRG;      // TC3
  REG_TC1_CCR1 = TC_CCR_CLKEN | TC_CCR_SWTRG;      // TC4
  REG_TC1_CCR2 = TC_CCR_CLKEN | TC_CCR_SWTRG;      // TC5
  REG_TC2_CCR2 = TC_CCR_CLKEN | TC_CCR_SWTRG;      // TC8
  
  // Externel Interrut(PB25 - pin2)
  attachInterrupt(SW_PIN, Motor_OnOff_Switch, CHANGE);
  // attachInterrupt(CUR_RIGHT_U, Get_Current, CHANGE);
  // attachInterrupt(CUR_RIGHT_V, Get_Current, CHANGE);
  // attachInterrupt(CUR_RIGHT_W, Get_Current, CHANGE);
  // attachInterrupt(CUR_RIGHT_U, Get_Current, CHANGE);
  // attachInterrupt(CUR_RIGHT_V, Get_Current, CHANGE);
  // attachInterrupt(CUR_RIGHT_W, Get_Current, CHANGE);
}