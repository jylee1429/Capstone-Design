#include "MotorControlSetting.h"
#include "Arduino.h"

extern float input_force;
extern uint8_t state;
float scale_factor = 4782;

LoadCell loadcell;
FND fnd;

void System_On(int val){
  state = TRUE; 
  PWM_VALUE_RIGHT = val;
  PWM_VALUE_LEFT = val;
}

void System_Off(void){
  state = FALSE;
  PWM_VALUE_RIGHT = 0;
  PWM_VALUE_LEFT = 0;
}

// Motor On/Off function
void Motor_OnOff_Switch(void){
  uint8_t sw_state = digitalRead(SW_PIN);
  if(sw_state == HIGH){                     // switch Off
    delayMicroseconds(10000);               // delay 10ms
    if(sw_state == HIGH){
      System_Off();                         // if switch off, system off
      digitalWrite(LED_PIN, LOW);           // led off
    }
  }
  else{                                     // switch On
    delayMicroseconds(10000);               // delay 10ms
    if(sw_state == LOW){
      System_On(0);                         // if switch on, system on
      digitalWrite(LED_PIN, HIGH);          // led on
    }
  }
}
void FND_Setting(void){
  SPI_Init();
  FND_Init(&fnd, RCLK_PIN);
  
  #ifdef SERIAL_PRINT
  Serial.println("Loadcell setting");
  #endif
}

void Display_FND(int val){
  digitalWrite(SS, LOW);
  Digit4_Zero(&fnd, val, 100);
}

void LoadCell_Setting(void){
  Init(&loadcell, DOUT_PIN, CLK_PIN);
  Tare(&loadcell, TIMES);
  Set_Scale(&loadcell, scale_factor);

  #ifdef SERIAL_PRINT
  Serial.println("Loadcell setting");
  #endif
}

float Get_Force(void){
  return Get_Units(&loadcell, 1) * 0.453592; // [kg]
}

void GPIO_Setting(void){
  pinMode(SW_PIN, INPUT);                       // On/Off switch
  pinMode(LED_PIN, OUTPUT);                     // test LED 
  pinMode(TRIGGER_PIN, OUTPUT);                 // ultrasonic sensor-trigger pin
  pinMode(ECHO_PIN, INPUT);                     // ultrasonic sensor-echo pin         
  #ifdef SERIAL_PRINT
  Serial.println("GPIO setting");
  #endif
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
  #ifdef SERIAL_PRINT
  Serial.println("PWM setting");
  #endif
}

void Interrupt_Setting(void){
  // PMC Peripheral Clock Enable Register0
  REG_PMC_PCER1 |= PMC_PCER1_PID33;               // ID-33 TC2 channel(TC6)
  REG_PMC_PCER1 |= PMC_PCER1_PID34;               // ID-34 TC2 channel(TC7)
  REG_PMC_PCER1 |= PMC_PCER1_PID35;               // ID-35 TC2 channel(TC8)

  // OC interrupt(100ms) - Ch6
  REG_TC2_CMR0 = TC_CMR_TCCLKS_TIMER_CLOCK1       // Internal MCK/2 clock signal
               | TC_CMR_WAVE                      // waveform mode
               | TC_CMR_ACPC_TOGGLE               // toggle TIOA output
               | TC_CMR_WAVSEL_UP_RC;             // up count until count = RC     
  REG_TC2_RC0 = VAL_100MS;                        // set RC2 value
  REG_TC2_IER0 = TC_IER_CPCS;                     // RC compare interrupt enable
  
  // OC interrupt(10ms) - Ch7
  REG_TC2_CMR1 = TC_CMR_TCCLKS_TIMER_CLOCK1       // Internal MCK/2 clock signal
               | TC_CMR_WAVE                      // waveform mode
               | TC_CMR_ACPC_TOGGLE               // toggle TIOA output
               | TC_CMR_WAVSEL_UP_RC;             // up count until count = RC     
  REG_TC2_RC1 = VAL_10MS;                         // set RC2 value
  REG_TC2_IER1 = TC_IER_CPCS;                     // RC compare interrupt enable

  // OC interrupt(500ms) - Ch8
  REG_TC2_CMR2 = TC_CMR_TCCLKS_TIMER_CLOCK1       // Internal MCK/2 clock signal
               | TC_CMR_WAVE                      // waveform mode
               | TC_CMR_ACPC_TOGGLE               // toggle TIOA output
               | TC_CMR_WAVSEL_UP_RC;             // up count until count = RC     
  REG_TC2_RC2 = VAL_500MS;                        // set RC2 value
  REG_TC2_IER2 = TC_IER_CPCS;                     // RC compare interrupt enable
  
  NVIC_EnableIRQ(TC6_IRQn); 
  NVIC_EnableIRQ(TC7_IRQn); 
  NVIC_EnableIRQ(TC8_IRQn);

  REG_TC2_CCR0 = TC_CCR_CLKEN | TC_CCR_SWTRG;
  REG_TC2_CCR1 = TC_CCR_CLKEN | TC_CCR_SWTRG;
  REG_TC2_CCR2 = TC_CCR_CLKEN | TC_CCR_SWTRG;

  // Externel Interrut(PB25 - pin2)
  attachInterrupt(SW_PIN, Motor_OnOff_Switch, CHANGE);
  #ifdef SERIAL_PRINT
  Serial.println("Ineterrupt setting");
  #endif
}