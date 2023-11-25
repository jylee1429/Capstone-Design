#include <Arduino.h>
#include "LoadCell.h"

#if ARDUINO_VERSION <= 106
    void yield(void) {};
#endif

LoadCell loadcell_right;
LoadCell loadcell_left;

float calibration_factor_right = 126430;
float calibration_factor_left = 123150;

void Init(LoadCell* loadcell, uint8_t data_pin, uint8_t clk_pin) {
  loadcell->clk = clk_pin;
  loadcell->data = data_pin;
  loadcell->calibration = 1;
  loadcell->offset = 0;

  pinMode(loadcell->clk, OUTPUT);
	pinMode(loadcell->data, INPUT);

  loadcell->gain = 1;
  digitalWrite(loadcell->clk, LOW);

  Read(loadcell);
}

uint8_t Check_Ready(LoadCell* loadcell) {
	return digitalRead(loadcell->data) == LOW; 
}

long Read(LoadCell* loadcell) {
	while (!Check_Ready(loadcell)) {
    yield();
	}

	uint32_t value = 0;
	uint8_t data[3] = { 0 };
	uint8_t signbit = 0x00;
  uint8_t i;

	data[2] = shiftIn(loadcell->data, loadcell->clk, MSBFIRST);
	data[1] = shiftIn(loadcell->data, loadcell->clk, MSBFIRST);
	data[0] = shiftIn(loadcell->data, loadcell->clk, MSBFIRST);
  
	for (i = 0; i < loadcell->gain; i++) {
		digitalWrite(loadcell->clk, HIGH);
		digitalWrite(loadcell->clk, LOW);
	}

	if (data[2] & 0x80) {
		signbit = 0xFF;
	} else {
		signbit = 0x00;
	}

	value = ( (uint32_t)(signbit) << 24
			| (uint32_t)(data[2]) << 16
			| (uint32_t)(data[1]) << 8
			| (uint32_t)(data[0]) );
	return (int32_t)(value);
}

int32_t Read_Average(LoadCell* loadcell, uint8_t times) {
	int32_t sum = 0;
	int8_t i;
  for (i = 0; i < times; i++) {
		sum += Read(loadcell);
	}
	return sum / times;
}

double Get_Value(LoadCell* loadcell, uint8_t times) {
	return Read_Average(loadcell, times) - (loadcell->offset);
}

float Get_Units(LoadCell* loadcell, uint8_t times) {
	return Get_Value(loadcell, times) / (loadcell->calibration);
}

void Tare(LoadCell* loadcell, uint8_t times) {
	double sum = Read_Average(loadcell, times);
	Set_Offset(loadcell, sum);
}

void Set_Calibration(LoadCell* loadcell, float calibration) {
	loadcell->calibration = calibration;
}

float Get_Calibration(LoadCell* loadcell) {
	return loadcell->calibration;
}

void Set_Offset(LoadCell* loadcell, double offset) {
	loadcell->offset = offset;
}

int32_t Get_Offset(LoadCell* loadcell) {
	return loadcell->offset;
}

void Power_Down(LoadCell* loadcell) {
	digitalWrite(loadcell->clk, LOW);
	digitalWrite(loadcell->clk, HIGH);
}

void Power_Up(LoadCell* loadcell) {
	digitalWrite(loadcell->clk, LOW);
}

void LoadCell_Setting(void){
  Init(&loadcell_right, DOUT_PIN_RIGHT, CLK_PIN_RIGHT);
  Init(&loadcell_left, DOUT_PIN_LEFT, CLK_PIN_LEFT);
  Tare(&loadcell_right, TIMES);
  Tare(&loadcell_left, TIMES);
  Set_Calibration(&loadcell_right, calibration_factor_right);
  Set_Calibration(&loadcell_left, calibration_factor_left);
}

int32_t Get_Force_Right(void){
  int32_t force;
  force = Get_Units(&loadcell_right, 1);      // [kg]
  if(force < DEADBAND)
    force = 0;
  return force;
}

int32_t Get_Force_Left(void){
  int32_t force;
  force = Get_Units(&loadcell_right, 1);      // [kg]
  if(force < DEADBAND)
    force = 0;
  return force;
}