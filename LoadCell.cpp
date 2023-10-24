#include <Arduino.h>
#include "LoadCell.h"


#if ARDUINO_VERSION <= 106
    void yield(void) {};
#endif

void Init(LoadCell* loadcell, uint8_t data_pin, uint8_t clk_pin) {
  loadcell->clk = clk_pin;
  loadcell->data = data_pin;
  
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

int32_t Get_Value(LoadCell* loadcell, uint8_t times) {
	return Read_Average(loadcell, times) - (loadcell->offset);
}

int32_t Get_Units(LoadCell* loadcell, uint8_t times) {
	return Get_Value(loadcell, times) / (loadcell->scale);
}

void Tare(LoadCell* loadcell, uint8_t times) {
	double sum = Read_Average(loadcell, times);
	Set_Offset(loadcell, sum);
}

void Set_Scale(LoadCell* loadcell, float scale) {
	loadcell->scale = scale;
}

float Get_Scale(LoadCell* loadcell) {
	return loadcell->scale;
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
