#ifndef _LOAD_CELL_H_
#define _LOAD_CELL_H_

#define DOUT_PIN 6
#define CLK_PIN 5

typedef struct _loadcell{
  uint8_t clk;
	uint8_t data;		
	uint8_t gain;	
	long offset = 0;	
	float scale = 1;	
}LoadCell;

void Init(LoadCell* loadcell, uint8_t data_pin, uint8_t clk_pin);
uint8_t Check_Ready(LoadCell* loadcell);
long Read(LoadCell* loadcell);
int32_t Read_Average(LoadCell* loadcell, uint8_t times) ;
int32_t Get_Value(LoadCell* loadcell, uint8_t times);
int32_t Get_Units(LoadCell* loadcell, uint8_t times);
int32_t LPF_Force(LoadCell* loadcell, int32_t prev_estimated_value, float theta);
void Tare(LoadCell* loadcell, uint8_t times);
void Set_Scale(LoadCell* loadcell, float scale);
float Get_Scale(LoadCell* loadcell);
void Set_Offset(LoadCell* loadcell, double offset);
int32_t Get_Offset(LoadCell* loadcell);
void Power_Down(LoadCell* loadcell);
void Power_Up(LoadCell* loadcell);
#endif