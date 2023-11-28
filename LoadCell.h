#ifndef _LOAD_CELL_H_
#define _LOAD_CELL_H_

#define DOUT_PIN_RIGHT    17
#define CLK_PIN_RIGHT     16
#define DOUT_PIN_LEFT     19
#define CLK_PIN_LEFT      18
#define TIMES             10
#define DEADBAND          0

typedef struct _loadcell{
  uint8_t clk;
	uint8_t data;		
	uint8_t gain;	
	long offset;	
	float calibration;	
}LoadCell;

void Init(LoadCell* loadcell, uint8_t data_pin, uint8_t clk_pin);
uint8_t Check_Ready(LoadCell* loadcell);
long Read(LoadCell* loadcell);
int32_t Read_Average(LoadCell* loadcell, uint8_t times) ;
int32_t Get_Value(LoadCell* loadcell);
int32_t Get_Units(LoadCell* loadcell);
int32_t LPF_Force(LoadCell* loadcell, int32_t prev_estimated_value, float theta);
void Tare(LoadCell* loadcell, uint8_t times);
void Set_Calibration(LoadCell* loadcell, float calibration);
float Get_Calibration(LoadCell* loadcell);
void Set_Offset(LoadCell* loadcell, double offset);
int32_t Get_Offset(LoadCell* loadcell);
void Power_Down(LoadCell* loadcell);
void Power_Up(LoadCell* loadcell);
void LoadCell_Setting(void);
float Get_Force_Right(void);
float Get_Force_Left(void);

#endif