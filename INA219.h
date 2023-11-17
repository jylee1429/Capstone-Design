#ifndef _INA219_H_
#define _INA219_H_

#include "Arduino.h"
#include "Wire.h"          

#define DEVICE_NUM 4

#define REG_CONFIGURATION                         0x00
#define REG_SHUNT_VOLTAGE                         0x01
#define REG_BUS_VOLTAGE                           0x02
#define REG_POWER                                 0x03
#define REG_CURRENT                               0x04
#define REG_CALIBRATION                           0x05
#define REG_CONFIG_RESET                          0x8000        // Reset Bit
#define REG_CONFIG_BUS_RANGE_VOLTAGE_MASK         0x2000        // Bus Voltage Range
#define REG_CONFIG_BUS_RANGE_VOLTAGE_16V          0x0000        // 0-16V range
#define REG_CONFIG_BUS_RANGE_VOLTAGE_32V          0x2000        // 0-32V range
#define REG_CONFIG_GAIN_MASK                      0x1800        // Gain 
#define REG_CONFIG_GAIN_40MV                      0x0000        // Gain 1, 40mV Range
#define REG_CONFIG_GAIN_80MV                      0x0800        // Gain 2, 80mV Range
#define REG_CONFIG_GAIN_160MV                     0x1000        // Gain 3, 160mV Range
#define REG_CONFIG_GAIN_320MV                     0x1800        // Gain 4, 320mV Range
#define REG_CONFIG_BUS_ADC_MASK                   0x0780        // BUS_ADC Mask
#define REG_CONFIG_BUS_ADC_9BIT                   0x0000        // 9 bit
#define REG_CONFIG_BUS_ADC_10BIT                  0x0080        // 10 bit
#define REG_CONFIG_BUS_ADC_11BIT                  0x0100        // 11 bit
#define REG_CONFIG_BUS_ADC_12BIT                  0x0180        // 12 bit
#define REG_CONFIG_BUS_ADC_12BIT_2SAMPLES         0x0480        // 12-bit bus 2 samples averaged together
#define REG_CONFIG_BUS_ADC_12BIT_4SAMPLES         0x0500        // 12-bit bus 4 samples averaged together
#define REG_CONFIG_BUS_ADC_12BIT_8SAMPLES         0x0580        // 12-bit bus 8 samples averaged together
#define REG_CONFIG_BUS_ADC_12BIT_16SAMPLES        0x0600        // 12-bit bus 16 samples averaged together
#define REG_CONFIG_BUS_ADC_12BIT_32SAMPLES        0x0680        // 12-bit bus 32 samples averaged together
#define REG_CONFIG_BUS_ADC_12BIT_64SAMPLES        0x0700        // 12-bit bus 64 samples averaged together
#define REG_CONFIG_BUS_ADC_12BIT_128SAMPLES       0x0780        // 12-bit bus 128 samples averaged together
#define REG_CONFIG_SHUNT_ADC_MASK                 0x0078        // Shunt_ADC Mask
#define REG_CONFIG_SHUNT_ADC_9BIT                 0x0000        // 9 bit
#define REG_CONFIG_SHUNT_ADC_10BIT                0x0008        // 10 bit
#define REG_CONFIG_SHUNT_ADC_11BIT                0x0010        // 11 bit
#define REG_CONFIG_SHUNT_ADC_12BIT                0x0018        // 12 bit
#define REG_CONFIG_SHUNT_ADC_12BIT_2SAMPLES       0x0048        // 12-bit bus 2 samples averaged together
#define REG_CONFIG_SHUNT_ADC_12BIT_4SAMPLES       0x0050        // 12-bit bus 4 samples averaged together
#define REG_CONFIG_SHUNT_ADC_12BIT_8SAMPLES       0x0058        // 12-bit bus 8 samples averaged together
#define REG_CONFIG_SHUNT_ADC_12BIT_16SAMPLES      0x0060        // 12-bit bus 16 samples averaged together
#define REG_CONFIG_SHUNT_ADC_12BIT_32SAMPLES      0x0068        // 12-bit bus 32 samples averaged together
#define REG_CONFIG_SHUNT_ADC_12BIT_64SAMPLES      0x0070        // 12-bit bus 64 samples averaged together
#define REG_CONFIG_SHUNT_ADC_12BIT_128SAMPLES     0x0078        // 12-bit bus 128 samples averaged together
#define REG_CONFIG_MODE_MASK                      0x0007        // Operating Mode Mask
#define REG_CONFIG_POWERDOWN                      0x00          // power down
#define REG_CONFIG_SHUNT_VOLTAGE_TRIGGERED        0x01          // shunt voltage triggered
#define REG_CONFIG_BUS_VOLTAGE_TRIGGERED          0x02          // bus voltage triggered
#define REG_CONFIG_SHUNT_BUS_VOLTAGE_TRIGGERED    0x03          // shunt voltage triggered
#define REG_CONFIG_ADC_OFF                        0x04          // ADC off
#define REG_CONFIG_SHUNT_VOLTAGE_CONTINUOUS       0x05          // shunt voltage continuous
#define REG_CONFIG_BUS_VOLTAGE_CONTINUOUS         0x06          // bus voltage continuous
#define REG_CONFIG_SHUNT_BUS_VOLTAGE_CONTINUOUS   0x07          // shunt and bus voltage continuous

typedef struct _INA219{
  uint8_t address;
  TwoWire* wire;
  uint32_t calibration;
  uint32_t current_divider_mA;
  float power_multiplier_mW;
}INA219;

void Init(INA219* ina219, uint8_t address, TwoWire* wire);
uint8_t Begin(INA219* ina219);
uint8_t IsConnected(INA219* ina219);
int16_t ReadRegister(INA219* ina219, uint8_t reg);
uint16_t WriteRegister(INA219* ina219, uint8_t reg, uint16_t data);
float GetShuntVoltage(INA219* ina219);
float GetBusVoltage(INA219* ina219);
float GetPower(INA219* ina219);
float GetCurrent(INA219* ina219);
uint8_t Check_MathOverflow(INA219* ina219);
uint8_t Check_Conversion(INA219* ina219);
void Reset(INA219* ina219);
uint8_t SetBusVolRange(INA219* ina219, uint8_t voltage);
uint8_t GetBusVolRange(INA219* ina219);
uint8_t SetGain(INA219* ina219, uint8_t gain);
uint8_t GetGain(INA219* ina219);
uint8_t SetBusADC(INA219* ina219, uint8_t badc);
uint8_t GetBusADC(INA219* ina219);
uint8_t SetShuntADC(INA219* ina219, uint8_t sadc);
uint8_t GetShuntADC(INA219* ina219);
uint8_t SetMode(INA219* ina219, uint8_t mode);
uint8_t GetMode(INA219* ina219);
uint8_t SetCalibration(INA219* ina219);
void Current_Sensor_Init(void);
void Get_Current_right(float* right_u, float* right_v, float* right_w);
void Get_Current_left(float* left_u, float* left_v, float* left_w);

#endif