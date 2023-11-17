#include "INA219.h"

void Init(INA219* ina219, uint8_t address, TwoWire* wire){
  ina219->address = address;
  ina219->wire = wire;
  ina219->calibration = 0;
  ina219->current_divider_mA = 0;
  ina219->power_multiplier_mW = 0;
  SetCalibration(ina219);                                                     // setting calibration(32V, 3A)
}

uint8_t Begin(INA219* ina219){
  ina219->wire->begin();

  if(! IsConnected(ina219))
    return false;
  
  return true;
}

uint8_t IsConnected(INA219* ina219){
  uint8_t address = ina219->address;

  if((address < 0x40) || (address > 0x4F))                                     // address : 0x40, 0x41, 0x43, 0x44 
    return false;
  ina219->wire->beginTransmission(address);

  return (ina219->wire->endTransmission() == 0);
}

int16_t ReadRegister(INA219* ina219, uint8_t reg){
  int16_t val;
  // write
  ina219->wire->beginTransmission(ina219->address);
  ina219->wire->write(reg);
  ina219->wire->endTransmission();

  // read
  ina219->wire->requestFrom(ina219->address, (uint8_t)2);                      // request 2 byte
  val =  ina219->wire->read();                                                 // MSB 
  val <<= 8;
  val |= ina219->wire->read();                                                 // LSB

  return val;
}

uint16_t WriteRegister(INA219* ina219, uint8_t reg, uint16_t data){
  uint16_t val;

  ina219->wire->beginTransmission(ina219->address);
  ina219->wire->write(reg);
  ina219->wire->write(data >> 8);                                               // MSB
  ina219->wire->write(data & 0xFF);                                             // LSB
  val = ina219->wire->endTransmission();  

  return val;
}

float GetShuntVoltage(INA219* ina219){                                          // [mV]
  int16_t vol = ReadRegister(ina219, REG_SHUNT_VOLTAGE); 
  return vol * (1e-2);                                                          // datasheet p26              
}

float GetBusVoltage(INA219* ina219){                                            // [V]
  float voltage;
  uint16_t vol = ReadRegister(ina219, REG_BUS_VOLTAGE);
  
  // flags && 0x02 ==> convert flag
  if(Check_MathOverflow(ina219))
    return -100;

  voltage = (vol >> 3) * (4e-3);                                                // LSB = 4 [mV] (4mV per bit)
                                                                                // voltage value is represented by the upper 13 bits of REG_BUS_VOLTAGE
  return voltage;
}

float GetPower(INA219* ina219){                                                 // [mW]
  int16_t val = ReadRegister(ina219, REG_POWER);
  float power = val * (ina219->power_multiplier_mW);                            // power_multiplier_mW per bit
  
  return power;
}

float GetCurrent(INA219* ina219){                                               // [mA]
  float current;

  WriteRegister(ina219, REG_CALIBRATION, ina219->calibration);                  // calibration reset
  int16_t val = ReadRegister(ina219, REG_CURRENT);
  current = (float)val / (ina219->current_divider_mA);                          // current_divider_mA per bit
  return current;
}

uint8_t Check_MathOverflow(INA219* ina219){
  uint16_t val = ReadRegister(ina219, REG_BUS_VOLTAGE);
  if((val & 0x0001) == 0x0001)
    return true;
  else
    return false;
}

uint8_t Check_Conversion(INA219* ina219){
  uint16_t val = ReadRegister(ina219, REG_BUS_VOLTAGE);
  
  if((val & 0x0002) == 0x0002)
    return true;
  else
    return false;
}

// configuration

void Reset(INA219* ina219){
  uint16_t config = ReadRegister(ina219, REG_CONFIGURATION);

  config |= REG_CONFIG_RESET;
  WriteRegister(ina219, REG_CONFIGURATION, config);

  ina219->calibration = 0;
  ina219->current_divider_mA = 0;
  ina219->power_multiplier_mW = 0;
}

uint8_t SetBusVolRange(INA219* ina219, uint8_t voltage){
  uint16_t config;
  config = ReadRegister(ina219, REG_CONFIGURATION);
  if(voltage > 32)
    return false;
  else if (voltage > 16) {
    voltage = 32;
    config |= REG_CONFIG_BUS_RANGE_VOLTAGE_32V;                           // 32V FSR
  }
  else {
    voltage = 16;
    config &= (~REG_CONFIG_BUS_RANGE_VOLTAGE_32V);                        // 16V FSR
  } 

  WriteRegister(ina219, REG_CONFIGURATION, config);
  return true;
}

uint8_t GetBusVolRange(INA219* ina219) {
  uint16_t config = ReadRegister(ina219, REG_CONFIGURATION);
  if(config & REG_CONFIG_BUS_RANGE_VOLTAGE_32V)
    return 32;
  else 
    return 16;
}

uint8_t SetGain(INA219* ina219, uint8_t gain){
  if(gain != 1 && gain != 2 && gain != 4 && gain != 8)
    return false;
  
  uint16_t config = ReadRegister(ina219, REG_CONFIGURATION);
  config &= (~REG_CONFIG_GAIN_MASK);

  if(gain == 2)
    config |= (1<<11);                                                    // gain = /2
  else if(gain == 4)
    config |= (1<<12);                                                    // gain = /4
  else if(gain == 8)
    config |= (3<<11);                                                    // gain = /8
  
  WriteRegister(ina219, REG_CONFIGURATION, config);
  return true;
}

uint8_t GetGain(INA219* ina219){
  uint16_t config = ReadRegister(ina219, REG_CONFIGURATION);
  uint16_t PG = config & REG_CONFIG_GAIN_MASK;
  if(PG == 0x0000)
    return 1;
  else if(PG == 0x0800)
    return 2;
  else if(PG == 0x1000)
    return 4;
  else 
    return 8;
}

uint8_t SetBusADC(INA219* ina219, uint8_t badc){
  if(badc > 0x0F)
    return false;
  
  uint16_t config = ReadRegister(ina219, REG_CONFIGURATION);
  config &= (~REG_CONFIG_BUS_ADC_MASK);
  config |= (badc << 7);
  WriteRegister(ina219, REG_CONFIGURATION, config);
  return true;
}

uint8_t GetBusADC(INA219* ina219){
  uint16_t config = ReadRegister(ina219, REG_CONFIGURATION);
  config &= REG_CONFIG_BUS_ADC_MASK;
  return config >> 7;
}

uint8_t SetShuntADC(INA219* ina219, uint8_t sadc){
  if(sadc > 0x0F)
    return false;
  
  uint16_t config = ReadRegister(ina219, REG_CONFIGURATION);
  config &= (~REG_CONFIG_SHUNT_ADC_MASK);
  config |= (sadc << 3);
  WriteRegister(ina219, REG_CONFIGURATION, config);
  return true;
}

uint8_t GetShuntADC(INA219* ina219){
  uint16_t config = ReadRegister(ina219, REG_CONFIGURATION);
  config &= REG_CONFIG_SHUNT_ADC_MASK;
  return config >> 3;
}

uint8_t SetMode(INA219* ina219, uint8_t mode){
  if(mode > 7)
    return false;
  uint16_t config = ReadRegister(ina219, REG_CONFIGURATION);
  config &= (~REG_CONFIG_MODE_MASK);
  config |= mode;
  WriteRegister(ina219, REG_CONFIGURATION, config);
  return true;
}

uint8_t GetMode(INA219* ina219){
  uint16_t config = ReadRegister(ina219, REG_CONFIGURATION);
  config &= REG_CONFIG_MODE_MASK;
  return config;
}

uint8_t SetCalibration(INA219* ina219){
  // VBUS_MAX = 32 [V]
  // VSHUNT_MAX = 0.32 [V]
  // RSHUNT = 0.1 [Ohm]

  // Max Possible i = VSHUNT_MAX / RSHUNT = 3.2 [A]
  // Max Expected i = 3.0 [A]

  // Minimum LSB = Max Possible i / 32767 = 0.0000976 (97.6uA per bit)
  // Maximum LSB = Max Expected i / 4096 = 0.000732 (732uA per bit)

  // Current LSB = 0.0001 (100uA per bit) = 0.1mA per bit
  // Calibration = trunc (0.04096 / (Current_LSB * RSHUNT)) = 4096 (0x1000)
  ina219->calibration = 4096;
  
  // Power LSB = 20 * Current LSB
  //           = 0.002 (2mW per bit)

  // Max_Current = Current_LSB * 32767
  // Max_Current = 3.2767A before overflow

  // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
  // Max_ShuntVoltage = 0.32V

  // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
  // MaximumPower = 3.2 * 32V
  // MaximumPower = 102.4W
  ina219->current_divider_mA = 10;                                      // Current LSB = 100uA per bit(1bit->100uA), ex) REG_CURRENT = 0x03 -> 300uA = 0.3mA
  ina219->power_multiplier_mW = 2;                                      // Power LSB = 2mW per bit (2/1), ex) REG_POWER = 0x03 -> 6mW
  WriteRegister(ina219, REG_CALIBRATION, ina219->calibration);
  
  uint16_t config = ReadRegister(ina219, REG_CONFIGURATION);
  
  config |= REG_CONFIG_BUS_RANGE_VOLTAGE_32V;
  config |= REG_CONFIG_GAIN_320MV;
  config |= REG_CONFIG_BUS_ADC_12BIT;
  config |= REG_CONFIG_SHUNT_ADC_12BIT;
  config |= REG_CONFIG_SHUNT_BUS_VOLTAGE_CONTINUOUS;

  WriteRegister(ina219, REG_CONFIGURATION, config);

  return true;}