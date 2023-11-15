#ifndef _INA219_H_
#define _INA219_H_

#include "Arduino.h"
#include "Wire.h"

#define INA219_LIB_VERSION              (F("0.1.5"))                    // lib version

//  REGISTER ADDRESSES
#define INA219_CONFIGURATION            0x00
#define INA219_SHUNT_VOLTAGE            0x01
#define INA219_BUS_VOLTAGE              0x02
#define INA219_POWER                    0x03
#define INA219_CURRENT                  0x04
#define INA219_CALIBRATION              0x05
#define INA219_MASK_ENABLE              0x06
#define INA219_ALERT_LIMIT              0x07
#define INA219_MANUFACTURER             0xFE
#define INA219_DIE_ID                   0xFF


//  CONFIGURATION REGISTER MASKS
#define INA219_CONF_RESET               0x8000
#define INA219_CONF_BUS_RANGE_VOLTAGE   0x2000
#define INA219_CONF_PROG_GAIN           0x1800
#define INA219_CONF_BUS_ADC             0x0780
#define INA219_CONF_SHUNT_ADC           0x0078
#define INA219_CONF_MODE                0x0007

class INA219
{
public:
  //  address between 0x40 and 0x4F
  explicit INA219(const uint8_t address, TwoWire *wire = &Wire);

#if defined(ESP8266) || defined(ESP32)
  bool     begin(const uint8_t sda, const uint8_t scl);
#elif defined (ARDUINO_ARCH_RP2040) && !defined(__MBED__)
  bool     begin(const uint8_t sda, const uint8_t scl);
#endif
  bool     begin();
  bool     isConnected();

  //  CORE FUNCTIONS               //  Register
  float    getShuntVoltage();      //  01
  float    getBusVoltage();        //  02
  float    getPower();             //  03
  float    getCurrent();           //  04
  bool     getMathOverflowFlag();  //  02
  bool     getConversionFlag();    //  02

  //  SCALE HELPERS - milli range
  float    getBusVoltage_mV()   { return getBusVoltage() * 1e3; };
  float    getShuntVoltage_mV() { return getShuntVoltage() * 1e3; };
  float    getCurrent_mA()      { return getCurrent() * 1e3; };
  float    getPower_mW()        { return getPower() * 1e3; };

  //  SCALE HELPERS - micro range
  float    getBusVoltage_uV()   { return getBusVoltage() * 1e6; };
  float    getShuntVoltage_uV() { return getShuntVoltage() * 1e6; };
  float    getCurrent_uA()      { return getCurrent() * 1e6; };
  float    getPower_uW()        { return getPower() * 1e6; };

  //  CONFIGURATION
  //  need improvement API wise.
  void     reset();
  //  voltage = 16, 32  (values below 32 are rounded to 16 or 32)
  bool     setBusVoltageRange(uint8_t voltage = 16);
  uint8_t  getBusVoltageRange();  //  returns 16 or 32.
  //  factor = 1, 2, 4, 8
  bool     setGain(uint8_t factor = 1);
  uint8_t  getGain();
  //  mask
  bool     setBusADC(uint8_t mask = 0x03);
  uint8_t  getBusADC();
  bool     setShuntADC(uint8_t mask = 0x03);
  uint8_t  getShuntADC();

  // Operating mode = 0..7
  bool     setMode(uint8_t mode = 7);
  uint8_t  getMode();
  bool     shutDown()                  { return setMode(0); };
  bool     setModeShuntTrigger()       { return setMode(1); };
  bool     setModeBusTrigger()         { return setMode(2); };
  bool     setModeShuntBusTrigger()    { return setMode(3); };
  bool     setModeADCOff()             { return setMode(4); };
  bool     setModeShuntContinuous()    { return setMode(5); };
  bool     setModeBusContinuous()      { return setMode(6); };
  bool     setModeShuntBusContinuous() { return setMode(7); };  //  default.


  //  CALIBRATION
  //  mandatory to set these! read datasheet.
  //  maxCurrent >= 0.001
  //  shunt      >= 0.001
  bool     setMaxCurrentShunt(float maxCurrent = 3.4, float shunt = 0.002);
  bool     isCalibrated()     { return _current_LSB != 0.0; };

  //  these return zero if not calibrated!
  float    getCurrentLSB()    { return _current_LSB; };
  float    getCurrentLSB_mA() { return _current_LSB * 1e3; };
  float    getCurrentLSB_uA() { return _current_LSB * 1e6; };
  float    getShunt()         { return _shunt; };
  float    getMaxCurrent()    { return _maxCurrent; };

  //  DEBUG
  uint16_t getRegister(uint8_t reg)  { return _readRegister(reg); };

private:

  uint16_t _readRegister(uint8_t reg);
  uint16_t _writeRegister(uint8_t reg, uint16_t value);
  float    _current_LSB;
  float    _shunt;
  float    _maxCurrent;

  uint8_t   _address;
  TwoWire * _wire;
};

#endif
