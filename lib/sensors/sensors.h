#ifndef _SENSOR_H_
#define _SENSOR_H_
#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_ADS1015.h"


/*!
*@brief Initialize Wire Library for sensors
*@param sda default 21
*@param scl default 22
*@param frequency default 100KHz
*/
void sensor_begin(int8_t sda_ = 21, int8_t scl_ = 22, uint32_t frequency_ = 100000U);

/*!
*@brief Oxygen concentration sensor config structure 
*@param address of ADS1115
*@param port of ADS1115 to which sensor is connected
*/
typedef struct _Oxy_config_t{
    byte address;
    uint8_t port;
}oxy_config_t;


/*!
*@brief Blower config structure for SPI pin configuration
*@param MOSI
*@param MISO
*@param CS
*@param clk
*/
typedef struct _Blower_config{
    int8_t MISO;
    int8_t MOSI;
    int8_t CS;
    int8_t clk;
}blower_config_t;

enum error {I2C_ERROR = 0};


/*!
*@brief Flow sensor class to get reading (RAW/Processed)
*@param Address of flow sensor
*/
class Flow_sensor{
    protected:
    uint8_t address;
    bool flow_sign;
    int polynomial;
    bool crc_error;
    uint8_t CRC_prim (uint8_t x, uint8_t crc);

    public:
    Flow_sensor(uint8_t _address);
    int16_t getReading();
    float getRAWReading();
    void flow_begin();
    bool is_breath();

};

/*!
*@brief Pressure sensor class to get reading (RAW/Processed)
*@param Address of pressure sensor
*/
class Pressure_sensor{
    protected:
    uint8_t address;

    public:
    Pressure_sensor(uint8_t _address);
    float getReading();
    int16_t getRAWReading();

};

/*!
*@brief oxygen sensor class to get reading (RAW/Processed)
*@param oxy_config_t oxygen configuration structure
*/
class Oxy_sensor{
    protected:
    oxy_config_t config;
    Adafruit_ADS1115 ads;

    public:
    Oxy_sensor(const oxy_config_t& _config);
    double getReading();
    double getRAWReading();

};


/*!
*@brief Blower control with SPI interface
*@param blower_config_t blower configuration structure
*/
class Blower{
    protected:
    blower_config_t config;
    SPIClass SPI1;

    public:
    Blower(const blower_config_t& _config);
    void setRPM(uint8_t val);
    void start();
    void stop();
};

/*!
*@brief Blower control with I2C interface
*@param address blower i2c address
*/
class Blower_i2c{
    protected:
    uint8_t address;

    public:
    Blower_i2c(uint8_t _address);
    void setRPM(uint16_t val, bool writeEEPROM = false);
    void start();
    void stop();
};

/*!
*@brief Value open/close/toggle control
*@param _pinNum pin Number connected to valve
*/
class Valve{
    private:
    bool pinState;

    protected:
    uint8_t pinNum;

    public:
    Valve(uint8_t _pinNum);
    void open();
    void close();
    void toggle();
};


#endif