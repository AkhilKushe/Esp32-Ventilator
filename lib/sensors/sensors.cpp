#include "sensors.h"

void sensor_begin(int8_t sda_, int8_t scl_, uint32_t frequency_){
        Wire.begin(sda_, scl_, frequency_);
}


Flow_sensor::Flow_sensor(uint8_t _address) : address(_address), flow_sign(0), polynomial(0x31), crc_error(0){}

void Flow_sensor::flow_begin(){
    //soft reset
    Wire.beginTransmission(address);
    Wire.write(0x20);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(10);

    Wire.beginTransmission(address);
    Wire.write(0x31);  // read serial number
    Wire.write(0xAE);  // command 0x31AE
    Wire.endTransmission();

    if (6 == Wire.requestFrom(address,6)) {
    uint32_t sn = 0;
    sn = Wire.read(); sn <<= 8;
    sn += Wire.read(); sn <<= 8;
    Wire.read(); // CRC - omitting for now
    sn += Wire.read(); sn <<= 8;
    sn += Wire.read();
    Wire.read(); // CRC - omitting for now
    Serial2.println(sn);
    } else {
    Serial2.println("serial number - i2c read error");
    }

    // start continuous measurement
    Wire.beginTransmission(address);
    Wire.write(0x10);
    Wire.write(0x00);
    Wire.endTransmission();
}

uint8_t Flow_sensor::CRC_prim (uint8_t x, uint8_t crc) {
    crc ^= x;
    for (uint8_t bit = 8; bit > 0; --bit) {
        if (crc & 0x80) crc = (crc << 1) ^ polynomial;
        else crc = (crc << 1);
    }
    return crc;
}

int16_t Flow_sensor::getReading(){
    int16_t raw = getRAWReading();
    if(raw == -1){
        return -1;
    }

    //processing on raw data to get in desired units
    return raw;
}

float Flow_sensor::getRAWReading(){
    if(3 == Wire.requestFrom(address, 3)){
        uint8_t crc = 0;
        uint16_t a = Wire.read();
        crc = CRC_prim (a, crc);
        uint8_t  b = Wire.read();
        crc = CRC_prim (b, crc);
        uint8_t  c = Wire.read();
        
        if ((crc_error = (crc != c)) ){ // report CRC error
            return -1;
        }
        a = (a<<8) | b;
        float new_flow = ((float)a - 32768) / 120;
        return new_flow;
    }
    else{
        return -1;
    }
}

bool Flow_sensor::is_breath(){
    return 0;
}



Pressure_sensor::Pressure_sensor(uint8_t _address) : address(_address) {}

float Pressure_sensor::getReading(){
    int16_t raw = getRAWReading();
    float sensitivity = 65.535;
    float readings = ((raw-1638)/sensitivity);
    return readings;
}

int16_t Pressure_sensor::getRAWReading(){
    byte byte_msb, byte_lsb;
    int16_t P_counts_diff;

    if(2 == Wire.requestFrom(address, 2u)){
        byte_msb = Wire.read();
        byte_lsb = Wire.read();
    }
    else{
        return -1;
    }

    P_counts_diff = ((int16_t)byte_msb << 8) | byte_lsb;
    float readings = ((P_counts_diff-1638)/65.535) - 100;
    return readings;
}



Oxy_sensor::Oxy_sensor(const oxy_config_t& _config) : config(_config), ads(config.address){
    ads.setGain(GAIN_SIXTEEN);   //change to gain sixteen                                         // +/- 0.256V, 1 bit = 0.0078125 mV
    //ads.begin();                                                        // called sensors_begin() to initialise wire lib
}

//reading in oxygen percentage 0-100 %
double Oxy_sensor::getReading(){
    uint16_t raw =  ads.readADC_SingleEnded(config.port);
    double mV = raw * 0.0078125;                                          // conver to mV
    mV = constrain(mV, 13, 16);
    double reading = (mV-13)*100/3;
    return reading;

}

//oxygen concentration sensor output voltage in mV
double Oxy_sensor::getRAWReading(){
    return (ads.readADC_SingleEnded(config.port))*0.0078125;
}



Blower::Blower(const blower_config_t& _config) : config(_config) , SPI1(HSPI){
    SPI1.begin(config.clk, config.MISO, config.MOSI, config.CS);
    pinMode(config.CS, OUTPUT);
    digitalWrite(config.CS, HIGH);
}

void Blower::setRPM(uint8_t val){
    SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(config.CS, LOW);
    SPI1.transfer(0B00010001);
    SPI1.transfer(val);
    digitalWrite(config.CS, HIGH);
    SPI1.endTransaction();
}


void Blower::start(){
    SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(config.CS, LOW);
    SPI1.transfer(0B00010001);
    SPI1.transfer(255);
    digitalWrite(config.CS, HIGH);
    SPI1.endTransaction();

}

void Blower::stop(){
    SPI1.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(config.CS, LOW);
    SPI1.transfer(0B00010001);
    SPI1.transfer(0);
    digitalWrite(config.CS, HIGH);
    SPI1.endTransaction();

}



Blower_i2c::Blower_i2c(uint8_t _address) : address(_address){
 
}

void Blower_i2c::setRPM(uint16_t val, bool writeEEPROM){
    uint8_t packet[3];

    if (writeEEPROM) {
        packet[0] = 0x60;
    } else {
        packet[0] = 0x40;
    }
    packet[1] = val >> 4;        // Upper data bits (D11.D10.D9.D8.D7.D6.D5.D4)
    packet[2] = (val & 15) << 4; // Lower data bits (D3.D2.D1.D0.x.x.x.x)
    Wire.beginTransmission(address);
    Wire.write(packet, 3);
    Wire.endTransmission();

}

void Blower_i2c::start(){
    uint16_t val = 0xFFFF;
    uint8_t packet[3];

    packet[0] = 0x40;
    packet[1] = val >> 4;        // Upper data bits (D11.D10.D9.D8.D7.D6.D5.D4)
    packet[2] = (val & 15) << 4; // Lower data bits (D3.D2.D1.D0.x.x.x.x)
    Wire.beginTransmission(address);
    Wire.write(packet, 3);
    Wire.endTransmission();

}

void Blower_i2c::stop(){
    uint16_t val = 0;
    uint8_t packet[3];

    packet[0] = 0x40;

    packet[1] = val >> 4;        // Upper data bits (D11.D10.D9.D8.D7.D6.D5.D4)
    packet[2] = (val & 15) << 4; // Lower data bits (D3.D2.D1.D0.x.x.x.x)
    Wire.beginTransmission(address);
    Wire.write(packet, 3);
    Wire.endTransmission();


}



Valve::Valve(uint8_t _pinNum) : pinState(0), pinNum(_pinNum) {
    pinMode(pinNum, OUTPUT);
    pinMode(pinNum, OUTPUT);
}

void Valve::open(){
    digitalWrite(pinNum, HIGH);
    pinState = 1;
}

void Valve::close(){
    digitalWrite(pinNum, LOW);
    pinState = 0;
}

void Valve::toggle(){
    digitalWrite(pinNum, !pinState);
    pinState = !pinState;
}





