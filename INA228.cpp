
//Libraries
#include "neoINA228.h"

// Constructors 
neoINA228::neoINA228(neoI2C& _i2c) 
{
    i2c = _i2c;
}

// Checks against Manufacturer ID Register (0x3E) [reset = 5449h] to see if I2C is connected
// Sets Configuration, ADC Configuration, and Shunt Calibration
bool neoINA228::begin(uint8_t address, float shnt_ul, float shnt_ol, float bus_ol, float bus_ul, float tmp_ol, float pwr_ol)
{
    I2CAddress = address;
    uint16_t i2c_check = 0;

    i2c_check = getManID();
     
    if((i2c_check) == 0x5449){
        configure();
        adcconfig();
        calibrate();

        writeInt16(INA228_REG_SOVL, min(shnt_ol, 0.163835)/0.000005 );
        writeInt16(INA228_REG_SUVL, max(shnt_ul, -0.163834)/0.000005);
        writeRegister16(INA228_REG_BOVL, min(bus_ol, 102.4)/0.003125);
        writeRegister16(INA228_REG_BUVL, min(bus_ul, 102.4)/0.003125);
        writeInt16(INA228_REG_TEMPLIMIT, min(tmp_ol, 255.9922)/0.0078125);
        writeRegister16(INA228_REG_PWRLIMIT, min(pwr_ol, 51199.22)/(powerLSB * 256));

       // double value;
        ///value = readRegister16(INA228_REG_BOVL);
        //value = value * .003125;

        //NRF_LOG_INFO("This is BOVL %s", f2s(value,8));
       // NRF_LOG_FLUSH();

        uint16_t adc_reg = readRegister16(INA228_REG_ADCCONFIG);

        writeRegister16(INA228_REG_DIAGALRT, 0x8003); 

        //NRF_LOG_INFO("This is adcconfig %x", adc_reg);

        return true;
    } else {
        return false;
    }
}

// Register (0x00) Configuration - Page 22
bool neoINA228::configure(ina228_shunt_range shunt, bool reset){

    _shunt = shunt;

    uint16_t config = 0x0;
    config = config | (_shunt << 4);
    if(reset == true) {
        config = config | 0x4000;
    }
    
    writeRegister16(INA228_REG_CONFIG, config);

    uint16_t config_reg = readRegister16(INA228_REG_CONFIG);

    //NRF_LOG_INFO("This is config %x", config_reg);

    return true;
}

void neoINA228::resetCharge(bool reset) {
    uint16_t config = 0x0;

    config = config | (_shunt << 4);
    
    if(reset == true) {
        config = config | 0x4000;
    }
    
    writeRegister16(INA228_REG_CONFIG, config);
}


// Register (0x01) ADC Configuration - Pages 22-24
bool neoINA228::adcconfig(ina228_averages_t avg, ina228_busConvTime_t busConvTime, ina228_shuntConvTime_t shuntConvTime, ina228_tempConvTime_t tempConvTime, ina228_mode_t mode)
{
    uint16_t adcconfig = 0;

    adcconfig |= (uint16_t)(mode << 12 | busConvTime << 9 | shuntConvTime << 6 | tempConvTime << 3 | avg);

    writeRegister16(INA228_REG_ADCCONFIG, adcconfig);

    return true;
}

// Register (0x02) Shunt Calibration - Page 24
bool neoINA228::calibrate()
{
    uint16_t calibrationValue;

    currentLSB = iMaxCurrentExpected / pow(2.0,19.0);

    powerLSB = currentLSB * 3.2;
    energyLSB = currentLSB * 51.2;
    chargeLSB = currentLSB;

    calibrationValue = (uint16_t)(13107.2 * pow(10.0,6.0) * currentLSB * rShuntValue);

    writeRegister16(INA228_REG_CURRLSBCALC, calibrationValue);
    
    return true;
}

// Register (0x03) Shunt Temperature Coefficient - Page 24
bool neoINA228::temperature_configure(uint16_t temp_coef)
{
    writeRegister16(INA228_REG_TEMPCOCONFIG, temp_coef);

    return true;
}

// Register (0x08) Power Result - Page 25-26
double neoINA228::getBusPower(void)
{
    double power = readRegister24(INA228_REG_POWER);
    return (power * powerLSB);
}

// Register (0x07) Current Result - Page 25
double neoINA228::getShuntCurrent(void)
{
    int32_t current_raw = readInt24(INA228_REG_CURRENT) / 16;

    return ( (double)current_raw * currentLSB );
}

// Register (0x04) Shunt Voltage Measurement (VSHUNT) - Pages 24-25
double neoINA228::getShuntVoltage(void)
{
    int32_t voltage_raw;
    int16_t config;

    config = readRegister16(INA228_REG_CONFIG);
    voltage_raw = readInt24(INA228_REG_VSHUT) / 16;

    if((config >> 4) & 1)
        return ((double)voltage_raw * 0.000000078125);
    else
        return ((double)voltage_raw * 0.0000003125);
}

// Register (0x05) Bus Voltage Measurement (VBUS) - Page 25
double neoINA228::getBusVoltage(void)
{
    int32_t voltage_raw = readInt24(INA228_REG_VBUS) >> 4;

    return ((double)voltage_raw * 0.0001953125);
}

// Register (0x06) Temperature Measurement (DIETEMP) - Page 25
double neoINA228::getTemperature(void)
{
    int16_t temperature = readRegister16(INA228_REG_DIETEMP);

    return ((float)temperature * 0.0078125);
}

// Register (0x0A) Charge Result - Page 26
double neoINA228::getCharge(void) {
    double charge = readInt40(INA228_REG_CHARGE);
    return (charge * chargeLSB);
}

double neoINA228::getAmpHours(void) {
    
    double amp_hours = getCharge();
    amp_hours = amp_hours / 3600.0;

    return amp_hours;
}

double neoINA228::getMilliampHours(void) {
    
    double milliamp_hours = getCharge();
    milliamp_hours = milliamp_hours * 1000.0;
    milliamp_hours = milliamp_hours / 3600.0;

    return milliamp_hours;
}


// Register (0x0B) Diagnostic Flags and Alert - Pages 26-28
bool neoINA228::alertStatus(void){

    uint16_t errors = readRegister16(INA228_REG_DIAGALRT);

    uint16_t errors_masked = errors | 0x8003;

    if(errors_masked == 0x8003) {
        return false;
    } else {
        if (errors & 0) {
            NRF_LOG_INFO("Memory Checksum ERROR");
        }
        if (errors & (1 << 1)) {
            NRF_LOG_INFO("Conversion is complete");
        }
        if (errors & (1 << 2)) {
            NRF_LOG_INFO("Power Over-Limit Event ERROR");
        }
        if (errors & (1 << 3)) {
            NRF_LOG_INFO("Bus Under-Limit Event ERROR");
        }
        if (errors & (1 << 4)) {
            NRF_LOG_INFO("Bus Over-Limit Event ERROR");
        }
        if (errors & (1 << 5)) {
            NRF_LOG_INFO("Under Shunt Voltage Event ERROR");
        }
        if (errors & (1 << 6)) {
            NRF_LOG_INFO("Over Shunt Voltage Event ERROR");
        }
        if (errors & (1 << 7)) {
            NRF_LOG_INFO("Over Temp Event ERROR");
        }
        if (errors & (1 << 9)) {
            NRF_LOG_INFO("Math Overflow ERROR");
        }
        if (errors & (1 << 10)) {
            NRF_LOG_INFO("Charge Overflow");
        }
        if (errors & (1 << 11)) {
            NRF_LOG_INFO("Energy Overflow");
        }
        return true;
    }
}

// Register (0x0C) Shunt Overvoltage Threshold (SOVL) - Pages 28
void neoINA228::setShuntOverVoltageLimit(float voltage)
{
    uint16_t value, config = readRegister16(INA228_REG_CONFIG);

    if((config >> 4) & 1)
        value = voltage / 0.000000078125;
    else
        value = voltage / 0.0000003125;

    writeRegister16(INA228_REG_SOVL, value);
}

// Register (0x0D) Shunt Undervoltage Threshold (SUVL) - Pages 28
void neoINA228::setShuntUnderVoltageLimit(float voltage)
{
    uint16_t value, config = readRegister16(INA228_REG_CONFIG);
    
    if((config >> 4) & 1)
        value = voltage / 0.000000078125;
    else
        value = voltage / 0.0000003125;

    writeRegister16(INA228_REG_SUVL, value);
}

// Register (0x10) Temperature Over-Limit Threshold (TEMP_LIMIT) - Pages 29
void neoINA228::setTemperatureLimit(float temperature)
{
    uint16_t value = temperature / 0.0078125;
    writeRegister16(INA228_REG_TEMPLIMIT, value);
}

// Register (0x11) Power Over-Limit Threshold (PWR_LIMIT) - Pages 29
void neoINA228::setPowerLimit(float watts)
{
    uint16_t value = watts / powerLSB;
    writeRegister16(INA228_REG_PWRLIMIT, value);
}

ina228_averages_t neoINA228::getAverages(void)
{
    uint16_t value;

    value = readRegister16(INA228_REG_ADCCONFIG);
    value &= 0b0000000000000111;

    return (ina228_averages_t)value;
}

ina228_busConvTime_t neoINA228::getBusConversionTime(void)
{
    uint16_t value;

    value = readRegister16(INA228_REG_ADCCONFIG);
    value &= 0b0000111000000000;
    value >>= 9;

    return (ina228_busConvTime_t)value;
}

ina228_tempConvTime_t neoINA228::getTemperatureConversionTime(void)
{
    uint16_t value;

    value = readRegister16(INA228_REG_ADCCONFIG);
    value &= 0b0000000000111000;
    value >>= 3;

    return (ina228_tempConvTime_t)value;
}

ina228_shuntConvTime_t neoINA228::getShuntConversionTime(void)
{
    uint16_t value;

    value = readRegister16(INA228_REG_ADCCONFIG);
    value &= 0b0000000111000000;
    value >>= 6;

    return (ina228_shuntConvTime_t)value;
}

ina228_mode_t neoINA228::getMode(void)
{
    uint16_t value;

    value = readRegister16(INA228_REG_ADCCONFIG);
    value &= 0b1111000000000000;
    value >>= 12;

    return (ina228_mode_t)value;
}

float neoINA228::getMaxPossibleCurrent(void)
{
    return (vShuntMax / rShuntValue);
}

float neoINA228::getMaxCurrent(void)
{
    float maxCurrent = (currentLSB * 524288);
    float maxPossible = getMaxPossibleCurrent();

    if (maxCurrent > maxPossible)
    {
        return maxPossible;
    } else
    {
        return maxCurrent;
    }
}

float neoINA228::getMaxShuntVoltage(void)
{
    float maxVoltage = getMaxCurrent() * rShuntValue;

    if (maxVoltage >= vShuntMax)
    {
        return vShuntMax;
    } else
    {
        return maxVoltage;
    }
}

float neoINA228::getMaxPower(void)
{
    return (getMaxCurrent() * vBusMax);
}

void neoINA228::setMaskEnable(uint16_t mask)
{
    writeRegister16(INA228_REG_DIAGALRT, mask);
}

uint16_t neoINA228::getMaskEnable(void)
{
    return readRegister16(INA228_REG_DIAGALRT);
}

void neoINA228::enableShuntOverLimitAlert(void)
{
    writeRegister16(INA228_REG_DIAGALRT, INA228_BIT_SOL);
}

void neoINA228::enableShuntUnderLimitAlert(void)
{
    writeRegister16(INA228_REG_DIAGALRT, INA228_BIT_SUL);
}

void neoINA228::enableBusOverLimitAlert(void)
{
    writeRegister16(INA228_REG_DIAGALRT, INA228_BIT_BOL);
}

void neoINA228::enableBusUnderLimitAlert(void)
{
    writeRegister16(INA228_REG_DIAGALRT, INA228_BIT_BUL);
}

void neoINA228::enableTempOverLimitAlert(void)
{
    writeRegister16(INA228_REG_DIAGALRT, INA228_BIT_TMPOL);
}

void neoINA228::enableOverPowerLimitAlert(void)
{
    writeRegister16(INA228_REG_DIAGALRT, INA228_BIT_POL);
}

void neoINA228::enableConversionReadyAlert(void)
{
    writeRegister16(INA228_REG_DIAGALRT, INA228_BIT_CNVR);
}

void neoINA228::setBusOverVoltageLimit(float voltage)
{
    uint16_t value = voltage / 0.0001953125;
    writeRegister16(INA228_REG_BOVL, value);
}

void neoINA228::setBusUnderVoltageLimit(float voltage)
{
    uint16_t value = voltage / 0.0001953125;
    writeRegister16(INA228_REG_BUVL, value);
}

void neoINA228::setAlertInvertedPolarity(bool inverted)
{
    uint16_t temp = getMaskEnable();

    if (inverted)
    {
        temp |= INA228_BIT_APOL;
    } else
    {
        temp &= ~INA228_BIT_APOL;
    }

    setMaskEnable(temp);
}

void neoINA228::setAlertLatch(bool latch)
{
    uint16_t temp = getMaskEnable();

    if (latch)
    {
        temp |= INA228_BIT_LEN;
    } else
    {
        temp &= ~INA228_BIT_LEN;
    }

    setMaskEnable(temp);
}

bool neoINA228::isMathOverflow(void)
{
    return ((getMaskEnable() & INA228_BIT_OVF) == INA228_BIT_OVF);
}

bool neoINA228::isAlert(void)
{
    return ((getMaskEnable() & INA228_BIT_SLWALRT) == INA228_BIT_SLWALRT);
}

int neoINA228::getManID(void)
{
    return readRegister16(INA228_REG_MANUFACTURERID);
}

// Reading and Writing Functions

uint16_t neoINA228::readRegister16(uint8_t reg)
{
    
    uint8_t data[2];  
    i2c.write(I2CAddress, &reg, 1, true);
    i2c.read(I2CAddress, data, 2);
    uint16_t value = data[0] << 8 | data[1];
    return value;
}

void neoINA228::writeRegister16(uint8_t reg, uint16_t value)
{
    uint8_t data[3];
    data[0] = reg;                      // register pointer
    data[1] = (value >> 8) & 0xFF;      // MSB first
    data[2] =  value        & 0xFF;     // LSB
    i2c.write(I2CAddress, data, 3);
}

void neoINA228::writeInt16(uint8_t reg, int16_t value)
{
    uint8_t data[3];
    data[0] = reg;
    data[1] = (value >> 8) & 0xFF;      // MSB first
    data[2] =  value        & 0xFF;     // LSB
    i2c.write(I2CAddress, data, 3);
}

uint32_t neoINA228::readRegister24(uint8_t reg)
{
    uint8_t data[3];
    i2c.write(I2CAddress, &reg, 1, true);   // repeated-start
    i2c.read(I2CAddress, data, 3);

    return  (uint32_t)data[0] << 16 |       // MSB
            (uint32_t)data[1] <<  8 |
                           data[2];         // LSB
}

int32_t neoINA228::readInt24(uint8_t reg)
{
    uint8_t data[3];
    i2c.write(I2CAddress, &reg, 1, true);
    i2c.read(I2CAddress, data, 3);

    // assemble MSB-first
    int32_t value =  (int32_t)data[0] << 16 |
                     (int32_t)data[1] <<  8 |
                                  data[2];

    // manual sign-extend from 24 bits to 32 bits
    if (value & 0x800000) { value |= 0xFF000000; }
    return value;
}

uint64_t neoINA228::readRegister40(uint8_t reg)
{
    uint8_t data[5];
    i2c.write(I2CAddress, &reg, 1, true);
    i2c.read(I2CAddress, data, 5);

    return  (uint64_t)data[0] << 32 |       // MSB
            (uint64_t)data[1] << 24 |
            (uint64_t)data[2] << 16 |
            (uint64_t)data[3] <<  8 |
                           data[4];         // LSB
}

int64_t neoINA228::readInt40(uint8_t reg)
{
    uint8_t data[5];
    i2c.write(I2CAddress, &reg, 1, true);
    i2c.read(I2CAddress, data, 5);

    int64_t value =  (int64_t)data[0] << 32 |
                     (int64_t)data[1] << 24 |
                     (int64_t)data[2] << 16 |
                     (int64_t)data[3] <<  8 |
                                  data[4];

    // sign-extend from 40 bits to 64 bits
    if (value & 0x8000000000LL) { value |= 0xFFFFFF0000000000LL; }
    return value;
}