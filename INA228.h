#ifndef NEOINA228_h
#define NEOINA228_h

//Libraries
#include "neoI2C.h"



//Addresses for INA288 registers
#define INA228_REG_CONFIG           (0x00) //Configuration
#define INA228_REG_ADCCONFIG        (0x01) //ADC Configuration
#define INA228_REG_CURRLSBCALC      (0x02) //Shunt Calibration
#define INA228_REG_TEMPCOCONFIG     (0x03) //Shunt Temperature Coefficient
#define INA228_REG_VSHUT	    (0X04) //Shunt Voltage Measurement
#define INA228_REG_VBUS             (0x05) //Bus Voltage Measurement
#define INA228_REG_DIETEMP          (0x06) //Temperature Measurement
#define INA228_REG_CURRENT          (0x07) //Current Result
#define INA228_REG_POWER            (0x08) //Power Result
#define INA228_REG_ENERGY           (0x09) //Energy Result
#define INA228_REG_CHARGE           (0x0A) //Charge Result
#define INA228_REG_DIAGALRT         (0x0B) //Diagnostic Flags and Alert
#define INA228_REG_SOVL             (0x0C) //Shunt Overvoltage Threshold
#define INA228_REG_SUVL             (0x0D) //Shunt Undervoltage Threshold
#define INA228_REG_BOVL             (0x0E) //Bus Overvoltage Threshold
#define INA228_REG_BUVL             (0x0F) //Bus Undervoltage Threshold
#define INA228_REG_TEMPLIMIT        (0x10) //Temperature Over-Limit Threshold
#define INA228_REG_PWRLIMIT         (0x11) //Power Over-Limit Threshold
#define INA228_REG_MANUFACTURERID   (0x3E) //Manufacturer ID
#define INA228_REG_DEVICEID         (0x3F) //Device ID

//DIAG_ALRT Register Field Descriptions
#define INA228_BIT_LEN              (0x8000)
#define INA228_BIT_CNVR             (0x4000)
#define INA228_BIT_SLWALRT          (0x2000)
#define INA228_BIT_APOL             (0x1000)
#define INA228_BIT_ENRGOF           (0x0800)
#define INA228_BIT_CHROF            (0x0400)
#define INA228_BIT_OVF              (0x0200)
#define INA228_BIT_TMPOL            (0x0080)
#define INA228_BIT_SOL              (0x0040)
#define INA228_BIT_SUL              (0x0020)
#define INA228_BIT_BOL              (0x0010)
#define INA228_BIT_BUL              (0x0008)    
#define INA228_BIT_POL              (0x0004)
#define INA228_BIT_CVRF             (0x0002)
#define INA228_BIT_MEM              (0x0001)

//Register Sizes in bits
#define INA228_REGISTER_BITS_CONFIG 16
#define INA228_REGISTER_BITS_ADC_CONFIG 16
#define INA228_REGISTER_BITS_SHUNT_CAL 16
#define INA228_REGISTER_BITS_SHUNT_TEMPCO 16
#define INA228_REGISTER_BITS_VSHUNT 24
#define INA228_REGISTER_BITS_VBUS 24
#define INA228_REGISTER_BITS_DIETEMP 16
#define INA228_REGISTER_BITS_CURRENT 24
#define INA228_REGISTER_BITS_POWER 24
#define INA228_REGISTER_BITS_ENERGY 40
#define INA228_REGISTER_BITS_CHARGE 40
#define INA228_REGISTER_BITS_DIAG_ALRT 16
#define INA228_REGISTER_BITS_SOVL 16
#define INA228_REGISTER_BITS_SUVL 16
#define INA228_REGISTER_BITS_BOVL 16
#define INA228_REGISTER_BITS_BUVL 16
#define INA228_REGISTER_BITS_TEMP_LIMIT 16
#define INA228_REGISTER_BITS_PWR_LIMIT 16
#define INA228_REGISTER_BITS_MANUFACTURER_ID 16
#define INA228_REGISTER_BITS_DEVICE_ID 16



typedef enum
{
    INA228_MODE_SHUTDOWN            = 0x0,
    INA228_MODE_BUS_TRIG            = 0x1,
    INA228_MODE_SHUNT_TRIG          = 0x2,
    INA228_MODE_SHUNT_BUS_TRIG      = 0x3,
    INA228_MODE_TEMP_TRIG           = 0x4,
    INA228_MODE_TEMP_BUS_TRIG       = 0x5,
    INA228_MODE_TEMP_SHUNT_TRIG     = 0x6,
    INA228_MODE_BUS_SHUNT_TEMP_TRIG = 0x7,
    INA228_MODE_SHUTDOWN2           = 0x8,
    INA228_MODE_BUS_CONT            = 0x9,
    INA228_MODE_SHUNT_CONT          = 0xA,
    INA228_MODE_SHUNT_BUS_CONT      = 0xB,
    INA228_MODE_TEMP_CONT           = 0xC,
    INA228_MODE_BUS_TEMP_CONT       = 0xD,
    INA228_MODE_TEMP_SHUNT_CONT     = 0xE,
    INA228_MODE_BUS_SHUNT_TEMP_CONT = 0xF
} ina228_mode_t;

typedef enum
{
    INA228_BUS_CONV_TIME_50US     = 0x0,
    INA228_BUS_CONV_TIME_84US     = 0x1,
    INA228_BUS_CONV_TIME_150US    = 0x2,
    INA228_BUS_CONV_TIME_280US    = 0x3,
    INA228_BUS_CONV_TIME_540US    = 0x4,
    INA228_BUS_CONV_TIME_1052US   = 0x5,
    INA228_BUS_CONV_TIME_2074US   = 0x6,
    INA228_BUS_CONV_TIME_4120US   = 0x7
} ina228_busConvTime_t;

typedef enum
{
    INA228_SHUNT_CONV_TIME_50US     = 0x0,
    INA228_SHUNT_CONV_TIME_84US     = 0x1,
    INA228_SHUNT_CONV_TIME_150US    = 0x2,
    INA228_SHUNT_CONV_TIME_280US    = 0x3,
    INA228_SHUNT_CONV_TIME_540US    = 0x4,
    INA228_SHUNT_CONV_TIME_1052US   = 0x5,
    INA228_SHUNT_CONV_TIME_2074US   = 0x6,
    INA228_SHUNT_CONV_TIME_4120US   = 0x7
} ina228_shuntConvTime_t;

typedef enum
{
    INA228_TEMP_CONV_TIME_50US      = 0x0,
    INA228_TEMP_CONV_TIME_84US      = 0x1,
    INA228_TEMP_CONV_TIME_150US     = 0x2,
    INA228_TEMP_CONV_TIME_280US     = 0x3,
    INA228_TEMP_CONV_TIME_540US     = 0x4,
    INA228_TEMP_CONV_TIME_1052US    = 0x5,
    INA228_TEMP_CONV_TIME_2074US    = 0x6,
    INA228_TEMP_CONV_TIME_4120US    = 0x7
} ina228_tempConvTime_t;

typedef enum
{
    INA228_AVERAGES_1             = 0x0,
    INA228_AVERAGES_4             = 0x1,
    INA228_AVERAGES_16            = 0x2,
    INA228_AVERAGES_64            = 0x3,
    INA228_AVERAGES_128           = 0x4,
    INA228_AVERAGES_256           = 0x5,
    INA228_AVERAGES_512           = 0x6,
    INA228_AVERAGES_1024          = 0x7
} ina228_averages_t;

typedef enum
{
    INA228_SHUNT_RANGE_163MV    = 0x0,
    INA228_SHUNT_RANGE_40MV     = 0x1
}   ina228_shunt_range;

class neoINA228
{
    public:


    //methods
    neoINA228(neoI2C& _i2c); //constructor prototype

    bool begin(uint8_t I2CAddress, float shnt_ul = -.163834, float shnt_ol = .163835, float bus_ol = 102.4, float bus_ul = 102.4, float tmp_ol = 255.9922, float pwr_ol = 51199.22);
    bool configure(ina228_shunt_range shunt = INA228_SHUNT_RANGE_163MV, bool reset = false);
    void resetCharge(bool reset = false);
    bool adcconfig(ina228_averages_t avg = INA228_AVERAGES_16, ina228_busConvTime_t busConvTime = INA228_BUS_CONV_TIME_1052US, ina228_shuntConvTime_t shuntConvTime = INA228_SHUNT_CONV_TIME_1052US, ina228_tempConvTime_t tempConvTime = INA228_TEMP_CONV_TIME_1052US, ina228_mode_t mode = INA228_MODE_BUS_SHUNT_TEMP_CONT);
    bool calibrate();
    bool temperature_configure(uint16_t temp_coef);

    ina228_mode_t getMode(void);
    ina228_busConvTime_t getBusConversionTime(void);
    ina228_shuntConvTime_t getShuntConversionTime(void);
    ina228_tempConvTime_t getTemperatureConversionTime(void);
    ina228_averages_t getAverages(void);

    void enableShuntOverLimitAlert(void);
    void enableShuntUnderLimitAlert(void);
    void enableBusOverLimitAlert(void);
    void enableBusUnderLimitAlert(void);
    void enableTempOverLimitAlert(void);
    void enableOverPowerLimitAlert(void);
    void enableConversionReadyAlert(void);

    void setBusOverVoltageLimit(float voltage);
    void setBusUnderVoltageLimit(float voltage);
    void setShuntOverVoltageLimit(float voltage);
    void setShuntUnderVoltageLimit(float voltage);
    void setPowerLimit(float watts);
    void setTemperatureLimit(float temperature);

    void setAlertInvertedPolarity(bool inverted);
    void setAlertLatch(bool latch);
    
    bool alertStatus(void);

    bool isMathOverflow(void);
    bool isAlert(void);

    double getBusPower(void);
    double getCharge(void);
    double getShuntCurrent(void);
    double getShuntVoltage(void);
    double getBusVoltage(void);
    double getTemperature(void);

    double getAmpHours(void);
    double getMilliampHours(void);

    float getMaxPossibleCurrent(void);
    float getMaxCurrent(void);
    float getMaxShuntVoltage(void);
    float getMaxPower(void);

    int getManID(void);

private:

	double currentLSB, powerLSB, energyLSB, chargeLSB;
        float vBusMax = 80;
        float vShuntMax = 0.050;
        float iMaxCurrentExpected = 500;
        float rShuntValue = vShuntMax / iMaxCurrentExpected;

        ina228_shunt_range _shunt = INA228_SHUNT_RANGE_163MV;
       
        neoI2C i2c;

        // I2C Address
        uint8_t I2CAddress; //User Defined

        float shnt_ul, shnt_ol, tmp_ol;
        float bus_ol, bus_ul, pwr_ol;

	void setMaskEnable(uint16_t mask);
	uint16_t getMaskEnable(void);

	void writeRegister16(uint8_t reg, uint16_t value);
        void writeInt16(uint8_t reg, int16_t value);
	uint16_t readRegister16(uint8_t reg);
        uint32_t readRegister24(uint8_t reg);
        uint64_t readRegister40(uint8_t reg);
        int32_t readInt24(uint8_t reg);
        int64_t readInt40(uint8_t reg);
};

#endif

// TEST SCRIPT
/*


//INA228 Sketch
// 1-19-23

// ******************    Includes     ******************
#include "neoUART_service.h"
#include "neoINA228.h"


// ****************** Initialize Hardware ******************
// --- UART Serial ---
neoUART_service bleSerial = neoUART_service();

// --- i2c Serial ---
neoI2C i2c = neoI2C();

// --- INA228 ---
neoINA228 ina228 = neoINA228(i2c);

// ******************      Setup      ******************
void setup(){

    pinMode(ARDUINO_13_PIN,INPUT_PULLUP);
    pinMode(LED_BUILTIN,OUTPUT);
    digitalWrite(LED_BUILTIN,LOW);

    //Serial.begin(115200);
    NRF_LOG_INFO("~~~ Testing INA228 ~~~");

    // begin i2c serial at 400k
    i2c.begin();
    
    //Alerts Set Values
    float shunt_ol = 0.163835; // Max +- 0.163835 V
    float shunt_ul = -0.163834; // Max +- 0.16384 V
    float bus_ol = 85; // Max 102.4 V
    float bus_ul = 0.0001; // Max 102.4 V
    float temp_ol = 125; // Max 255.9922 C
    float power_ol = 42500; // Max 51199.22 W

    // begin INA228
    bool checker = ina228.begin(0x40, shunt_ul, shunt_ol, bus_ol, bus_ul, temp_ol, power_ol); //add resistance
    // leave an example here in comments for sample rate / filters / range / etc
 
    if (checker == true) {
        NRF_LOG_INFO("i2c Connected!!")
    } else{
        NRF_LOG_INFO("i2c DIDN'T Connect");
    }

    ina228.resetCharge();

}

// ******************       Loop      ******************
void loop(){

    double my_current = ina228.getShuntCurrent();
    double my_s_voltage = ina228.getShuntVoltage();
    double my_voltage = ina228.getBusVoltage();
    double my_power = ina228.getBusPower();
    double my_Ah = ina228.getAmpHours();
    double my_mAh = ina228.getMilliampHours();

    uint32_t alert_pin = digitalRead(ARDUINO_13_PIN);
    NRF_LOG_INFO("", alert_pin);
    NRF_LOG_INFO("Alert Pin: %x", alert_pin);

    if (alert_pin == 0){
        bool my_errors = ina228.alertStatus();
        if (my_errors == true){
            NRF_LOG_INFO("You have errors");
        }
    }

    NRF_LOG_INFO("Current (A): %s", f2s(my_current,8));
    NRF_LOG_FLUSH();
    NRF_LOG_INFO("vshunt (V): %s", f2s(my_s_voltage,8));
    NRF_LOG_FLUSH();
    NRF_LOG_INFO("voltage (V): %s", f2s(my_voltage,8));
    NRF_LOG_FLUSH();
    NRF_LOG_INFO("Power (W): %s", f2s(my_power,8));
    NRF_LOG_FLUSH();
    NRF_LOG_INFO("Amp Hours (Ah): %s", f2s(my_Ah,8));
    NRF_LOG_FLUSH();
    NRF_LOG_INFO("Milliamp Hours (mAh): %s", f2s(my_mAh,8));
    NRF_LOG_FLUSH();

    
    digitalWrite(LED_BUILTIN,LOW);
    delay(1000);
    digitalWrite(LED_BUILTIN,HIGH);
    delay(1000);
}




*/