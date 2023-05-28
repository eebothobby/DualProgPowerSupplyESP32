#include <ADS7828.h>
#include <DAC7574.h>
#include <Preferences.h>

#define DAC7574_I2CADR 0  // A1, A0 pins
// DAC channels
#define VSETB 0
#define ISETB 1
#define ISETA 2
#define VSETA 3

#define ADS7828_I2CADR 3  // A1, A0 pins
// ADC channels
#define IMONB 0
#define TMONB 1
#define VSWB 2
#define VOUTB 3
#define VOUTA 4
#define VSWA 5
#define IMONA 6
#define TMONA 7

#define VINADC 33  // 1/10 of the inout voltage on GPIO33

class Calib {
   public:
    Calib();
    void begin();

    // print calibration data to serial port
    void print();

    // Read the calibrated values from the ADCs
    float readVsw(uint8_t chan); 
    float readVout(uint8_t chan);
    float readIout(uint8_t chan);
    float readTempC(uint8_t chan);
    float readVin();

    // Read the raw values from the ADCs
    uint16_t rawVsw(uint8_t chan);
    uint16_t rawVout(uint8_t chan);
    uint16_t rawIout(uint8_t chan);
    uint16_t rawTempC(uint8_t chan);
    uint16_t rawVin();
    
    // Set the DAC values, return the equivalent calibrated result
    // If rel is true, then add val to existing value
    // If rel is false, set value to val
    float setVdac(uint8_t chan, int32_t val, bool rel);
    float setIdac(uint8_t chan, int32_t val, bool rel);

    // Get the value represented by val in the DAC
    float getVdac(uint8_t chan, uint16_t val);
    float getIdac(uint8_t chan, uint16_t val);

    // get the closest DAC value for volts or amps
    uint16_t getVdacVolts(uint8_t chan, float volts);
    uint16_t getIdacAmps(uint8_t chan, float amps);

    // Get the raw values of the DACs
    uint16_t rawVdac(uint8_t chan);
    uint16_t rawIdac(uint8_t chan);

    // set the ADC calibration values
    void adcCalibVsw(uint8_t chan, float factor, float offset);
    void adcCalibVout(uint8_t chan, float factor, float offset);
    void adcCalibIout(uint8_t chan, float factor, float offset);
    void adcCalibTempC(uint8_t chan, float factor, float offset);
    void adcCalibVin(float factor, float offset);

    // set dac calibration values
    void dacCalibV(uint8_t chan, float factor, float offset);
    void dacCalibI(uint8_t chan, float factor, float offset);

   private:
    Preferences _pref;
    DAC7574 _dac;
    const uint8_t _dacvset[2] = {VSETA, VSETB};
    const uint8_t _daciset[2] = {ISETA, ISETB};
    // DAC values
    uint16_t _vsetval[2] = {0, 0};
    uint16_t _isetval[2] = {0, 0};
    ADS7828 _adc;
    const uint8_t _adcimon[2] = {IMONA, IMONB};
    const uint8_t _adctmon[2] = {TMONA, TMONB};
    const uint8_t _adcvsw[2] = {VSWA, VSWB};
    const uint8_t _adcvout[2] = {VOUTA, VOUTB};
    float _espADCfactor, _espADCoffset;
    float _ADCfactor[8], _ADCoffset[8];
    float _DACfactor[4], _DACoffset[4];
    void _load();  // Load the calibration values
    void _adcCalib(uint8_t ch, float factor, float offset); // common ADC calib
    void _dacCalib(uint8_t ch, float factor, float offset); // common DAC calib
    uint16_t _incr12(uint16_t cur, int32_t incr);
};
