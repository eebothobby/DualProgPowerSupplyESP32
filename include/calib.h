#include <Preferences.h>

class Calib {
   public:
    Calib();
    void begin();
    // Load the calibration values
    void get();
    // Set the ESP32 ADC calibration values
    void setESP(float espADCfactor, float espADCoffset);
    // Set the ADS7828 calibtation values for channel ch
    void setADCf(float ADCfactor, uint8_t ch);
    // return the esp32 voltage for the analog reading aval
    float espADCToVolts(uint16_t aval);
    // return the ADS7828 voltage for the analog reading aval for channel ch
    float ADCToVolts(uint16_t aval, uint8_t ch);
    // Return the expected voltage at the DAC for the value aval
    float DACToVolts(uint16_t aval);

   private:
    Preferences _pref;
    float _espADCfactor, _espADCoffset;
    float _ADCfactor[8], _ADCoffset;
    float _DACfactor;
};
