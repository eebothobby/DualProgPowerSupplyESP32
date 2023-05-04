#include <Preferences.h>

class Calib {
    public:
      Calib();
      void begin();
      void get();
      void set(float espADCfactor, float espADCoffset);
      float espADCToVolts(uint16_t aval);
      float ADCToVolts(uint16_t aval);
      float DACToVolts(uint16_t aval);
    private:
      Preferences _pref;
      float _espADCfactor, _espADCoffset;
      float _ADCfactor, _ADCoffset;
      float _DACfactor;

};
