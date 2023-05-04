
#include "calib.h"

Calib::Calib() {
}

void Calib::begin() {
  _pref.begin("calib", false);
  get();
}

void Calib::get() {
  _espADCfactor = _pref.getFloat("espADCfactor", 0.00766); // 3.137/4096.0
  _espADCoffset = _pref.getFloat("espADCoffset", 0.1078);
  // XXX No calibration for ADS7828 for now
  _ADCfactor = 0.00080566; // 3.3/4096.0
  _ADCoffset = 0.0;
  // XXX No calibrartion for the DAC for now
  _DACfactor = 0.00080566; // 3.3/4096.0
  return;
}

void Calib::set(float espADCfactor, float espADCoffset) {
  _espADCfactor = espADCfactor;
  _espADCoffset = espADCoffset;
  _pref.putFloat("espADCfactor", _espADCfactor);
  _pref.putFloat("espADCoffset", _espADCoffset);
  return;
}

float Calib::espADCToVolts(uint16_t aval) {
  return _espADCoffset + (_espADCfactor * aval);
}

// Convert ADC reading to voltage
float Calib::ADCToVolts(uint16_t aval) {
  return _ADCoffset + (_ADCfactor * aval);
}

float Calib::DACToVolts(uint16_t aval) {
  return aval * _DACfactor;
}
