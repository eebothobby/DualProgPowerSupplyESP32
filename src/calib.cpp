
#include "calib.h"

Calib::Calib() {}

void Calib::begin() {
    _pref.begin("calib", false);
    // _pref.clear();
    get();
}

void Calib::get() {
    _espADCfactor = _pref.getFloat("espADCf", 0.000766);  // 3.137/4096.0
    _espADCoffset = _pref.getFloat("espADCo", 0.1078);

    for (int i = 0; i < 8; i++) {
        char buf[8];
        sprintf(buf, "ADCf_%d", i);
        _ADCfactor[i] = _pref.getFloat(buf, 0.00080566);  // 3.3/4096.0
    }

    // Not sure we need an offset
    _ADCoffset = 0.0;
    // XXX No calibrartion for the DAC for now
    _DACfactor = 0.00080566;  // 3.3/4096.0
    return;
}

void Calib::setESP(float espADCfactor, float espADCoffset) {
    _espADCfactor = espADCfactor;
    _espADCoffset = espADCoffset;
    _pref.putFloat("espADCf", _espADCfactor);
    _pref.putFloat("espADCo", _espADCoffset);
    return;
}

void Calib::setADCf(float ADCfactor, uint8_t ch) {
    char buf[8];
    sprintf(buf, "ADCf_%d", ch);
    _ADCfactor[ch] = ADCfactor;

    _pref.putFloat(buf, ADCfactor);
    return;
}

float Calib::espADCToVolts(uint16_t aval) {
    return _espADCoffset + (_espADCfactor * aval);
}

// Convert ADC reading to voltage
float Calib::ADCToVolts(uint16_t aval, uint8_t ch) {
    return _ADCoffset + (_ADCfactor[ch] * aval);
}

float Calib::DACToVolts(uint16_t aval) { return aval * _DACfactor; }
