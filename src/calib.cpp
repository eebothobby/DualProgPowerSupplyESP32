
   /*
   Copyright (c) 2023 Ashok Singhal.

   Software License Agreement (BSD License)

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
   2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
   3. Neither the name of the copyright holders nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
   EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "calib.h"

Calib::Calib() {}

void Calib::begin() {
    _pref.begin("calib", false);
    // _pref.clear();
    _dac.begin(DAC7574_I2CADROF);
    _adc.begin(ADS7828_I2CADROF);
    _load();
    // Set the DAC values to 0
    for (uint8_t chan = 0; chan < 2; chan++) {
        setVdac(chan, 0, false);
        setIdac(chan, 0, false);
    }
    print();
}

void Calib::print() {
    // Print out the calibration factor and offset
    Serial.println("ADC Calib");
    Serial.print("esp: "); Serial.print(_espADCfactor, 7);
    Serial.print(" "); Serial.println(_espADCoffset, 7);

    for (uint8_t ch = 0; ch < 2; ch++) {
        Serial.print(" Ch: "); Serial.println(ch);
        Serial.print(" Vout: "); Serial.print(_ADCfactor[_adcvout[ch]], 7); 
        Serial.print( " "); Serial.println(_ADCoffset[_adcvout[ch]], 7);
        Serial.print(" Vsw: "); Serial.print(_ADCfactor[_adcvsw[ch]], 7); 
        Serial.print( " "); Serial.println(_ADCoffset[_adcvsw[ch]], 7);
        Serial.print(" Iout: "); Serial.print(_ADCfactor[_adcimon[ch]], 7); 
        Serial.print( " "); Serial.println(_ADCoffset[_adcimon[ch]], 7);
        Serial.print(" TempC: "); Serial.print(_ADCfactor[_adctmon[ch]], 7); 
        Serial.print( " "); Serial.println(_ADCoffset[_adctmon[ch]], 7);
    }

    Serial.println("DAC Calib");
    for (uint8_t ch = 0; ch < 2; ch++) {
        Serial.print(" Ch: "); Serial.println(ch);
        Serial.print(" Vset: "); Serial.print(_DACfactor[_dacvset[ch]], 7); 
        Serial.print( " "); Serial.println(_DACoffset[_dacvset[ch]], 7);
        Serial.print(" Iset: "); Serial.print(_DACfactor[_daciset[ch]], 7); 
        Serial.print( " "); Serial.println(_DACoffset[_daciset[ch]], 7);
    }
}

void Calib::_load() {
    float defFact[8], defOff[8]; // default values for factor and offset

    // Load the ESP calibration values, default values empirically determined
    _espADCfactor = _pref.getFloat("espADCf", 0.00825);
    _espADCoffset = _pref.getFloat("espADCo", 1.37);

    // calculate the default values for ADC calibs
    for (uint8_t ch = 0; ch < 2; ch++) {
        defFact[_adcvout[ch]] = 0.0080566; // 10 * 3.3 / 4096.0
        defOff[_adcvout[ch]] = 0.0;
        defFact[_adcvsw[ch]] = 0.0080566; // 10 * 3.3 / 4096.0
        defOff[_adcvsw[ch]] = 0.0;
        defFact[_adcimon[ch]] = 0.000757; // 0.94 * 3.3/4096.0
        defOff[_adcimon[ch]] = 0.0;
        defFact[_adctmon[ch]] = 0.80566; // 1000 * 3.3/4096.0
        defOff[_adctmon[ch]] = 0.0;
    }

    // Load ADC calibration
    for (uint8_t i = 0; i < 8; i++) {
        char buf[8];
        sprintf(buf, "ADCf_%d", i);
        _ADCfactor[i] = _pref.getFloat(buf, defFact[i]);
        sprintf(buf, "ADCo_%d", i);
        _ADCoffset[i] = _pref.getFloat(buf, defOff[i]);
    }

    // calculate the default values for DAC calibs
    for (uint8_t ch = 0; ch < 2; ch++) {
        defFact[_dacvset[ch]] = 0.0080566; // 10 * 3.3 / 4096.0
        defOff[_dacvset[ch]] = 0.0;
        defFact[_daciset[ch]] = 0.000757; // 0.94 * 3.3/4096.0
        defOff[_daciset[ch]] = 0.0;
    }

    // load the DAC calibration
    for (uint8_t i = 0; i < 4; i++) {
        char buf[8];
        sprintf(buf, "DACf_%d", i);
        _DACfactor[i] = _pref.getFloat(buf, defFact[i]);
        sprintf(buf, "DACo_%d", i);
        _DACoffset[i] = _pref.getFloat(buf, defOff[i]);
    }
    return;
}

void Calib::adcCalibVin(float factor, float offset) {
    _espADCfactor = factor;
    _espADCoffset = offset;
    _pref.putFloat("espADCf", _espADCfactor);
    _pref.putFloat("espADCo", _espADCoffset);
    return;
}

// Common code for 8-port ADC calibratrion
// chan = 0..7
void Calib::_adcCalib(uint8_t ch, float factor, float offset) {
    char buf[8];
    if (ch > 7) {
        return;
    }
    _ADCfactor[ch] = factor;
    _ADCoffset[ch] = offset;
    
    sprintf(buf, "ADCf_%d", ch);
    _pref.putFloat(buf, factor);
    sprintf(buf, "ADCo_%d", ch);
    _pref.putFloat(buf, offset);
    return;
}

void Calib::adcCalibVsw(uint8_t chan, float factor, float offset) {
    if (chan > 1) {
        return;
    }
    _adcCalib(_adcvsw[chan], factor, offset);
    return;
}

void Calib::adcCalibVout(uint8_t chan, float factor, float offset) {
    if (chan > 1) {
        return;
    }
    _adcCalib(_adcvout[chan], factor, offset);
    return;
}

void Calib::adcCalibIout(uint8_t chan, float factor, float offset) {
    if (chan > 1) {
        return;
    }
    _adcCalib(_adcimon[chan], factor, offset);
    return;
}

void Calib::adcCalibTempC(uint8_t chan, float factor, float offset) {
    if (chan > 1) {
        return;
    }
    _adcCalib(_adctmon[chan], factor, offset);
    return;
}

// Common code for 4-port DAC calibratrion
// chan = 0..3
void Calib::_dacCalib(uint8_t ch, float factor, float offset) {
    char buf[8];
    if (ch > 3) {
        return;
    }
    _DACfactor[ch] = factor;
    _DACoffset[ch] = offset;
    
    sprintf(buf, "DACf_%d", ch);
    _pref.putFloat(buf, factor);
    sprintf(buf, "DACo_%d", ch);
    _pref.putFloat(buf, offset);
    return;
}

void Calib::dacCalibV(uint8_t chan, float factor, float offset) {
    if (chan > 1) {
        return;
    }
    _dacCalib(_dacvset[chan], factor, offset);
    return;
}

void Calib::dacCalibI(uint8_t chan, float factor, float offset) {
    if (chan > 1) {
        return;
    }
    _dacCalib(_daciset[chan], factor, offset);
    return;
}

float Calib::readVsw(uint8_t chan) {
    uint8_t ch;
    if (chan > 1) {
        return 0.0;
    }
    ch = _adcvsw[chan];
    return _adc.read(ch) * _ADCfactor[ch] + _ADCoffset[ch];
}

float Calib::readVout(uint8_t chan) {
    uint8_t ch;
    if (chan > 1) {
        return 0.0;
    }
    ch = _adcvout[chan];
    return _adc.read(ch) * _ADCfactor[ch] + _ADCoffset[ch];
}

float Calib::readIout(uint8_t chan) {
    uint8_t ch;
    if (chan > 1) {
        return 0.0;
    }
    ch = _adcimon[chan];
    return _adc.read(ch) * _ADCfactor[ch] + _ADCoffset[ch];
}

float Calib::readTempC(uint8_t chan) {
    uint8_t ch;
    if (chan > 1) {
        return 0.0;
    }
    ch = _adctmon[chan];
    return _adc.read(ch) * _ADCfactor[ch] + _ADCoffset[ch];
}

float Calib::readVin() {
    return analogRead(VINADC) * _espADCfactor + _espADCoffset;
}

uint16_t Calib::rawVsw(uint8_t chan) {
    uint8_t ch;
    if (chan > 1) {
        return 0.0;
    }
    ch = _adcvsw[chan];
    return _adc.read(ch);
}

uint16_t Calib::rawVout(uint8_t chan) {
    uint8_t ch;
    if (chan > 1) {
        return 0.0;
    }
    ch = _adcvout[chan];
    return _adc.read(ch);
}

uint16_t Calib::rawIout(uint8_t chan) {
    uint8_t ch;
    if (chan > 1) {
        return 0.0;
    }
    ch = _adcimon[chan];
    return _adc.read(ch);
}

uint16_t Calib::rawTempC(uint8_t chan) {
    uint8_t ch;
    if (chan > 1) {
        return 0.0;
    }
    ch = _adctmon[chan];
    return _adc.read(ch);
}

uint16_t Calib::rawVin() {
    return analogRead(VINADC);
}

// Return new value for 12-bit cur incremented by incr
// Limit the return value to 0..4095 (2^12 - 1)
uint16_t Calib::_incr12(uint16_t cur, int32_t incr) {
    uint16_t rval = cur + incr;
    if (incr > 0 && rval > 4095) {
        return 4095;
    } else if (incr < 0 && rval > cur) {
        // wrapped
        return 0;
    }
    return rval;
}

float Calib::setVdac(uint8_t chan, int32_t val, bool rel) {
    uint8_t ch;
    if (chan > 1) {
        return 0.0;
    }
    ch = _dacvset[chan];
    _vsetval[chan] = _incr12((rel? _vsetval[chan]: 0), val);
    _dac.setData(_vsetval[chan], ch);
    return _vsetval[chan] * _DACfactor[ch] + _DACoffset[ch];
}

float Calib::setIdac(uint8_t chan, int32_t val, bool rel) {
    uint8_t ch;
    if (chan > 1) {
        return 0.0;
    }
    ch = _daciset[chan];
    _isetval[chan] = _incr12((rel? _isetval[chan]: 0), val);
    _dac.setData(_isetval[chan], ch);
    return _isetval[chan] * _DACfactor[ch] + _DACoffset[ch];
}

float Calib::getVdac(uint8_t chan, uint16_t val) {
    uint8_t ch;
    if (chan > 1) {
        return 0.0;
    }
    ch = _dacvset[chan];
    return _incr12(val, 0) * _DACfactor[ch] + _DACoffset[ch];
}

float Calib::getIdac(uint8_t chan, uint16_t val) {
    uint8_t ch;
    if (chan > 1) {
        return 0.0;
    }
    ch = _daciset[chan];
    return _incr12(val, 0) * _DACfactor[ch] + _DACoffset[ch];
}

 uint16_t Calib::getVdacVolts(uint8_t chan, float volts) {
    float tval;
    uint8_t ch;

    if (chan > 1) {
        return 0;
    }
    ch = _dacvset[chan];

    tval = (volts - _DACoffset[ch])/ _DACfactor[ch];
    if (tval < 0.0) {
        return 0;
    }
    if (tval > 4095.0) {
        return 4095;
    }
    return  (uint16_t)(round(tval));
 }

 uint16_t Calib::getIdacAmps(uint8_t chan, float amps) {
    float tval;
    uint8_t ch;

    if (chan > 1) {
        return 0;
    }
    ch = _daciset[chan];
    tval = (amps - _DACoffset[ch])/ _DACfactor[ch];
    if (tval < 0.0) {
        return 0;
    }
    if (tval > 4095.0) {
        return 4095;
    }
    return  (uint16_t)(round(tval));
 }

uint16_t Calib::rawVdac(uint8_t chan) {
    if (chan > 1) {
        return 0;
    }
    return _vsetval[chan];
}

uint16_t Calib::rawIdac(uint8_t chan) {
    if (chan > 1) {
        return 0;
    }
    return _isetval[chan];
}
