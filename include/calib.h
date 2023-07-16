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

#include <ADS7828.h>
#include <DAC7574.h>
#include <Preferences.h>

#define DAC7574_I2CADROF 0  // A1, A0 pins
// DAC channels
#define VSETB 0
#define ISETB 1
#define ISETA 2
#define VSETA 3

#define ADS7828_I2CADROF 3  // A1, A0 pins
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
