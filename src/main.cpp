/*
   Dual channel programmable power supply using
     * TTGO TDISPLAY standard module with encoder (esp32_tdisp_enc).
     * Buck-Boost converter modules with LT3081 LDO
     * DAC7574 to generate voltage and current settings
     * ADS7828 to read various voltage levels.
   Wifi is not used so wifi code is not included.

   The controls are:
   Button0: On/Off
   Button1: Selects display mode
   Encoder button: Selects Voltage/Current limit settings
   

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
#include <Arduino.h>
#include <TFT_eSPI.h> // Hardware-specific library
#include <Encoder.h>
#include <Bounce2.h>
#include <DAC7574.h>
#include <ADS7828.h>
#include "calib.h"

DAC7574 dac;
#define DAC7574_I2CADR 0 // A1, A0 pins
// DAC channels
#define VSETB 0
#define ISETB 1
#define ISETA 2
#define VSETA 3
const uint8_t dacvset[2] = {VSETA, VSETB};
const uint8_t daciset[2] = {ISETA, ISETB};

ADS7828 adc;
#define ADS7828_I2CADR 3 // A1, A0 pins
// ADC channels
#define IMONB 0
#define TMONB 1
#define VSWB  2
#define VOUTB 3
#define VOUTA 4
#define VSWA  5
#define IMONA 6
#define TMONA 7
const uint8_t adcimon[2] = {IMONA, IMONB};
const uint8_t adctmon[2] = {TMONA, TMONB};
const uint8_t adcvsw[2] = {VSWA, VSWB};
const uint8_t adcvout[2] = {VOUTA, VOUTB};

TFT_eSPI tft = TFT_eSPI();       // Invoke custom library

Calib calib;

// flag to indicate that the calibration values have been updated in preferences and need to be saved
bool calibupdate = false; 

#define VINADC 33 // 1/10 of the inout voltage on GPIO33

#define COUNTMAX 50
int countadc = 0; // Count of number of adc measurements summed
// 0..7 are on ADS7828, 8 is from esp32 pin33 adc meausurement
uint32_t adcsum[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; // Sum of raw adc measurements
uint16_t adcaval[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; // Average over count of raw adc measurements

// Final calculated outputs
float vsw[2] = {0.0, 0.0};
float vout[2] = {0.0, 0.0};
float iout[2] = {0.0, 0.0};
float tempC[2] = {0.0, 0.0}; // Temperature in deg C
float vin = 0.0;

// DAC values
uint16_t vsetval[2] = {0, 0};
uint16_t isetval[2] = {0, 0};

// Corresponding calculated setting values
float vsetout[2] = {0.0, 0.0};
float isetout[2] = {0.0, 0.0};

#define ENC_CLK 13
// Can only use pin 12 with internal pullup, since it must not be pulled up on boot.
//  Also avoid using pins with LEDs attached
#define ENC_DT 12
#define ENC_SW 15

// These two buttons are on the T-DISPLAY board on GPIO0, GPIO35
#define TT_BUT0 0
#define TT_BUT1 35
Encoder enc(ENC_CLK, ENC_DT);
Bounce enc_but = Bounce(); // Button in the encoder
Bounce but0 = Bounce();
Bounce but1 = Bounce();

long oldPosition  = 0; // Keeps track of encoder's previous postion.
unsigned smode = 0; // Encoder setting mode
                    // dmode0 onluy 0: voltA, 1: voltB, 2: ilimA, 3: ilimB

unsigned dmode = 0; // Diag/debug mode.  0: Normal, 1: debug
int enpin[2] = {2, 17}; // ENA, ENB
int enable[2] = {1, 1}; // Enable power to each channel
int climpin[2] = {27, 32}; // CLIMA, CLIMB active low to indicate current limit
bool clim[2] = {false, false};

float powOutW[2] = {0.0, 0.0}; // Output power Watts
float ldoDissW[2] = {0.0, 0.0}; // LDO power dissipation

unsigned long ptimeus = 0;

// Read the clim status for both channels
// Return: false: no change (no need to update display), true: there has been a change
bool readClims() {
  bool disp = false;
  // Only display if clim values change.
  bool prevclim;
  for (uint8_t chan = 0; chan < 2; chan++) {
    prevclim = clim[chan];
    clim[chan] = (digitalRead(climpin[chan]) == 0) ? true: false;
    if (prevclim != clim[chan]) {
      disp = true;
    }
  }
  return disp;
}

// Read the ADCs and sum them, calculate the output values
// Return false: display need not be updated, true: display should be updated
bool readAvolts() {
  bool disp = false;

  for (int i = 0; i < 8; i++) {
    adcsum[i] = adcsum[i] + adc.read(i);
  }
  adcsum[8] = adcsum[8] + analogRead(VINADC);
  countadc++;
  if (countadc >= COUNTMAX) {
    for (int i = 0; i < 9; i++) {
      adcaval[i] = adcsum[i]/countadc;
    }

    for (uint8_t chan = 0; chan < 2; chan++)
    {
      // factor of 10.0 comes from external voltage divider
      vout[chan] = 10.0 * calib.ADCToVolts(adcaval[adcvout[chan]]);
      vsw[chan] = 10.0 * calib.ADCToVolts(adcaval[adcvsw[chan]]);
      // Imon = Iout/5000
      // Old board R = 10K, new board R = 4.7K
      // voltage = Imon * R = Iout/5000 * 4700
      // Iout amps = voltage *4700/5000 = voltage * 0.94
      iout[chan] = 0.94 * calib.ADCToVolts(adcaval[adcimon[chan]]);
      // tempmon current is 1uA/C across 1k res = 1mV/C (old value was 10K resistor)
      tempC[chan] = 1000.0 * calib.ADCToVolts(adcaval[adctmon[chan]]);
      // Compute powers
      powOutW[chan] = vout[chan] * iout[chan];
      ldoDissW[chan] = (vsw[chan] - vout[chan]) * iout[chan];
    }
    vin = 10.0 * calib.espADCToVolts(adcaval[8]);

    countadc = 0;
    for(int i = 0; i < 9; i++) {
      adcsum[i] = 0.0;
    }
    disp = true;
  }
  return disp;
}

// Each row of text is 20 pixels apart on Y axis, there are 7 rows (0..6)
// Each char is 12 pixels wide on X axis, there are 21 chars across (0..20)
// Button1 shown on end of row 0, Button0 on end of row 6.

// line dmode: 0   
// 0 Vo VoutB    VoutA <BUT1>  
// 1 Ao IoutB    IoutA
// 2 Vv VsetB    VsetA
// 3 Al ILimB    IlimA Lb La
// 4 Wo PowB     PowA        
// 5 Vs VswB     VswA   Vin    
// 6 C TempB     TempA <BUT0>

void displayMode0() {
  char buf[22];
  int line = 0;
  tft.setTextColor(TFT_WHITE);
  sprintf(buf, "Vo %5.2f %5.2f", vout[1], vout[0]);
  tft.drawString(buf, 0, line*20);
  
  line = 1;
  sprintf(buf, "Ao %5.3f %5.3f", iout[1], iout[0]);
  tft.drawString(buf, 0, line*20);
  
  line = 2;
  tft.drawString("Vv ", 0, line*20);
  tft.setTextColor(smode == 1? TFT_GREEN : TFT_WHITE);
  sprintf(buf, "%5.2f ", vsetout[1]);
  tft.drawString(buf, 12*3, line*20);
  tft.setTextColor(smode == 0? TFT_GREEN : TFT_WHITE);
  sprintf(buf, "%5.2f", vsetout[0]);
  tft.drawString(buf, 12*9, line*20);
  
  line = 3;
  tft.setTextColor(TFT_WHITE);
  tft.drawString("Al ", 0, line*20);
  tft.setTextColor(smode == 3? TFT_GREEN : TFT_WHITE);
  sprintf(buf, "%5.3f", isetout[1]);
  tft.drawString(buf, 12*3, line*20);
  tft.setTextColor(smode == 2? TFT_GREEN : TFT_WHITE);
  sprintf(buf, "%5.3f", isetout[0]);
  tft.drawString(buf, 12*9, line*20);
  // Only draw current limit channel in red if it is true.
  if (clim[1]) {
    tft.setTextColor(TFT_RED);
    tft.drawString("B", 12*15, line*20);
  }
  if (clim[0]) {
    tft.setTextColor(TFT_RED);
    tft.drawString("A", 12*17, line*20);
  }
  
  line = 4;
  tft.setTextColor(TFT_WHITE);
  sprintf(buf, "Wo %5.2f %5.2f", powOutW[1], powOutW[0]);
  tft.drawString(buf, 0, line*20);

  line = 5;
  sprintf(buf, "Vs %5.2f %5.2f %5.2f", vsw[1], vsw[0], vin);
  tft.drawString(buf, 0, line*20);

  line = 6;
  sprintf(buf, "TC %5.1f %5.1f", tempC[1], tempC[0]);
  tft.drawString(buf, 0, line*20);
}

void displayInfo() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setCursor(0, 0);
  tft.setTextColor(TFT_WHITE);
  tft.setTextWrap(true);
  if (dmode == 0) {
    displayMode0();
  }
  
  // Draw identifiers for buttons
  tft.setTextColor(TFT_BLUE);
  tft.drawString("DMOD>", 12 * 15, 0);
  if (enable[0]) {
    tft.setTextColor(TFT_GREEN);
    tft.drawString("OFF >", 12 * 15, 6 * 20);
  } else {
    tft.setTextColor(TFT_RED);
    tft.drawString("ON  >", 12 * 15, 6 * 20);
  }
}

void setup(void)
{
  uint8_t chan;
  Serial.begin(115200);
  for (chan = 0; chan < 2; chan++) {
    pinMode(climpin[chan], INPUT_PULLUP);
    pinMode(enpin[chan], OUTPUT);
  }

  Wire.begin();
  Wire.setClock(400000L);
  dac.begin(DAC7574_I2CADR);
  adc.begin(ADS7828_I2CADR);
  
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  // BOUNCE SETUP
  // SELECT ONE OF THE FOLLOWING :
  // 1) IF YOUR INPUT HAS AN INTERNAL PULL-UP
  // bounce.attach( BOUNCE_PIN ,  INPUT_PULLUP ); // USE INTERNAL PULL-UP
  // 2) IF YOUR INPUT USES AN EXTERNAL PULL-UP
  enc_but.attach( ENC_SW, INPUT_PULLUP ); // USE INTERNAL PULL-UP
  but0.attach(TT_BUT0, INPUT_PULLUP);
  but1.attach(TT_BUT1, INPUT_PULLUP);

  // DEBOUNCE INTERVAL IN MILLISECONDS
  enc_but.interval(5); // interval in ms
  but0.interval(5);
  but1.interval(5);

  calib.begin();

  for (chan = 0; chan < 2; chan++) {
    dac.setData(vsetval[chan], dacvset[chan]);
    dac.setData(isetval[chan], daciset[chan]);
    digitalWrite(enpin[chan], enable[chan] ? HIGH : LOW);
  }

  displayInfo();
  ptimeus = micros();
}

// Return new value for 12-bit cur incremented by incr
// Limit the return value to 0..4095 (2^12 - 1)
uint16_t incr12(uint16_t cur, int32_t incr) {
  uint16_t rval = cur + incr;
  if (incr > 0 && rval > 4095) {
    return 4095;
  } else if (incr < 0 && rval > cur) {
    // wrapped
    return 0;
  }
  return rval;
}

void loop(void)
{
  bool disp = false; // update display if true
  uint8_t chan;
  unsigned long ctimeus = micros();
  unsigned long deltaus;
  
  // XXX but0 controls both enables together for now
  but0.update();
  if (but0.fell()) {
    enable[0] = 1 - enable[0]; // Toggles enable between 0 and 1
    enable[1] = enable[0];
    digitalWrite(enpin[0], enable[0]? HIGH : LOW);
    digitalWrite(enpin[1], enable[1]? HIGH : LOW);
    disp = true;
  }

  // but1 changes display mode
  but1.update();
  if (but1.fell()) {
    dmode = 1 - dmode; // Toggles dmode between 0 and 1
    // If we came out of debug mode check if we need to save calibration
    // and reset smode to 0
    if (!dmode) {
      smode = 0;
      if (calibupdate) {
        // calib.set();
        calibupdate = false;
      }
    }
    disp = true;
  }

  // enc_but controls smode
  enc_but.update();
  if (enc_but.fell()) {
    smode = (smode + 1) % 4;
    disp = true;
  }

  long newPosition = enc.read() / 4;
  if (newPosition != oldPosition) {
    int32_t incr = newPosition - oldPosition;
    // Speed up increment if the user is turning the encoder knob fast.
    // If the increment is > 1, multiply increment by 16
    deltaus = ctimeus - ptimeus;
    if (abs(incr) > 1) {
      incr *= 16;
      Serial.print("incr>1 ");
    }
    // If the increment happens faster than 40ms, multiply increment by 8
    // or if faster than 60ms multiple by 4
    if (deltaus < 40000) {
      incr *= 8;
      Serial.print("us<40k ");
    } else if (deltaus < 60000) {
      incr *= 4;
      Serial.print("us<60k ");
    }
    Serial.println(incr);
    ptimeus = ctimeus;
    
    oldPosition = newPosition;
    if (smode == 0 || smode == 1) {
      chan = smode;
      vsetval[chan] = incr12(vsetval[chan], incr);
      dac.setData(vsetval[chan], dacvset[chan]);
      vsetout[chan] = calib.DACToVolts(vsetval[chan]) * 10.0;
    } else if (smode == 2 || smode == 3) {
      chan = smode - 2;
      isetval[chan] = incr12(isetval[chan], incr);
      dac.setData(isetval[chan], daciset[chan]);
      isetout[chan] = calib.DACToVolts(isetval[chan]) * 0.94; // new board R on imon = 4.7K
    }
    disp = true;
  }

  disp = readClims()? true : disp;
  disp = readAvolts()? true : disp;
  if (disp) {
    displayInfo();
  } 
}
