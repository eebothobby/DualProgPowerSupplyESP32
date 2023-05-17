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
   Encoder button: Selects elements within screen

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
#include <Arduino.h>
#include <Bounce2.h>
#include <DAC7574.h>
#include <Encoder.h>
#include <TFT_eSPI.h>  // Hardware-specific library

#include "calib.h"
#include "preset.h"
#include "wificon.h"

DAC7574 dac;
#define DAC7574_I2CADR 0  // A1, A0 pins
// DAC channels
#define VSETB 0
#define ISETB 1
#define ISETA 2
#define VSETA 3
const uint8_t dacvset[2] = {VSETA, VSETB};
const uint8_t daciset[2] = {ISETA, ISETB};

ADS7828 adc;
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
const uint8_t adcimon[2] = {IMONA, IMONB};
const uint8_t adctmon[2] = {TMONA, TMONB};
const uint8_t adcvsw[2] = {VSWA, VSWB};
const uint8_t adcvout[2] = {VOUTA, VOUTB};

TFT_eSPI tft = TFT_eSPI();  // Invoke custom library

Calib calib;
Preset preset;
Wificon wificon;

// flag to indicate that the calibration values have been updated in preferences
// and need to be saved
bool calibupdate = false;

#define VINADC 33  // 1/10 of the inout voltage on GPIO33

#define COUNTMAX 20  // Max number of loops without a display
int loopcount = 0;   // Number of loops without a display

// Final calculated outputs
float vsw[2] = {0.0, 0.0};
float vout[2] = {0.0, 0.0};
float iout[2] = {0.0, 0.0};
float tempC[2] = {0.0, 0.0};  // Temperature in deg C
float vin = 0.0;

// DAC values
uint16_t vsetval[2] = {0, 0};
uint16_t isetval[2] = {0, 0};

// Corresponding calculated setting values
float vsetout[2] = {0.0, 0.0};
float isetout[2] = {0.0, 0.0};

#define ENC_CLK 13
// Can only use pin 12 with internal pullup, since it must not be pulled up on
// boot.
//  Also avoid using pins with LEDs attached
#define ENC_DT 12
#define ENC_SW 15

// These two buttons are on the T-DISPLAY board on GPIO0, GPIO35
#define TT_BUT0 0
#define TT_BUT1 35
Encoder enc(ENC_CLK, ENC_DT);
Bounce enc_but = Bounce();  // Button in the encoder
Bounce but0 = Bounce();
Bounce but1 = Bounce();

long oldPosition = 0;  // Keeps track of encoder's previous postion.
unsigned smode = 0;    // Encoder setting mode
                       // dmode0 onluy 0: voltA, 1: voltB, 2: ilimA, 3: ilimB

unsigned dmode = 0;  // Display mode.  0: Normal, 1: Preset, 2: Calib

uint8_t pract = 0;  // Preset action. 0: None, 1: Use 2: Save

int enpin[2] = {2, 17};     // ENA, ENB
int enable[2] = {1, 1};     // Enable power to each channel
int climpin[2] = {27, 32};  // CLIMA, CLIMB active low to indicate current limit
bool clim[2] = {false, false};

float powOutW[2] = {0.0, 0.0};   // Output power Watts
float ldoDissW[2] = {0.0, 0.0};  // LDO power dissipation

unsigned long ptimeus = 0;

// Read the clim status for both channels
// Return: false: no change (no need to update display), true: there has been a
// change
bool readClims() {
    bool disp = false;
    // Only display if clim values change.
    bool prevclim;
    for (uint8_t chan = 0; chan < 2; chan++) {
        prevclim = clim[chan];
        clim[chan] = (digitalRead(climpin[chan]) == 0) ? true : false;
        if (prevclim != clim[chan]) {
            disp = true;
        }
    }
    return disp;
}

// Simple low pass filter returns
//    alpha*newv + (1 - alpha)*curv
// which can bre re-written as
//    curv + alpha * (newv - curv)
float lowPass(float curv, float newv, float alpha) {
    return curv + alpha * (newv - curv);
}

void readAvolts() {
    float alpha = 0.1;
    float newv;
    for (uint8_t chan = 0; chan < 2; chan++) {
        newv = 10.0 * calib.ADCToVolts(adc.read(adcvout[chan]), adcvout[chan]);
        vout[chan] = lowPass(vout[chan], newv, alpha);
        newv = 10.0 * calib.ADCToVolts(adc.read(adcvsw[chan]), adcvsw[chan]);
        vsw[chan] = lowPass(vsw[chan], newv, alpha);
        newv = 0.94 * calib.ADCToVolts(adc.read(adcimon[chan]), adcimon[chan]);
        iout[chan] = lowPass(iout[chan], newv, alpha);
        newv =
            1000.0 * calib.ADCToVolts(adc.read(adctmon[chan]), adctmon[chan]);
        tempC[chan] = lowPass(tempC[chan], newv, alpha);
        // Compute powers
        powOutW[chan] = vout[chan] * iout[chan];
        ldoDissW[chan] = (vsw[chan] - vout[chan]) * iout[chan];
    }
    newv = 10.0 * calib.espADCToVolts(analogRead(VINADC));
    vin = lowPass(vin, newv, alpha);
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
    tft.drawString(buf, 0, line * 20);

    line = 1;
    sprintf(buf, "Ao %5.3f %5.3f", iout[1], iout[0]);
    tft.drawString(buf, 0, line * 20);

    line = 2;
    tft.drawString("Vv ", 0, line * 20);
    tft.setTextColor(smode == 1 ? TFT_GREEN : TFT_WHITE);
    sprintf(buf, "%5.2f ", vsetout[1]);
    tft.drawString(buf, 12 * 3, line * 20);
    tft.setTextColor(smode == 0 ? TFT_GREEN : TFT_WHITE);
    sprintf(buf, "%5.2f", vsetout[0]);
    tft.drawString(buf, 12 * 9, line * 20);

    line = 3;
    tft.setTextColor(TFT_WHITE);
    tft.drawString("Al ", 0, line * 20);
    tft.setTextColor(smode == 3 ? TFT_GREEN : TFT_WHITE);
    sprintf(buf, "%5.3f", isetout[1]);
    tft.drawString(buf, 12 * 3, line * 20);
    tft.setTextColor(smode == 2 ? TFT_GREEN : TFT_WHITE);
    sprintf(buf, "%5.3f", isetout[0]);
    tft.drawString(buf, 12 * 9, line * 20);
    // Only draw current limit channel in red if it is true.
    if (clim[1]) {
        tft.setTextColor(TFT_RED);
        tft.drawString("B", 12 * 15, line * 20);
    }
    if (clim[0]) {
        tft.setTextColor(TFT_RED);
        tft.drawString("A", 12 * 17, line * 20);
    }

    line = 4;
    tft.setTextColor(TFT_WHITE);
    sprintf(buf, "Wo %5.2f %5.2f", powOutW[1], powOutW[0]);
    tft.drawString(buf, 0, line * 20);

    line = 5;
    sprintf(buf, "Vs %5.2f %5.2f %5.2f", vsw[1], vsw[0], vin);
    tft.drawString(buf, 0, line * 20);

    line = 6;
    sprintf(buf, "TC %5.1f %5.1f", tempC[1], tempC[0]);
    tft.drawString(buf, 0, line * 20);
}

void displayMode1() {
    float pvsetout[2], pisetout[2];
    char buf[22];
    int line = 0;
    for (uint8_t chan = 0; chan < 2; chan++) {
        pvsetout[chan] = calib.DACToVolts(preset.pvsetval[chan]) * 10.0;
        pisetout[chan] = calib.DACToVolts(preset.pisetval[chan]) * 0.94;
    }
    tft.setTextColor(TFT_WHITE);
    tft.drawString("Preset", 0, line * 20);

    line = 1;
    tft.drawString("Act: ", 0, line * 20);
    tft.setTextColor(TFT_GREEN);
    if (pract == 0) {
        tft.drawString("None", 12 * 6, line * 20);
    } else if (pract == 1) {
        tft.drawString("Use", 12 * 6, line * 20);
    } else if (pract == 2) {
        tft.drawString("Save", 12 * 6, line * 20);
    }

    line = 2;
    tft.setTextColor(TFT_WHITE);
    tft.drawString("Vv ", 0, line * 20);
    sprintf(buf, "%5.2f ", pvsetout[1]);
    tft.drawString(buf, 12 * 3, line * 20);
    sprintf(buf, "%5.2f", pvsetout[0]);
    tft.drawString(buf, 12 * 9, line * 20);

    line = 3;
    tft.drawString("Al ", 0, line * 20);
    sprintf(buf, "%5.3f", pisetout[1]);
    tft.drawString(buf, 12 * 3, line * 20);
    sprintf(buf, "%5.3f", pisetout[0]);
    tft.drawString(buf, 12 * 9, line * 20);
}

void displayMode2() {
    char buf[22];
    int line = 0;
    tft.setTextColor(TFT_WHITE);
    tft.drawString("Calib", 0, line * 20);

    line = 1;
    sprintf(buf, " %5.2f %5.2f", vout[1], vout[0]);
    tft.drawString(buf, 0, line * 20);
}

void displayInfo() {
    tft.fillScreen(TFT_BLACK);
    tft.setTextSize(2);
    tft.setCursor(0, 0);
    tft.setTextColor(TFT_WHITE);
    tft.setTextWrap(true);
    if (dmode == 0) {
        displayMode0();
    } else if (dmode == 1) {
        displayMode1();
    } else if (dmode == 2) {
        displayMode2();
    } else {
        // shouldn't be here!
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

void setup(void) {
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
    enc_but.attach(ENC_SW, INPUT_PULLUP);  // USE INTERNAL PULL-UP
    but0.attach(TT_BUT0, INPUT_PULLUP);
    but1.attach(TT_BUT1, INPUT_PULLUP);

    // DEBOUNCE INTERVAL IN MILLISECONDS
    enc_but.interval(5);  // interval in ms
    but0.interval(5);
    but1.interval(5);

    calib.begin();
    preset.begin();
    wificon.begin();

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

// perform the action selected in the Preset mode
void presetAction() {
    if (pract == 0) {
        Serial.println("pract: None");
    } else if (pract == 1) {
        Serial.println("pract: Use");
        for (uint8_t chan = 0; chan < 2; chan++) {
            vsetval[chan] = preset.pvsetval[chan];
            isetval[chan] = preset.pisetval[chan];
            dac.setData(vsetval[chan], dacvset[chan]);
            vsetout[chan] = calib.DACToVolts(vsetval[chan]) * 10.0;
            dac.setData(isetval[chan], daciset[chan]);
            isetout[chan] = calib.DACToVolts(isetval[chan]) *
                            0.94;  // new board R on imon = 4.7K
        }
    } else if (pract == 2) {
        Serial.println("pract: Save");
        for (uint8_t chan = 0; chan < 2; chan++) {
            preset.pvsetval[chan] = vsetval[chan];
            preset.pisetval[chan] = isetval[chan];
            preset.save();
        }
    }
}

void loop(void) {
    bool disp = false;  // update display if true
    uint8_t chan;
    unsigned long ctimeus = micros();
    unsigned long deltaus;
    char inbyte = 0;
    String ssid, pass;

    // r: print raw ADC values
    // s: print ssid
    // S <ssid> : set ssid
    // p: print pass
    // P <pass> : set pass
    // e: print wfen
    // E: toggle wfen
    if (Serial.available() > 0) {
        inbyte = Serial.read();
        switch (inbyte) {
            case 'r':
                for (uint8_t chan = 0; chan < 8; chan++) {
                    Serial.print(adc.read(chan));
                    Serial.print(' ');
                }
                Serial.println(analogRead(VINADC));
                break;
            case 's':
                Serial.print("ssid: ");
                Serial.println(wificon.getSSID());
                break;
            case 'S':
                ssid = Serial.readString();
                ssid.trim();
                wificon.setSSID(ssid);
                Serial.print("ssid saved: "); Serial.println(ssid);
                break;
            case 'p':
                Serial.print("pass: ");
                Serial.println(wificon.getPass());
                break;
            case 'P':
                pass = Serial.readString();
                pass.trim();
                wificon.setPass(pass);
                Serial.print("pass saved: "); Serial.println(pass);
                break;
            case 'e':
                Serial.print("wfen: "); Serial.println(wificon.wfen);
                break;
            case 'E':
                wificon.wfen = !wificon.wfen;
                wificon.setWfen();
                break;
            default:
                if (inbyte == '\n' || inbyte == '\r') {
                } else {
                    Serial.print("Unknown: ");
                    Serial.println(inbyte);
                }
        }
    }

    if (wificon.wfen) {
      if (!wificon.connect()) {
        Serial.println("Connection failed");
      }
    } else {
      if (!wificon.disconnect()) {
        Serial.println("Disocnnection failed");
      }
    }

    // XXX but0 controls both enables together for now
    but0.update();
    if (but0.fell()) {
        enable[0] = 1 - enable[0];  // Toggles enable between 0 and 1
        enable[1] = enable[0];
        digitalWrite(enpin[0], enable[0] ? HIGH : LOW);
        digitalWrite(enpin[1], enable[1] ? HIGH : LOW);
        disp = true;
    }

    // but1 changes display mode
    but1.update();
    if (but1.fell()) {
        dmode = (dmode + 1) % 3;  // Cycles dmode through 0, 1, 2
        // XXX If we came out of debug mode check if we need to save calibration
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
        if (dmode == 0) {
            // For Normal mode control V for both channels and I for both
            // channels
            smode = (smode + 1) % 4;
        } else if (dmode == 1) {
            // For Preset mode does the action selected by encoder
            presetAction();
        } else if (dmode == 2) {
        }
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
            // Serial.print("incr>1 ");
        }
        // If the increment happens faster than 40ms, multiply increment by 8
        // or if faster than 60ms multiple by 4
        if (deltaus < 40000) {
            incr *= 8;
            // Serial.print("us<40k ");
        } else if (deltaus < 60000) {
            incr *= 4;
            // Serial.print("us<60k ");
        }
        // Serial.println(incr);
        ptimeus = ctimeus;

        oldPosition = newPosition;
        if (dmode == 0) {
            // Normal mode, change the voltage or current limit settings
            if (smode == 0 || smode == 1) {
                chan = smode;
                vsetval[chan] = incr12(vsetval[chan], incr);
                dac.setData(vsetval[chan], dacvset[chan]);
                vsetout[chan] = calib.DACToVolts(vsetval[chan]) * 10.0;
            } else if (smode == 2 || smode == 3) {
                chan = smode - 2;
                isetval[chan] = incr12(isetval[chan], incr);
                dac.setData(isetval[chan], daciset[chan]);
                isetout[chan] = calib.DACToVolts(isetval[chan]) *
                                0.94;  // new board R on imon = 4.7K
            }
        } else if (dmode == 1) {
            // Preset mode, change preset action
            pract = (pract + 1) % 3;
        }
        disp = true;
    }

    disp = readClims() ? true : disp;
    readAvolts();
    loopcount++;
    if (loopcount >= COUNTMAX) {
        disp = true;
    }
    if (disp) {
        loopcount = 0;
        displayInfo();
    }
}
