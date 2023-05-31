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

#include "wificon.h"

Wificon::Wificon() {};

void Wificon::begin() {
    _pref.begin("cred", false);
    wfen = _pref.getBool("wfen", false);
    WiFi.mode(WIFI_STA);
}

void Wificon::setSSID(String ssid) {
    _pref.putString("ssid", ssid);
}

void Wificon::setPass(String pass) {
    _pref.putString("pass", pass);
}

void Wificon::setWfen() {
    _pref.putBool("wfen", wfen);
}

String Wificon::getSSID() {
    return _pref.getString("ssid", "");
}

String Wificon::getPass() {
    return _pref.getString("pass", "");
}

bool Wificon::connect() {
    String ssid, pass;
    int waitCount = 10;

    if (!wfen) {
        Serial.print("Wifi not enabled");
        return false;
    }

    if (_wl_status == WL_CONNECTED) {
        return true;
    }

    ssid = _pref.getString("ssid", "");
    pass = _pref.getString("pass", "");
    Serial.print("Connection to ");
    Serial.println(ssid);
    _wl_status = WiFi.begin(ssid.c_str(), pass.c_str());

    while (_wl_status != WL_CONNECTED && waitCount >= 0) {
        waitCount--;
        delay(500);
        Serial.print(".");
        _wl_status = WiFi.status();
    } 

    if (_wl_status == WL_CONNECTED) {
        _addr = WiFi.localIP();
        Serial.println("");
        Serial.print("WiFi Connected, IP addr: "); Serial.println(_addr);
        return true;
    }
    return false;
}

bool Wificon::disconnect() {
    int waitCount = 10;

    if (_wl_status == WL_DISCONNECTED) {
        return true;
    }
    while (!WiFi.disconnect() && waitCount >= 0) {
        waitCount--;
        delay(500);
        Serial.print(".");
    }

    _wl_status = WiFi.status();

    if (_wl_status == WL_DISCONNECTED) {
        Serial.println("WiFi disconnected");
        return true;
    }
    return false;
}