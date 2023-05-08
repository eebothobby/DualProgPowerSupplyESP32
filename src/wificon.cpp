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