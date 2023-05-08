#include <Preferences.h>
#include <WiFi.h>

class Wificon {
   public:
    Wificon();
    void begin();
    void setSSID(String ssid);
    void setPass(String pass);
    void setWfen();
    String getSSID();
    String getPass();
    bool connect();
    bool disconnect();
    bool wfen;

   private:
    Preferences _pref;
    wl_status_t _wl_status;
    IPAddress _addr;    
};
