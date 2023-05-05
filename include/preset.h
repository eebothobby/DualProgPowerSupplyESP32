#include <Preferences.h>

class Preset {
   public:
    Preset();
    void begin();
    void get();
    void save();
    uint16_t pvsetval[2];
    uint16_t pisetval[2];

   private:
    Preferences _pref;
};
