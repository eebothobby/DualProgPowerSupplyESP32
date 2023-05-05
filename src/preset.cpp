#include "preset.h"

Preset::Preset() {}

void Preset::begin() {
    _pref.begin("preset", false);
    // _pref.clear();
    get();
}

void Preset::get() {
    pvsetval[0] = _pref.getUShort("vval_0", 0);
    pvsetval[1] = _pref.getUShort("vval_1", 0);
    pisetval[0] = _pref.getUShort("ival_0", 0);
    pisetval[1] = _pref.getUShort("ival_1", 0);
}

void Preset::save() {
    _pref.putUShort("vval_0", pvsetval[0]);
    _pref.putUShort("vval_1", pvsetval[1]);
    _pref.putUShort("ival_0", pisetval[0]);
    _pref.putUShort("ival_1", pisetval[1]);
}