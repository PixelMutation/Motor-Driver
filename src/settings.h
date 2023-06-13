#ifndef SETTINGS_H
#define SETTINGS_H

#include "Arduino.h"
#include "EEPROM.h"
enum PIDMode{POSITION,VELOCITY};

// See README for details
struct Settings {
    // Limits
    float max_speed=1000;
    float accel=10;
    float max_pos=NAN;
    float min_pos=NAN;
    int endstop=-1;
    bool endstop_pullup=true; // whether switch is pullup or pulldown
    bool stop_at_endstop=true; // if circular, you may not want to stop...
    // PID
    float kp=1,ki=0,kd=0;
    PIDMode controller_mode=POSITION;
    // Motor
    uint encoder_steps=11;
    float gear_ratio=1;
    float out_ratio=1;
    bool invert=false;
    bool invert_encoder=false;
    int circular=-1;    
};

class SettingsManager {
    static uint next_EEPROM_addr; // end of last instantiated settings object
    uint EEPROM_start; // start addr of this settings object in eeprom
public:
    Settings settings;
    // Functions
    SettingsManager(){
        EEPROM_start=next_EEPROM_addr;
        next_EEPROM_addr=sizeof(this);
        load();
    }
    void save() {
        EEPROM.put(EEPROM_start,settings);
        EEPROM.commit();
    }
    void load() {
        settings=EEPROM.get(EEPROM_start,settings);
    }
    // Change settings using JSON values. Returns true if changes require a reset
    bool set_JSON(String serialised_json) {

        return false;
    }
    // Convert settings to JSON string
    String get_JSON() {

        return "";
    }
};



#endif