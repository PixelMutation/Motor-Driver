
#include <cstdio>
#include "encoder.hpp"
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "hardware/pio.h"
#include "ctype.h"

#include <Arduino.h>
#include "Adafruit_PWMServoDriver.h"

#include "motor.h"
#include "utility.h"
#include "controller.h"
#include "settings.h"

#include "i2cscan.h"

/* -------------------------------- Settings -------------------------------- */

#define PWM_FREQ 1000
#define SERIAL_BAUD 230400
#define SDA_PIN 26
#define SCL_PIN 27
#define I2C_CLK 100000
#define MOTORS 8
#define ARM_PIN 28
/* -------------------------------- Variables ------------------------------- */

// pinout
// int pwmA        [] = { 0, 2, 4, 6, 8,10,12,14};
// int pwmB        [] = { 1, 3, 5, 7, 9,11,13,15}; 

// int encA        [] = { 0, 2, 4, 6, 8,10,12,14};
// int encB        [] = { 1, 3, 5, 7, 9,11,13,15};

/* --------------------------------- Objects -------------------------------- */

Adafruit_PWMServoDriver pwm(0x40,Wire1);

Motor * motors [MOTORS]; 
SettingsManager settings[MOTORS];
PIDControl * controllers[MOTORS];


/* ---------------------------- Utility Functions --------------------------- */


bool relative(char mode) {return mode=='~';}
bool absolute(char mode) {return mode=='=';}
bool circular(String command) {return command.endsWith(">");}

/* ---------------------------- Command Functions --------------------------- */

void setDuty(int mIdx, String command) {
    controllers[mIdx]->stop(); // ensure PID not running
    char mode=command.charAt(0);
    String value=command.substring(1,command.indexOf(' '));
    int val=value.toInt();
    if (relative(mode)) {
        Serial.printf("M%i duty + %i\n",mIdx,val);
        motors[mIdx]->set_duty(val,true);
        Serial.printf(" = %i",motors[mIdx]->get_duty());
    } else 
    if (absolute(mode)) {
        Serial.printf("M%i duty -> %i\n",mIdx,val);
        motors[mIdx]->set_duty(val);
    }
    else
        Serial.print("Invalid format, use ~ for relative or = for absolute");
}
void setPosition(int mIdx, String command) {
    controllers[mIdx]->start();
    char mode=command.charAt(0);
    String value=command.substring(1,command.indexOf(' '));
    float val=value.toFloat();
    int speed_idx=command.indexOf("S");
    float speed=-1;
    if (speed_idx>0)
        speed=command.substring(speed_idx+1).toFloat();
    if (relative(mode)) {
        Serial.printf("M%i pos +  %f speed %f\n",mIdx,val,speed);
        controllers[mIdx]->accel_pos(val,speed,true);
    } else 
    if (absolute(mode)) {
        Serial.printf("M%i pos -> %f speed %f\n",mIdx,val,speed);
        controllers[mIdx]->accel_pos(val,speed);
    // } else
    // if (circular(mode)) { //! add check that motor is set to circular mode
    //     Serial.printf("M%i pos <-> %f",mIdx,val);
    } else
        Serial.print("Invalid format, use ~ for relative or = for absolute");
}
void setVelocity(int mIdx, String command) {
    controllers[mIdx]->start();
    char mode=command.charAt(0);
    String value=command.substring(1,command.indexOf(' '));
    float val=value.toFloat();
    if (relative(mode)) {
        Serial.printf("M%i vel +  %f\n",mIdx,val);
        controllers[mIdx]->accel_velocity(val,true);
    }
    else 
    if (absolute(mode)) {
        Serial.printf("M%i vel -> %f\n",mIdx,val);
        controllers[mIdx]->accel_velocity(val);
    }
    else
        Serial.print("Invalid format, use ~ for relative or = for absolute");
}
void setConfig(int mIdx, String command) {
    Serial.printf("[SIMULATION] M%i config to %s\n",mIdx,command);
    if (settings[mIdx].set_JSON(command)) { // reset if changes require it
        motors[mIdx]->reset();
        controllers[mIdx]->reset();
    }
    settings[mIdx].save();
}
void stop(int mIdx) {
    Serial.printf("M%i emergency stop\n",mIdx);
    motors[mIdx]->stop();
    controllers[mIdx]->stop();
}
void reset(int mIdx) {
    Serial.printf("M%i reset\n",mIdx);
    motors[mIdx]->reset();
    controllers[mIdx]->reset();
}
void autotune(int mIdx) {
    Serial.printf("[SIMULATION] M%i autotune start\n",mIdx);
    // controllers[mIdx]->rTune();
}

/* ---------------------------------- Setup --------------------------------- */

void setup() {
    // Uncomment the below line to reverse the counting direction
    // enc.direction(REVERSED_DIR);
    Serial1.setPinout(16,17);
    Serial1.begin(SERIAL_BAUD);
    Serial.begin(SERIAL_BAUD);
    Wire1.setSDA(SDA_PIN);
    Wire1.setSCL(SCL_PIN);
    Wire1.setClock(I2C_CLK);
    Wire1.begin();

    pinMode(ARM_PIN,INPUT);

    delay(1000);
    i2cscan(&Wire1,Serial);

    // init pwm driver
    pwm.begin();
    pwm.setPWMFreq(PWM_FREQ);

    EEPROM.begin(512);

    // load settings from EEPROM
    for (auto s: settings) {
        //! s.load();
    }

    //! TEMP
    settings[0].settings.controller_mode=VELOCITY;

    Serial.println("hi");
    delay(100);

    for (int i=0; i<MOTORS; i++) {
        motors[i]=new Motor(pwm,settings[i].settings,i*2,i*2+1,i*2,i*2+1);
        controllers[i]=new PIDControl(motors[i],settings[i].settings);
    }

    

}

/* ---------------------------------- Loop ---------------------------------- */

// void loop1()
bool armed=false;

void loop() {
    // when disarmed, all motors and controllers are stopped reset
    if ((!digitalRead(ARM_PIN))!=armed) {
        // if coming back online, now we can zero all motors without hanging
        if (!armed) {
            // set correct parameters and init driver
            pwm.begin();
            pwm.setPWMFreq(PWM_FREQ);
            // send stop signal, zero all targets
            for (auto c : controllers) {
                c->stop();
            }
        }

        armed=!digitalRead(ARM_PIN);
    }

    for (auto m : motors) {
        m->run();
    }
    // Run PID
    if (armed) {
        for (auto c : controllers) {
            c->run();
        }
    }

    String line="";
    if (Serial1.available())
        line=Serial1.readStringUntil('\n');
    else
    if (Serial.available()) 
        line=Serial.readStringUntil('\n');
    
    // Serial interface
    if (line!="") {
        // String line=Serial.readStringUntil('\n');
        line.trim();
        if (line.startsWith("M")) {
            int commandIndex=line.indexOf(' ')+1;
            String Motor=line.substring(1,commandIndex-1);
            int mIdx=Motor.toInt();
            if (isDigit(Motor.charAt(0))) {
                int mIdx=Motor.toInt();
                if (inRange(mIdx,0,7)) {
                    char commandChar=line.charAt(commandIndex);
                    String command = line.substring(commandIndex+1);

                    switch (commandChar) {
                        case 'P':
                            setPosition (mIdx,command);break;
                        case 'V':
                            setVelocity (mIdx,command);break;
                        case 'D':
                            setDuty     (mIdx,command);break;
                        case 'C': 
                            setConfig (mIdx,command);break;
                        case 'S': 
                            stop(mIdx);break;
                        case 'R':
                            reset(mIdx);break;
                        case 'T':
                            autotune(mIdx);break;
                        default:
                            Serial.printf("Unknown command \"%c\". Valid options are P=position, V=velocity, S=stop, D=duty, T=tune, R=reset or C=config",commandChar);
                    }
                } else {
                    Serial.printf("Motor out of range 0-%u\n",MOTORS-1);
                }
            } else {
                Serial.print("Not an integer");
            }

            Serial.println();
        }
    }

    static unsigned long prevPrint=0;
    if (millis()-prevPrint>30) {
        prevPrint=millis();
        plot("CurrentPos",controllers[0]->current_pos);
        plot("CurrentVel",controllers[0]->current_vel);
        plot("TargetPos",controllers[0]->target_pos);
        plot("TargetVel",controllers[0]->target_vel);
        plot("SetPoint",controllers[0]->setpoint);
        plot("AccelDist",controllers[0]->accel_dist);
        plot("DecelDist",controllers[0]->decel_dist);
        // Serial.print(">Pos:");
        for (int i =0; i<MOTORS; i++) {
            // Serial.print(millis());Serial.print(":");Serial.print(motors[i]->get_float_revolution()); 
            // if (i<7)
            //     Serial.print(";");
            plot("Pos"+String(i),motors[i]->get_float_revolution());
        }
        // Serial.println();
        // Serial.print(">Vel:");
        for (int i =0; i<MOTORS; i++) {
            // Serial.print(millis());Serial.print(":");Serial.print(motors[i]->get_velocity());
            // if (i<7)
            //     Serial.print(";");
            plot("Vel"+String(i),motors[i]->get_velocity());
        }
        // Serial.println();
    }
}