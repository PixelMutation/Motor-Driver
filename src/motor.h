#ifndef MOTOR_H
#define MOTOR_H

#include "encoder.hpp"

#include "Arduino.h"
#include "Ewma.h"
#include "Wire.h"
#include "Adafruit_PWMServoDriver.h"
#include "settings.h"


#define SAMPLE_FREQ 100 // rate of velocity calculations

/* --- Holds the encoder object, converts gear ratios, sets output values --- */
class Motor {
    /* -------------------------------- Settings -------------------------------- */
    int pwmA,pwmB; // pins on PWM driver for this motor
    const uint dead_time=100; // time in us to stop both sides when changing direction
    bool invert;

    /* -------------------------------- Variables ------------------------------- */
    enum Dir{FORWARD,BACKWARD,STOP};
    // track motor status
    Dir direction=STOP; // used to track current direction to prevent shorts
    bool init=false;
    float velocity; // velocity in rev/sec
    float prev_velocity;
    float accel; // accel in rev/sec^2
    int current_duty;
    // encoder variables
    static uint instances; // stores number of class instances such that the correct state machine is selected
    int sm;
    PIO pio;
    pimoroni::pin_pair encPins;

    const uint sample_interval=1000000/SAMPLE_FREQ;

    /* --------------------------------- Objects -------------------------------- */
    Adafruit_PWMServoDriver &pwm;
    encoder::Encoder * enc;
    Ewma delta;
    Settings & settings;

    /* --------------------------------- Methods -------------------------------- */
    void init_encoder();
public:
    Motor(Adafruit_PWMServoDriver &pwm, Settings & settings, int encA, int encB, int pwmA, int pwmB);
    
    void reset(); // reset the object

    void run(); // call every loop update speed and accel

    void zero() {enc->zero();};

    float get_velocity_degrees(); // angular velocity in degrees/sec
    float get_velocity(); // angular velocity in rev/sec
    float get_float_revolution(); // get position in terms of revolutions. 1.0 is a revolution. floating point output position. precision decreases as position increases...
    float get_total_angle(); // total angle in degrees across all revs
    long get_integer_revolution(); // position as integer number of revolutions. position is thus 360*revolution()+angle()
    float get_revolution_angle(); // angle within the current revolution, degrees


    void stop(); // stop power to motor

    void forward(uint16_t duty);
    void backward(uint16_t duty);

    void set_duty(int duty, bool relative=false);
    // void change_duty(int change);
    int get_duty();
};

#endif