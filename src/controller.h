#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "QuickPID.h"
#include "sTune.h"
// #include "PIDAutotuner.h"

#include "motor.h"
#include "settings.h"

#define COMPUTE_FREQ 1000
#define SETPOINT_FREQ 100

// whether to simulate a perfect PID response for setpoint testing
#define SIMULATED false

class PIDControl{
protected:
    enum SetpointMode {STEP_POS,SPEED_POS,ACCEL_POS,STEP_VEL,ACCEL_VEL};

    // two different tuning methods. sTune may yield better results
    // however it requires the approx time constant to be known already
    // relay tuner is a bit simpler

    /* --------------------------------- Objects -------------------------------- */
    sTune sTuner;
    Motor * motor;
    //? PIDAutotuner rTuner;

    Settings & settings;
    QuickPID pid;

    /* -------------------------------- Variables ------------------------------- */

    // float input=0;
    // float output=0;
    // float setpoint=0;

    // float prev_pos,current_pos,target_pos;
    // float prev_vel,current_vel,target_vel;

    bool active=true;

    SetpointMode setpoint_mode=STEP_POS;

    unsigned long prev_setpoint_time=0;
    unsigned long prev_compute_time=0;

    

    /* -------------------------------- Constants ------------------------------- */

    const float out_min=-4095;
    const float out_max=4095;

    const int endstop_pins[4]={18,19,20,21};

    // time between PID computes, us
    const int compute_interval=1000000/COMPUTE_FREQ;
    // time between setpoint computes, us
    const int setpoint_interval=1000000/SETPOINT_FREQ;

    /* ---------------------------- Internal Methods ---------------------------- */

    // set PID tunings to values in Settings
    void apply_tunings();
    // check endstop
    void check_endstop(); 
    // run PID
    void calc_output();

    void calc_input(); // get input from encoder (speed or pos)
    void calc_setpoint(); // calculate new setpoint (speed or pos)
public:
    float input=0;
    float output=0;
    float setpoint=0;

    float accel_dist,decel_dist;

    float start_pos=0,prev_pos=0,current_pos=0,target_pos=0;
    float start_vel=0,prev_vel=0,current_vel=0,target_vel=0;
    float target_speed=0;

    // constructor
    PIDControl(Motor * motor, Settings & settings);

    /* ----------------------------- Utility Methods ---------------------------- */

    // s-curve tuning. better results but harder to set up. if continuous, tuning will continue during operation
    void s_tune(bool continuous, float time_const=0.5,float step=4095);
    // relay tuning using a step input. simpler, faster, worse result
    void r_tune();
    // Set custom tunings
    void set_tunings(float kp, float ki, float kd);

    // used to run the class
    void run(); // run this every loop
    void reset(); // resets the PID and targets
    void stop(); // cuts power, resets all values including targets
    void start(); // enables output, starts PID again

    /* -------------------------------- Commands -------------------------------- */
    
    // Move to position as fast as motor will go
    void set_pos(float position, bool relative=false);
    // Move at a set speed until position is reached
    void speed_pos(float position, float speed=-1, bool relative=false); // if speed=-1, uses maxSpeed
    // Accelerate to a set speed, then decelerate toward position 
    void accel_pos(float position, float speed=-1, bool relative=false); // if speed=-1, uses maxSpeed
    // Move as a set velocity
    void set_velocity(float velocity, bool relative=false);
    // Accelerate towards this velocity
    void accel_velocity(float velocity, bool relative=false);

};

#endif