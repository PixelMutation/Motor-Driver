#include "controller.h"
#include "utility.h"



PIDControl::PIDControl(Motor * motor, Settings & settings):
motor(motor),settings(settings),
sTuner(&input,&output,sTune::Mixed_PID,sTune::direct5T,sTune::printSUMMARY)
{
    pid=QuickPID(&input,&output,&setpoint);
    pid.SetSampleTimeUs(compute_interval);
    pid.SetOutputLimits(out_min,out_max);

}

void PIDControl::set_tunings(float kp, float ki, float kd) {
    settings.kp=kp; settings.ki=ki; settings.kd=kd;
    apply_tunings();
}

void PIDControl::run() {
    // Get latest position and velocity
    if (SIMULATED && active) {
        static unsigned long prevSim=0;
        if (millis()-prevSim>10) {
            float dt=10/1000.0;
            if (settings.controller_mode==VELOCITY) {
                current_vel=setpoint;
                current_pos+=current_vel*dt;
            } else {
                current_pos=setpoint;
                current_vel+=(current_pos-prev_pos)/dt;
            }
        }
    } else {
        current_pos=motor->get_float_revolution()*settings.out_ratio;
        current_vel=motor->get_velocity()*settings.out_ratio;
    }
    // Check if endstop pressed
    check_endstop();
    // Find new setpoint for PID
    if (active) {
        calc_setpoint();
        calc_output();
    }
}
void PIDControl::calc_output() {
    if (micros()-prev_compute_time>compute_interval) {
        prev_compute_time=micros();
        calc_input();
        int prevOutput=(int)output;
        pid.Compute();
        if (prevOutput!=(int)output) {
            motor->set_duty((int)output);
        }
    }
}
void PIDControl::apply_tunings() {
    pid.SetTunings(settings.kp,settings.ki,settings.kd);
}
void PIDControl::start() {
    active=true;
    // pid.SetMode(QuickPID::Control::automatic);
    // pid.Initialize();
}
void PIDControl::reset() {
    pid.Reset();
    apply_tunings();
    target_pos=current_pos;
    target_vel=current_vel;
    if (settings.controller_mode==VELOCITY) {
        setpoint=current_vel;
    } else {
        setpoint=current_pos;
    }
}
void PIDControl::stop() {
    active=false;
    reset();
    motor->stop();
    // pid.SetMode(QuickPID::Control::manual);
}
void PIDControl::s_tune(bool continuous, float time_const,float step) {
    // Reset();
    // float inSpan=1000;
    // float outSpan=4095*2;
    // uint16_t samples=200;
    // sTuner.Configure(inSpan,outSpan,0,step,time_const,1,samples);
    // while(sTuner.Run()!=)
    // sTuner.Run();
    // sTuner.printPidTuner(5);
    // sTuner.GetAutoTunings(&tunings.kp,&tunings.ki,&tunings.kd);
    // ApplyTunings();
}
void PIDControl::r_tune() {
    // tuner->tu
    
}
void PIDControl::check_endstop() {
    if (inRange(settings.endstop,0,3)) {
        bool state=digitalRead(endstop_pins[settings.endstop]);
        if (settings.endstop_pullup && !state || !settings.endstop_pullup && state) {
            if (settings.stop_at_endstop)
                motor->stop();
            reset();
            current_pos=0;
            motor->zero();
        }
    }
}

/* -------------------------------- Commands -------------------------------- */

void PIDControl::set_pos(float position, bool relative) {
    if (relative)
        position+=current_pos;
    // limit pos to allowed range
    if (!isnan(settings.max_pos) && position>settings.max_pos)
        position=settings.max_pos;
    if (!isnan(settings.min_pos) && position<settings.min_pos)
        position=settings.min_pos;
    target_pos=position;
    setpoint_mode=STEP_POS;
}
void PIDControl::speed_pos(float position, float speed, bool relative) {
    set_pos(position);
    // limit pos to allowed range. if -1 (default), set to max
    if (speed<0)
        target_speed=settings.max_speed;
    else
    if (speed>settings.max_speed)
        target_speed=settings.max_speed;
    else
        target_speed=speed;
    setpoint_mode=SPEED_POS;
}
void PIDControl::accel_pos(float position, float speed, bool relative) {
    start_pos=current_pos;
    start_vel=current_vel;
    if (relative)
        position+=target_pos;
    speed_pos(position,speed);
    setpoint_mode=ACCEL_POS;
} 
void PIDControl::set_velocity(float velocity, bool relative) {
    if (relative)
        velocity+=target_vel;
    target_vel=constrain(velocity,-settings.max_speed,settings.max_speed);
    setpoint_mode=STEP_VEL;
}
void PIDControl::accel_velocity(float velocity, bool relative) {
    if (relative)
        velocity+=target_vel;
    set_velocity(velocity);
    setpoint_mode=ACCEL_VEL;
}

/* ------------------------------- Kinematics ------------------------------- */

void PIDControl::calc_input() {
    // if position mode
    if (settings.controller_mode==POSITION) 
        input=current_pos;
    // if velocity mode
    else 
        input=current_vel;
}
// generate setpoint waveform based upon movement mode
void PIDControl::calc_setpoint() {
    unsigned long interval=micros()-prev_setpoint_time;
    float at;
    float dist=target_pos-start_pos;
    // float remaining_dist;
    float half=start_pos+dist/2;
    // float end=start_pos+dist*0.99;
    if (interval>setpoint_interval) {
        prev_setpoint_time=micros();
        float dt= (float)interval/1000000.0f;
        // if position mode
        if (settings.controller_mode==POSITION) {
            switch (setpoint_mode) {
                case STEP_POS: // step input, move as fast as motor allows
                    setpoint=target_pos;
                    break;
                case SPEED_POS: // ramp input until reaches target
                    if (setpoint<target_pos)
                        setpoint+=dt*target_vel;
                    else
                    if (setpoint>target_pos)
                        setpoint-=dt*target_vel;
                case ACCEL_POS: // S-curve ramp toward target
                    
                    break;
                case STEP_VEL: // ramp at target speed
                    setpoint+=dt*target_vel;
                    break;
                case ACCEL_VEL: // increase ramp until reaches target speed
                    break;
            }
        }
        // if velocity mode
        else {
            switch (setpoint_mode) {
                case STEP_POS: // this is treated as speed_pos in this mode
                case SPEED_POS: // step input until reached position, then speed zero. Likely to overshoot / oscillate
                    if (current_pos<target_pos)
                        setpoint=target_vel;
                    else
                    if (current_pos>target_pos)
                        setpoint-=dt*target_vel;
                    else
                        setpoint=0;
                case ACCEL_POS: // trapezoid (ramp up, flat then ramp down)
                    at=settings.accel*dt;
                    
                    if (start_pos<target_pos) {
                        target_vel=target_speed;
                        if (start_vel<target_vel) {
                            accel_dist=(powf(target_vel,2)-powf(start_vel,2))/(2*settings.accel);
                            // dist to accelerate between start velocity and target velocity
                            // s= (v^2-u^2)/2a where v=target, u=start, a=accel
                            // dist to decelerate from current velocity to zero
                            decel_dist=(powf(target_vel,2))/(2*settings.accel);
                        } else {
                            accel_dist=0;
                            decel_dist=(powf(current_vel,2))/(2*settings.accel);
                        }
                        if (current_pos>=target_pos) {
                            setpoint=0;
                            target_pos=current_pos;
                        } else {
                            // whether to decelerate such that it arrives at the correct position
                            if (current_pos>half&&abs(target_pos-current_pos)<decel_dist) {
                                if (setpoint>(target_vel*0.01))
                                    setpoint-=at;
                            } else
                            if (current_pos<half&&abs(current_pos-start_pos)<accel_dist) {
                                setpoint+=at;
                            }
                        }
                                             
                    } else 
                    if (start_pos>target_pos) {
                        target_vel=-target_speed;
                        if (start_vel>target_vel) {
                            accel_dist=(powf(target_vel,2)-powf(start_vel,2))/(2*settings.accel);
                            // dist to accelerate between start velocity and target velocity
                            // s= (v^2-u^2)/2a where v=target, u=start, a=accel
                            // dist to decelerate from current velocity to zero
                            decel_dist=(powf(target_vel,2))/(2*settings.accel);
                        } else {
                            accel_dist=0;
                            decel_dist=(powf(current_vel,2))/(2*settings.accel);
                        }
                        if (current_pos<=target_pos) {
                            setpoint=0;
                            target_pos=current_pos;
                        } else {
                            // whether to decelerate such that it arrives at the correct position
                            if (current_pos<half&&abs(current_pos-target_pos)<decel_dist) {
                                if (setpoint<(target_vel*0.01))
                                    setpoint+=at;
                            } else
                            if (current_pos>half&&abs(start_pos-current_pos)<accel_dist) {
                                setpoint-=at;
                            }
                        }
                           
                    }
                    break;
                case STEP_VEL: // step to target speed
                    setpoint=target_vel;
                    break;
                case ACCEL_VEL: // ramp to target speed
                    // v=u+a*t
                    float at=settings.accel*dt;
                    if (setpoint<target_vel) {
                        setpoint+=at;
                        if (setpoint>=target_vel)
                            setpoint=target_vel;
                    } else
                    if (setpoint>target_vel) {
                        setpoint-=at;
                        if (setpoint<=target_vel)
                            setpoint=target_vel;
                    }
                    break;
            }
        }
    }
}
