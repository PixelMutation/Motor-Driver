#include "motor.h"


Motor::Motor(Adafruit_PWMServoDriver &pwm, Settings & settings, int encA, int encB, int pwmA, int pwmB) 
    :pwmA(pwmA),pwmB(pwmB),
    delta(0.5),pwm(pwm), settings(settings)
{
    // Need to find which state machine to use for the encoder
    
    if (instances<4) {
        pio=pio0;
        sm=instances;
    } else {
        pio=pio0;
        sm=instances-4;
    }
    
    encPins={(uint8_t)encA,(uint8_t)encB};
    init_encoder();
    instances++;
}
void Motor::init_encoder() {
    if (init)  // see if it has been initialised
        delete enc; // if so, delete the old encoder
    enc=new encoder::Encoder(pio, sm, encPins,pimoroni::PIN_UNUSED,pimoroni::NORMAL_DIR,settings.encoder_steps*settings.gear_ratio);
    enc->init();
    init=true;
}
void Motor::reset() {
    stop();
    init_encoder();    
}

void Motor::run() {
    static unsigned long prevVelTime=0;
    // update the velocity, calc accel
    if (micros()-prevVelTime>sample_interval) {
        float new_velocity=enc->capture().revolutions_per_second();
        if (invert)
            new_velocity=-new_velocity;
        accel=((new_velocity-prev_velocity)/sample_interval)*1000000;
        prev_velocity=velocity;
        velocity=new_velocity;
    }
}

float Motor::get_float_revolution() {
    if (invert)
        return -enc->revolutions();
    else
        return enc->revolutions();
}
long Motor::get_integer_revolution() {
    if (invert)
        return -enc->turn();
    else
        return enc->turn();
}
float Motor::get_total_angle() {
    if (invert)
        return -enc->Degrees();
    else
        return enc->Degrees();
}
float Motor::get_revolution_angle() {
    if (invert)
        return -enc->count();
    else
        return enc->count();
}
float Motor::get_velocity() {
    return velocity;
}
float Motor::get_velocity_degrees() {
    return velocity * 360.0f;
}

void Motor::stop() {
    direction=STOP;
    current_duty=0;
    pwm.setPin(pwmA,0);
    pwm.setPin(pwmB,0);
}
void Motor::forward(uint16_t duty) {
    // set both to zero for a bit when changing direction
    if (direction==BACKWARD) {
        stop();
        delayMicroseconds(dead_time); // this delay may be an issue...
        direction=FORWARD;
    }
    current_duty=constrain(duty,0,4095);
    if (invert){
        pwm.setPin(pwmA,current_duty);
        pwm.setPin(pwmB,0);
    } else {
        pwm.setPin(pwmA,0);
        pwm.setPin(pwmB,current_duty);
    }
    
}
void Motor::backward(uint16_t duty) {
    
    // set both to zero for a bit when changing direction
    if (direction==FORWARD) {
        stop();
        delayMicroseconds(dead_time); 
        direction=BACKWARD;
    }
    current_duty=constrain(duty,0,4095);
    if (invert){
        pwm.setPin(pwmA,0);
        pwm.setPin(pwmB,current_duty);
    } else {
        pwm.setPin(pwmA,current_duty);
        pwm.setPin(pwmB,0);
    }
}

void Motor::set_duty(int duty,bool relative) {
    if (relative)
        duty+=current_duty;
    if (duty>=0)
        forward((uint16_t)abs(duty));
    else
        backward((uint16_t)abs(duty));
}


int Motor::get_duty() {return current_duty;}

uint Motor::instances=0;