#include "encoder.h"

QuadratureEncoder::QuadratureEncoder(int pinA, int pinB, int detents, bool pullup)
    :pinA(pinA),pinB(pinB),detents(detents),pullup(pullup){}

int QuadratureEncoder::begin() {
    // set pullups
    if (pullup) {
        gpio_pull_up(pinA);
        gpio_pull_up(pinB);
    }
    // add pio program to pio block if not done already, find offset
    if (instances==0) {
        pio0_offset = pio_add_program(pio0, &quadrature_program);
    } else if (instances==2) {
        pio1_offset = pio_add_program(pio1, &quadrature_program);
    }
    // find which pio block the program is in and use the corresponding offset
    if (instances<2) {
        pio=pio0;
        offset=pio0_offset;
    } else {
        pio=pio1;
        offset=pio1_offset;
    }
    // select a state machine and start program, return -1 if this fails
    sm = pio_claim_unused_sm(pio, false);
    if (sm==-1)
        return sm;
    quadrature_program_init(pio, sm, offset, pinA, pinB);
    // increment number of instances of this class
    instances++;
    return sm;
}

uint QuadratureEncoder::get_raw() {
    pio_sm_exec_wait_blocking(pio, sm, pio_encode_in(pio_x, 32));
    uint x = pio_sm_get_blocking(pio, sm);
    raw=x;
    return raw;
}

void QuadratureEncoder::zero(){
    pio_sm_exec(pio, sm, pio_encode_set(pio_x, 0));
}

uint QuadratureEncoder::get_angle() {
    return get_total_angle()%360;
}

uint QuadratureEncoder::get_total_angle() {
    return (3600/detents)*get_raw();
}

// init encoder class with zero instances
uint QuadratureEncoder::instances=0;