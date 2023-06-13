#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "quadrature.pio.h"


/* -------------------------- General Encoder class ------------------------- */
class Encoder {
    public:
    // get current angle within a single rotation, in tenths of a degree
    virtual uint get_angle();
    // get current cumulative angle
    virtual uint get_total_angle();
};
/* ------------------------- PIO quadrature encoder ------------------------- */
class QuadratureEncoder : public Encoder {
    // internal values
    int detents;
    uint raw;
    int pinA,pinB;
    bool pullup;
    // PIO variables
    static uint instances; // stores number of class instances such that the correct state machine is selected
    static uint pio0_offset;
    static uint pio1_offset;
    uint sm;
    uint offset;
    PIO pio;
public:
    QuadratureEncoder(int pinA, int pinB, int detents, bool pullup=true);
    // initialises pio sm, returns 0 if sucessful, -1 if failed
    int begin();
    // get raw value from pio program
    uint get_raw();
    // set current position in pio to zero
    void zero();
    // parent virtual method overrides
    uint get_angle();
    uint get_total_angle();
};