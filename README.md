# 8-Channel DC Motor Controller  
This firmware provides speed or position control for up to 8 DC motors using an RP2040.

- Uses a PID control loop
- Includes auto-tuning and manual tuning
- Serial interface over hardware and USB UART

## Operation
- Each motor uses a quadrature encoder, which is read by the PIO to reduce CPU load. From this, motor position and speed are measured.  
- A PID loop maintains position or speed control
    - position control 
        - speed control possible by generating a ramp setpoint with acceleration curves
        - this allows the speed and acceleration between points to be controlled
        - due to the use of floats, precision decreases with distance.
    - speed control
        - acceleration implemented using a ramp setpoint
        - allows approximate position control by 
- The control system is separated from the serial interface to ensure consistent performance.  
### Core 0 
- Controls the Serial Interface, sending telemetry and recieving commands
- Generates the setpoint for each motor based upon commands

### Core 1  
- Tracks the encoder positions, calculates speed
- Runs the PID loops
- Performs autotuning
## Commands
Each command starts with the motor number from 0-7, e.g. `M7 P=23`
### Utility
- `S` performs an emergency stop, zeroing all targets and resets PID.
- `T` begins autotuning sequence
- `R` resets the motor classes (required after updating certain settings)
### Movement
The command contains the type, mode, value and optional arguments
Valid movements commands are 
- *float* `P`osition, 
- *float* `V`elocity
- *int* `D`uty cycle, range -4095 to 4095 overrides everything except limit switches. 
#### Relative Mode
Indicated by a ~ between the command and value, e.g:   
- `V~-5.5` decreases velocity by 5.5  
- `P~5679.23 S23.5` increases position by 5679.23 at speed 23.5
#### Absolute Mode 
Indicated by an = between the command and value, e.g:   
- `V=5.5` sets velocity to 5.5
- `P=-5679.23` sets position to -5679.23 at *maxSpeed*
#### Speed Override
Speed for a position command can be reduced from the maximum by adding `S{val}` after, e.g. `M2 P=55 S12`
#### Circular Systems
If *circular*>0, position values will wrap at *circular*. In the following examples, *circular*=360.  
Absolute commands set the position within any one rotation, whilst relative commands offset the position, e.g.
- at pos 90, `P~90` moves +90 to 180, whilst `P=90` does nothing 
- at pos 45, `P~-90` moves -45 to -45=315, whilst `P=-90` moves +135 to -90=270

To control which direction it takes to that position, add > for cw/+ and < for ccw/- at the end of the command.
- at pos 10, `P=350 <` turns +20 to 350, whilst `P=350 >` turns -340 to 350.

By default, the motor takes the shortest route:
- at pos 45, both `P=90 >` and `P=90` will turn 45 cw to 90
- at pos 120, `P=90 >` will turn +330 whilst `P=90` will turn -30 to 90

A new position *value* greater than *circular* passes through *value//circular* revolutions before stopping at *value%circular*. This ignores direction overrides.  

- at pos 30, `P=360` moves through 330 cw to 0
- at pos 30, `P=3605` moves +3575 through 10 revolutions to 5, `P=3605`
- at pos 30, `P=-3605` moves +3575 through 10 revolutions to 5

#### Sequential Commands
By default, all commands override everything recieved so far.
Position commands can be set to wait for the previous position command to complete by adding `#` at the end. This allows the user to send a sequence of GCODE style commands to execute pre-defined movements.

## Settings
Each setting starts with the motor number from 0-7.
After a space and the `C`onfig command, JSON key:pairs are used to set parameters, 
These values are saved to EEPROM and recalled upon restart.
e.g. `M5 C{"maxSpeed":55,"accel":2,"circular":360}`.
### Limits
- *float* `maxSpeed`  
Sets maximum speed 
- *float* `accel`  
Sets target acceleration, real acceleration is limited by the motor speed, inertia etc.
- *float* `maxPos`  
Set upper position limit. Use `NaN` to remove limit
- *float* `minPos`   
Set lower position limit. Use `NaN` to remove limit
- *float* `home`  
Sets location of homing limit switch. Ignored if *endstop*=-1
- *int* `endstop`  
Set endstop pin, range 0-3 (not enough pins for all to have endstops, if need more use port expander). Set -1 to disable (default). Changing requires reset.
### PID 
PID constants are saved to EEPROM and recalled upon restart.
- *float* `KP`  
Set KP constant
- *float* `KI`  
Set KI constant
- *float* `KD`  
Set KD constant
- `sTune`  
This program includes two different autotune methods
S-curve autotune (LINK HERE)
- `rTune`  
Relay autotune (LINK HERE)
- *uint* `mode`  
If 0=position, 1=velocity. Each requires different tunings. Changing requires restart.

### Motor
- *uint* `encoderSteps`  
Set number of encoder steps per revolution. Changing requires reset. Must be >0
- *float* `gearRatio`  
Set gear ratio (encoder rev per shaft rev). Changing requires reset. Must be >0
- *float* `outRatio`  
Set output ratio (shaft rev per output unit). Must be >0  
e.g. for a wheel of diameter *D* mm moves *2\*pi\*D mm* per revolution, so use *1/(2\*pi)*
- *bool* `invert`  
Whether to invert direction of motor
- *int* `circular`  
Set wrap point for output unit (such as 360). Use -1 to disable. This sets minPos and maxPos to NaN.
## Telemetry
The state of the system is sent at regular intervals over Serial. This is formatted for TelePlot.  
Several variables are printed for each motor.
Plotting these allows the PID response to be evaluated and to see whether the motor is reaching commanded limits. 
You could relay these strings over UDP for remote telemetry.

<!-- Position  
`>realPos:{pos0};{pos1};{pos2}...`  
`>targetPos:{pos0};{pos1};{pos2}...`  
Velocity  
`>realVel:{pos0};{pos1};{pos2}...`  
`>targetVel:{pos0};{pos1};{pos2}...`  
Duty cycle  
`>duty:{pos0};{pos1};{pos2}...`   -->

`>pos:{current position};{target position}`  
`>vel:{current velocity};{target velocity}`  
`>duty:{duty cycle}`

## State of Development
- [x] Quadrature encoder support
- [ ] Basic PID loop operation
- [ ] PID autotuning
- [x] Serial commands
- [ ] Serial output
- [ ] Circular systems
- [ ] Sequential commands