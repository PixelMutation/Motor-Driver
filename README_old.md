# 8-Channel DC Motor Controller  
This firmware provides speed or position control for up to 8 DC motors using an RP2040.

- Uses a PID control loop
- Includes auto-tuning and manual tuning
- Serial interface over hardware and USB UART

## Operation
### Core 0 
- Controls the Serial Interface, sending telemetry and recieving commands
- Generates the setpoint for each motor based upon commands

### Core 1  
- Tracks the encoder positions, calculates speed
- Runs the PID loops
- Performs autotuning
## Commands
Commands are formatted in a GCode style.  
Each command starts with the motor number from 0-7 e.g. `M5`.  
After a space, the parameter and value are given.
### Movement
- `X{val}`
Set position
- `V{val}`
Set velocity  
#### Modifiers
- `R` makes the command relative.
- `O` overrides the previous command instead of waiting for completion  
#### Other commands
- `S`
Stop the motor. Overrides everything
- `E{val}`
Simulate an endstop. Resets stored position to `val` (typically zero)
### Movement Settings
These values are saved to EEPROM and recalled upon restart.
- `S{val}`
Sets maximum speed 
- `A{val}`
Sets target acceleration, real acceleration is limited by the motor
- `U{val}`
Set upper position limit. Use `N` to remove limit
- `L{val}` 
Set lower position limit. Use `N` to remove limit
### Tuning
PID constants are saved to EEPROM and recalled upon restart.
- `T{val}`
Autotune where methods are `S` tuning or `R`elay tuning
- `P{val}`
Sets KP constant
- `I{val}`
Sets KI constant
- `D{val}`
Sets KD constant
### Motor Settings
- `G{val}` Set gear ratio (encoder rev per shaft rev)
- `R{val}` Set output ratio (shaft rev per output unit)  
e.g. for a wheel of diameter *D* mm moves *2\*pi\*D mm* per revolution, so use *1/(2\*pi)*
## Telemetry
The state of the system is sent at regular intervals over Serial. This is formatted for TelePlot.  
For each motor, several variables are printed:  
`>pos:{current position};{target position}`  
`>vel:{current velocity};{target velocity}`  
`>duty:{duty cycle}`
## State of Development
- [x] Quadrature encoder support
- [ ] Basic PID loop operation
- [ ] PID autotuning
- [ ] Serial commands
- [ ] Serial output