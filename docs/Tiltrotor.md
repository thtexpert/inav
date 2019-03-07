

# Tiltrotor

## Mixer Structure

The Tiltrotor mixer consists of 2 parts:

- Swash Plate Mixer
- Servo Output Mixer

The signal from the flight control (roll, pitch, yaw and collective) is forwarded to the _Swash Plate Mixer_.

The _Swash Plate Mixer_ generates the controls for the left and right swash plate. The mixer scalings are intended to show the degrees of rotor blade angle.

The _Servo Output Mixer_ generates the individual Servo signal to move the Swashplate as given by the _Swash Plate Mixer_.

The Tiltrotor platform inherits all features, limitations and capabilities from the AIRPLANE platform type. The mixer occupies the first up to 6 servos of the servomixer.

The nacelle is controlle by 6th channel of receiver. An input of 1050usec maps to heli position, an input of 1950usec maps to plane position.

The tilt speed of the Nacelle is controlled by a speed parameter (`tilt_nacellespeed`). The nacelle trip is limited between `tilt_nacellemin` and `tilt_nacellemax`.

The 4th reciever channel maps to the collective rotor settings. A receiver input at Minimum Throttle set lowest collective rotor blade anble, at an input of  Maximum Throttle the blades are inclined maximum.

## CLI Command Reference

| Command | scaling  | Description |
|-------------------------------------------|-----------------|-------------------------------|
| **Swash Plate Mixer** | | |  
| `tilt_nacellemax` |(scaling 100 = 1 degree) | maximum degrees the nacelle will travel |
| `tilt_nacellemin` | (scaling 100 = 1 degree)   |  minimum degrees the nacelle will travel |
| `tilt_nacellespeed` | (scaling 100 = 1 degree per second)   | nacelle rotation speed | 
| `tilt_cyclicring` |  (scaling 100 = 1 degree)| maximum blade deflection of swashplate (pitch will not travel further to avoid mechanical damage) |
| `tilt_collectivemaxheli` |  (scaling 100 = 1 degree)| maximum collective in heli position |
| `tilt_collectivemaxplane` |  (scaling 100 = 1 degree)|  maximum collective in plane position |
| `tilt_collectiveminheli` |  (scaling 100 = 1 degree)|  minimum collective in heli position |
| `tilt_collectiveminplane` | (scaling 100 = 1 degree) |   minimum collective in plane position|
| `tilt_gainpitchheli` | (scaling 10 = 1%) |  mixer pitch gain in heli position |
| `tilt_gainpitchplane` | (scaling 10 = 1%) | mixer pitch gain in plane position |
| `tilt_gaindiffcollheli` |  (scaling 10 = 1%)| mixer differnetial collective gain in heli position  |
| `tilt_gaindiffcollplane` | (scaling 10 = 1%) | mixer differnetial collective gain in plane position  |
| `tilt_gaindiffpitchheli` | (scaling 10 = 1%) | mixer differnetial pitch gain in heli position  |
| `tilt_gaindiffpitchplane` | (scaling 10 = 1%) | mixer differnetial pitch gain in plane position  |
| `tilt_centerall` |  | debug support for mechanical leveling. When set to 1, all swashplates will move to zero degree. Defualt is 0. Caution this disables all control! |
| `tilt_platetype` |  | Select the SwashPlateType for the servo mixer NORMAL (0): two servos front and back of rotor mast = 0, H120 = 1, custom = 2. This is storage only for the configurator. Configurator is able to set individual servo mixes (tmix) for each servo.
| `tilt_nacelletype` |  | Select number of nacelle servos  |
| `tilt_cyclictravel` | (scaling 10 = 1%) | preset cyclic servo mixer gain for tilt_platetype NORMAL |
| `tilt_collectivetravel` | (scaling 10 = 1%)  | preset collective servo mixer gain for tilt_platetype NORMAL |
| | **Servo Output Mixer** |
| `tmix 0 <pitch> <collective>` | (scaling 10 = 1%) | mixing of left swash plate pitch and collective to servo number 1 |
| `tmix 1 <pitch> <collective>` | (scaling 10 = 1%) | mixing of left swash plate pitch and collective to servo number 2 |
| `tmix 2 <pitch> <collective>` | (scaling 10 = 1%) | mixing of right swash plate pitch and collective to servo number 3 |
| `tmix 3 <pitch> <collective>` | (scaling 10 = 1%) | mixing of right swash plate pitch and collective to servo number 4 |
| `tmix 4 <plane> <heli>` | usec | servo 5 pulse length in heli-postion and  servo 5 pulse length in plane-postion|
| `tmix 5 <plane> <heli>` | usec | servo 6 pulse length in heli-postion and  servo 5 pulse length in plane-postion|
| | **Servo Output Configuration** |
| `servo <number> <min> <max> <middle> <direction>` | set maximum travel of given servo (<min> <max>), the offset trimming (<middle>) and the travel direction (<direction>` |  |  | 100 normal, <direction>` |  |  | 101 reverse) |

### useful CLI commands

`set platform_type TILTROTOR`

`set servo_pwm_rate 120`

`servopwmoverride 4 50`

`tiltrotor`  report all tiltrotor related settings

