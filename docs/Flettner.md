

# Flettner

## Mixer Structure

The Flettner mixer consists of 2 parts:

- Swash Plate Mixer
- Servo Output Mixer

The signal from the flight control (roll, pitch, yaw and collective) is forwarded to the _Swash Plate Mixer_.

The _Swash Plate Mixer_ generates the controls for the left and right swash plate. The mixer scalings are intended to show the degrees of rotor blade angle.

The _Servo Output Mixer_ generates the individual Servo signal to move the Swashplate as given by the _Swash Plate Mixer_.

The Flettner platform inherits all features, limitations and capabilities from the MULTICOPTER platform type. The mixer occupies the first 6 servos of the servomixer. Manual mode selection is possible for tiltrotor.

System uses multicopter PID controller.

## CLI Command Reference

| Command | Description |
|-------------------------------------------|------------------------------------------------|
| | **Swash Plate Mixer** |
| `flettner_pitchtravel` | amount of mixing from control elevator to swashplates (scaling 10 = 1%) |
| `flettner_rolltravel` | amount of mixing from control aileron to swashplates (scaling 10 = 1%) |
| `flettner_pitchtravel` | amount of mixing from control collective to swashplates (scaling 10 = 1%) |
| `flettner_cyclicring` | maximum angle that swash plate will go (scaling 100 = 1degree) |
| `flettner_collectivemax` | maximum value that the collective movement of the swashplate will do. Higher values are clamped (scaling 100 = 1degree) |
| `flettner_collectivemin` | minimum value that the collective movement of the swashplate will do. Higher values are clamped (scaling 100 = 1degree) |
| `flettner_cyclicmix` | amount of mixing from control yaw to differential elevator of the two swashplates. (scaling 10 = 1%) |
| `flettner_collectivemix` | amount of mixing from control yaw to differential collective of the two swashplates. (scaling 10 = 1%) |
| `flettner_collectivemixthreshold` | minimum value that the collective has to be to enable collective yaw mixing. (scaling 100 = 1degree) |
| `flettner_collectivemixmax` | maximum value that the yaw mixing will change the collective of the swashplates. (scaling 100 = 1degree) |
| `flettner_pitchff` | mix collective change to elevator. Allows feed forward compensation for nose up at pitch pumping (scaling 10 = 1%) |
| `flettner_centerall` | debug support for mechanical leveling. When set to 1, all swashplates will move to zero degree. Defualt is 0. Caution this disables all control! |
| | **Servo Output Mixer** |
| `flettner_platetype` | Select the SwashPlateType for the servo mixer preset H90 = 0, H120 = 1, custom = 2. This is storage only for the configurator. Configurator is able to set individual servo mixes (fmix) for each servo. |
| `flettner_rotationleft` | Provide info for configurator how the servos of left swash plate are rotated relative to aircraft (scaling 10 = 1degree). Configurator is able to set individual servo mixes (fmix) for each servo. |
| `flettner_rotationright` | Provide info for configurator how the servos of right swash are rotated relative to aircraft (scaling 10 = 1degree). Configurator is able to set individual servo mixes (fmix) for each servo. |
| `flettner_virtualrotleft` | Provide info for configurator how the servos of left swash plate are virtually rotated relative to regular rotation (scaling 10 = 1degree). Configurator is able to set individual servo mixes (fmix) for each servo. |
| `flettner_virtualrotright` | Provide info for configurator how the servos of right swash plate are virtually rotated relative to regular rotation (scaling 10 = 1degree). Configurator is able to set individual servo mixes (fmix) for each servo. |
| `flettner_cyclictravel` | Provide info for configurator how much and which direction the servos move for swash plate changes (scaling 10 = 1%). Configurator is able to set individual servo mixes (fmix) for each servo. Use this parameter to match _Swash Plate Mixer_ degree to real blade degrees |
| `flettner_collectivetravel` | Provide info for configurator how much and which direction the servos move for swash plate changes (scaling 10 = 1%). Configurator is able to set individual servo mixes (fmix) for each servo. Use this parameter to match _Swash Plate Mixer_ degree to real blade degrees |
| `flettner_collectiveoffset` | Collective angle at center of collective stick. Allows full usage of collective stick travel for non symmetric blade angles.(scaling 100 = 1degree) |
| `fmix 0 <roll> <pitch> <collective>` | mixing of left swash plate roll, pitch and collective to servo number 1 (scaling 10 = 1%) |
| `fmix 1 <roll> <pitch> <collective>` | mixing of left swash plate roll, pitch and collective to servo number 2 (scaling 10 = 1%) |
| `fmix 2 <roll> <pitch> <collective>` | mixing of left swash plate roll, pitch and collective to servo number 3 (scaling 10 = 1%) |
| `fmix 3 <roll> <pitch> <collective>` | mixing of right swash plate roll, pitch and collective to servo number 4 (scaling 10 = 1%) |
| `fmix 4 <roll> <pitch> <collective>` | mixing of right swash plate roll, pitch and collective to servo number 5 (scaling 10 = 1%) |
| `fmix 5 <roll> <pitch> <collective>` | mixing of right swash plate roll, pitch and collective to servo number 6 (scaling 10 = 1%) |
| | **Servo Output Configuration** |
| `servo <number> <min> <max> <middle> <direction>` | set maximum travel of given servo (<min> <max>), the offset trimming (<middle>) and the travel direction (<direction> = 100 normal, <direction> = -100 reverse) |

### useful CLI commands

`set platform_type = FLETTNER`

`set servo_pwm_rate = 120`

`set min_check = 1000`  to avoid yaw locking at low pitch


