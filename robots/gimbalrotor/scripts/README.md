**Last updated:** 2026-08-17  
**Documented branch:** `develop/pochitab_passiveperching/main`  

## Keyboard command

### `keyboard_changerpy.py`

Change robot's rpy using keyboard. 

### `keyboard_command2.py`

Same as: keyboard_command.py 

- except: track flight state (prevent arming when it is already in hovering state...)

### `keyboard_perching_pitch.py`

Manually changes the pitch delta relative to the locked perching orientation.(For passive pitch joint)

### `keyboard_perching_secondary.py`

rosrun gimbalrotor keyboard_perching_secondary.py \
  _pitch_step_deg:=0.5 \
  _secondary_step_deg:=0.5

e: disables admittance, enables perching, locks current contact transform and both joint angles 
j(-) / l(+): move yaw/roll joint 
s: press and wait for secondary_settled=True
i(+) / k(-): move pitch servo 
t: after completing all joint movements 
a: arm multilink perching admittance 
space: returns both joints to the angles captured when "e" was pressed 
d: disables admittance and perching

### `keyboard_servo.py`

Moves one servo by relative encoder steps while using the latest measured servo angle as the command base.

Run with:  
rosrun gimbalrotor keyboard_servo.py _servo_index:=4

### `saw_control.py`

Publish saw pwm with keyboard

## Perching geometry script

### `perching.py`

Manual publish robot's perching pith (using robot's pitch joint, robot's orientation, robot's position)

## Tasks

### `auto_perching_pitch_experiment.py`

Runs a repeatable downward perching-pitch step experiment.

### `close_hand_then_lock.py`

Safely performs the ordered mechanical sequence: close the hand first, then close the lock.

-- Todo, delete if unnecessary

### `servo_pidgain.py`

Applies and verifies one DYNAMIXEL servo's position PID gains.

To check for servo pid, run:  
rosservice call /get_board_info

## URDF 

### `urdf_inertia_markers.py`

Visualizes URDF inertial properties in RViz.

## Mujoco_scripts 

### `postprocess_beetle_cutting_model.py`
Post-processes the MuJoCo model by positioning the robot at the configured perch pivot and adding the cutting contact site, perching constraints, and branch visualization.