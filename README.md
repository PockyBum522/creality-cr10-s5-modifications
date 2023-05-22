# creality-cr10-s5-modifications
All files related to my Creality CR-10 S5

# Modifications
* Micro Swiss Direct Drive + Hotend
https://store.micro-swiss.com/products/micro-swiss-direct-drive-extruder-for-creality-ender-5

* CR-Touch Auto-Levelling Sensor
https://www.amazon.com/gp/product/B098QT5MXY/

* OctoKlipperPi

# Custom GCODE

## On Print Start GCODE

; ----------===== SETUP GCODE BEGIN =====----------
M201 X500.00 Y500.00 Z100.00 E5000.00 ;Setup machine max acceleration
M203 X500.00 Y500.00 Z10.00 E50.00 ;Setup machine max feedrate
M204 P500.00 R1000.00 T500.00 ;Setup Print/Retract/Travel acceleration
M205 X8.00 Y8.00 Z0.40 E5.00 ;Setup Jerk
M220 S100 ;Reset Feedrate
M221 S100 ;Reset Flowrate
G91                                                          ; Relative positioning to move head up
M211 S0                                                      ; Remove soft stop since machine isn't homed yet. But we don't want to home until things are hot, in case there's cold filament sticking out a little
G1 F1500                                                     ; Set default travel speed (Used when feedrate isn't explicitly specified in subsequent g commands)
G0 Z15                                                       ; Move head up a little in case it's at bed level. So the user can pull any filament that oozes, off. Also not heating the nozzle directly on the bed
M211 S1                                                      ; Re-enable soft stops
G90                                                          ; Back to absolute positioning
M140 S{material_bed_temperature_layer_0}                     ; Set Heat Bed temperature
M190 S{material_bed_temperature_layer_0}                     ; Wait for Heat Bed temperature
M104 S{material_print_temperature_layer_0}                   ; Set Extruder temperature
M109 S{material_print_temperature_layer_0}                   ; Wait for Extruder temperature
G28                                                          ; Home the printer
G92 E0 ;Reset Extruder
G1 Z2.0 F3000 ;Move Z Axis up
G1 X10.1 Y20 Z0.28 F5000.0 ;Move to start position
G1 X10.1 Y200.0 Z0.28 F1500.0 E15 ;Draw the first line
G1 X10.4 Y200.0 Z0.28 F5000.0 ;Move to side a little
G1 X10.4 Y20 Z0.28 F1500.0 E30 ;Draw the second line
G92 E0 ;Reset Extruder
G1 Z2.0 F3000 ;Move Z Axis up
; ----------===== SETUP GCODE FINISHED =====----------


## On Print End GCODE

; ----------===== FINISHED GCODE BEGIN =====----------
M104 S0 ;Turn-off hotend
M140 S0 ;Turn-off bed

G91 ;Relative positioning
G1 E-2 F2700 ;Retract a bit
G1 E-2 Z0.2 F2400 ;Retract and raise Z
G1 X5 Y5 F3000 ;Wipe out
G1 Z10 ;Raise Z more
G90 ;Absolute positioning

G1 X0 Y{machine_depth} ;Present print
M106 S0 ;Turn-off fan

M84 X Y E ;Disable all steppers but Z
; ----------===== FINISHED GCODE END =====----------


## On Cancelled (This goes in Octoprint)

; ----------===== CANCELLED GCODE BEGIN =====----------
{% snippet 'disable_hotends' %}     ; disable all heaters
{% snippet 'disable_bed' %}

G91         ; Relative positioning to move head up
G0 Z20      ; Move head up a little 
G90         ; Back to absolute positioning
G28 X       ; Home X
G28 Y       ; Home Y

; disable motors
M84

; disable part fan
M106 S0
; ----------===== CANCELLED GCODE FINISHED =====----------


# Klipper

## printer.cfg

# This file contains common pin mappings for the 2020 Creality CR-10
# V3. The mainboard is a Creality 3D v2.5.2 (8-bit mainboard with
# ATMega2560). To use this config, the firmware should be compiled for
# the AVR atmega2560.

# See docs/Config_Reference.md for a description of parameters.

# For better compatibility with GCodes generated for Marlin, you
# may wish to add the following section, if you have BLTouch:
[gcode_macro G29]
gcode:
    BED_MESH_CALIBRATE

[stepper_x]
step_pin: PF0 #ar54
dir_pin: PF1 #ar55
enable_pin: !PD7 #!ar38
microsteps: 16
rotation_distance: 40
endstop_pin: ^PE5 #^ar3
position_endstop: 0
position_max: 300
homing_speed: 50

[stepper_y]
step_pin: PF6 #ar60
dir_pin: PF7 #ar61
enable_pin: !PF2 #!ar56
microsteps: 16
rotation_distance: 40
endstop_pin: ^PJ1 #^ar14
position_endstop: 0
position_max: 300
homing_speed: 50

[stepper_z]
step_pin: PL3 #ar46
dir_pin: !PL1 #!ar48
enable_pin: !PK0 #!ar62
microsteps: 16
rotation_distance: 8
position_max: 400
#Uncomment if you have a BL-Touch:
position_min: -4
endstop_pin: probe:z_virtual_endstop
#and comment the follwing lines:
#position_endstop: 0.0
#endstop_pin: ^PD3 #ar18

[safe_z_home]
home_xy_position: 104.25,147.6
speed: 80
z_hop: 10
z_hop_speed: 10

[extruder]
step_pin: PA4 # ar26
dir_pin: !PA6 # !ar28
enable_pin: !PA2 # !ar24
microsteps: 16
rotation_distance: 7.7201944 # 16 microsteps * 200 steps/rotation / steps/mm
#Correction formula is new_rotation_distance = old_rotation_distance * mmsExtracted / 100.0
nozzle_diameter: 0.400
filament_diameter: 1.750
heater_pin: PB4 #ar10
sensor_type: EPCOS 100K B57560G104F
sensor_pin: PK5 #analog13
control: pid
pid_kp: 22.107
pid_ki: 1.170
pid_kd: 104.458
min_temp: 0
max_temp: 255

[heater_bed]
heater_pin: PH5 #ar8
sensor_type: ATC Semitec 104GT-2
sensor_pin: PK6 #analog14
control: pid
#Stock PID configuration taken from Marlin
pid_Kp: 201.86
pid_Ki: 10.67
pid_Kd: 954.96
min_temp: 0
max_temp: 130

[fan]
pin: PH6 #ar9

[mcu]
serial: /dev/ttyUSB0

[printer]
kinematics: cartesian
max_velocity: 300
max_accel: 3000
max_z_velocity: 5
max_z_accel: 100

[display]
lcd_type: st7920
cs_pin: PH1 #ar16
sclk_pin: PA1 #ar23
sid_pin: PH0 #ar17
encoder_pins: ^PC4, ^PC6 #^ar33, ^ar31
click_pin: ^!PC2 #^!ar35


#Uncomment the following lines if you have a BL-Touch
[bltouch]
sensor_pin: ^PD2 #^ar19
control_pin: PB5 #ar11
set_output_mode: 5V
pin_move_time: 0.4
stow_on_each_sample: False
probe_with_touch_mode: False
x_offset: 45.75
y_offset: -3.40
z_offset:  3.28
samples: 2
sample_retract_dist: 2
samples_result: average

#Uncomment the following lines if you have a BL-Touch
[bed_mesh]
speed: 50
horizontal_move_z: 6
mesh_min: 46.50,0.75
mesh_max: 253.5,295.85
probe_count: 7,7
algorithm: bicubic

[pause_resume]
recover_velocity: 50

[filament_switch_sensor fil_runout_sensor]
pause_on_runout: True
switch_pin: PE4 #ar2

[bed_screws]
screw1: 33,29
screw1_name: front left screw
screw2: 273,29
screw2_name: front right screw
screw3: 273,269
screw3_name: rear right screw
screw4: 33,269
screw4_name: rear left screw

#Uncomment the following lines if you have a BL-Touch
[screws_tilt_adjust]
screw1: 0,29
screw1_name: front left screw
screw2: 228,29
screw2_name: front right screw
screw3: 228,269
screw3_name: rear right screw
screw4: 0,269
screw4_name: rear left screw
speed: 50
horizontal_move_z: 10
screw_thread: CW-M3