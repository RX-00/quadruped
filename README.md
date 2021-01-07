# quadruped

Custom quadruped based on the MOCO quadruped open source project

Used for/at MassRobotics

Differences include:
    - More powerful and larger servos
    - STL mods for carbon fiber 3D printing
    - All custom electronics and power
    - Better servo options/interface
    - Rewritten software
    - STL design modifications based on mods
    - etc.


Using the champ by chvmp ros packages for functionality
Using the spotMicro code base for low level control

# TODO
package: cocoa2_config -> bringup.launch, seems like the communication between i2cpwm_board's (/servos_absolute
/servos_drive
/servos_proportional)
seems to be working with cocoa_config_node
--> Take a look at servo_move_keyboard package's servoMoveKeyboard


# CHAMP & QUADRUPED COCOA PACKAGE USE:
1. [remote pc] roscore
2. NOTE: ONLY USE IF ERROR, otherwise skip [quadruped pc] rosrun servo_move_keyboard servoMoveKeyboard.py # then cancel ctrl-c
3. [quadruped pc] rosrun imu_mpu6050 imu_mpu6050_node
4. [quadruped pc] rosrun i2cpwm_board i2cpwm_board
wait a bit for things to init.
5  [remote pc] roslaunch cocoa_config motion_cmd.launch
6. [remote pc] roslaunch cocoa_config bringup.launch rviz:=true
7. [remote pc] roslaunch champ_teleop teleop.launch
