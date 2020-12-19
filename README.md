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


# CHAMP & QUADRUPED COCOA PACKAGE USE:
1. [remote pc] roscore
2. [quadruped pc] rosrun servo_move_keyboard servoMoveKeyboard.py # then cancel ctrl-c
3. [quadruped pc] rosrun i2cpwm_board i2cpwm_board
4. [remote pc] roslaunch cocoa2_config bringup.launch rviz:=true hardware:=true
5. [remote pc] roslaunch champ_teleop teleop.launch
