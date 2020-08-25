# 1 OLDX-MocoMoco Quadruped Robot Development Platform Project  
<div align=center><img width="600" height="130" src="https://github.com/golaced/OLDX_DRONE_SIM/blob/rmd/support_file/img_file/logo.JPG"/></div >
<div align=center><img width="400" height="300" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/fc2.jpg"/>< /div>

  ____OLDX-MocoMoco Quadruped (OLDX-MocoMoco Quadruped) is currently the first real open source foot robot secondary development platform in China. The team has developed a complete set of free open source
  Project OLDX-FC, with the continuous launch of foot robots by Boston Dynamics in recent years, has attracted the attention of many enthusiasts at home and abroad, and there have also been foot robots developed by many electronic enthusiasts.
  Different from the four-axis aircraft that can be developed only by using brushless motors and frames, the foot-mounted robot itself is complex in structure, especially the servo drive part can be independently studied as a discipline, which results in
  Although there are many good footed robot projects, their promotion or open source is hindered due to the cost of the project or the reason that the programming language is not familiar with the domestic environment. The team started to develop six
  The foot robot has a wealth of robots. The MocoMoco quadruped robot development platform is launched by combining it with the developed flight control project, which includes the robot structure, gait algorithm and SLAM unmanned driving
  And other core technical content of multiple foot robots. The project follows the GPL agreement and can modify and re-develop the relevant source code in the DEMO. <br><br>

**Relevant information** 
Robot test video update address: https://www.bilibili.com/video/av46405055 <br>
To facilitate the program to download the current versions of the firmware separately from here: https://github.com/golaced/MocoMoco_Software<br>
Robot calibration video tutorial: https://www.bilibili.com/video/av51603154<br>
<br>
**-How to build the project-**

Method|Description|Delivery Cycle
-------------|-------------|-------------
Whole machine purchase (**recommended**)|Buy a carbon rack + processed controller from the official Taobao store (excluding the steering gear)|Maximum half a month
Rack printing + controller purchase | 3D printer rack provided free of charge for processing project files + purchase controller from official + purchase steering gear by yourself | up to 1 week
Rack printing + controller plate processing | self-made 3D printer frame + self-made, welding controller + own purchase of steering gear | controller (4-layer board) + IMU + downloader adapter + power supply module

**-Open source program and VMC robot library-**

Way|description|open source mode
-------------|-------------|-------------
(**VMC Robot Library**)|Supports a variety of ground feedback, dynamic gait adjustment and more stable posture control capabilities. You need to purchase an authorization code from the official | Closed source (currently supports the STM32F4 series, and the library is bound to the chip ID )
Open source program | supports basic diagonal gait and posture control | open source

Code version|Open source code description|Closed source VMC library description
-------------|--------------|-------------
Publish Ver1.0|(1)Virtual model basic gait<br> (2)Gradient descent attitude calculation<br> (3)NRF radio frequency communication remote control<br> (4)UCOS operating system|(1)Virtual model+ IMU Homeostasis<br> (2) Dynamic gait + posture adjustment
Publish Ver1.1|(1)SDK development|(1)SDK development<br> (2)Adaptive step frequency<br> (3)Stable speed S function switching
Publish Ver1.2|(1) Gradient descent fusion magnetic field + magnetic field interference judgment<br> (2) KF barometer height fusion<br> (3) KF odometer pose fusion<br> |(1) Position closed loop control+ Jerk trajectory planning<br> (2) Jump API<br> (3) SLIP mode<br> (4) Gradient descent fusion magnetic field + magnetic field interference judgment<br> (5) KF barometer height fusion<br> (6) KF Odometer Pose Fusion<br>   
		
Official rack taobao link: https://item.taobao.com/item.htm?spm=a1z10.1-c.w4004-15110596499.22.6b86242c3mvjW9&id=589551490320
<br>
**-If this project is helpful to you, please Star our project-**<br>
**-If you are willing to share the optimization and improvement of the project, please contact golaced@163.com or join our QQ group 567423074 to accelerate the progress of the open source project-**<br>

<br>
**-Because I am non-professional research on quadruped robots, the project is only developed based on personal academic level and does not mean that the current footed robot algorithm is like this. Please do not rely too much on the content of the project or compare with regular products-**<br>
	
# 2 Introduction to MocoMoco Quadruped Robot Platform

 <div align=center><img width="550" height="300" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/r2.jpg"/>< /div>

____MocoMoco quadruped robot is an 8-DOF foot robot. It borrows from the Minitature robot introduced by GhostRobtic. The reason why it is not adopted like Boston Dynamics or other developers
The 12-DOF robot is greatly improved in maintenance difficulty and cost, and the 8-DOF robot can perfectly simulate the control characteristics of the 12-DOF single axis and quickly learn virtual legs and
The gait theory related to the virtual model only loses part of the lateral movement ability. At the same time, for the small-footed robot, the external environment is not narrow in an absolute sense compared to its own moving speed.
The area can meet the reliable movement in most indoor and outdoor scenes. <br>
____In addition, the drive adopts a fixed position installation method, which has greater advantages in service life and difficulty of maintenance, and is suitable for damage that may occur during experimental debugging. In order to reduce the cost of the servo drive, the brushless motor used is replaced with a high-performance steering gear. The carbon plate and 3D printed parts are used as the main structure. It has the characteristics of convenient installation and easy maintenance. It is a desktop-level quadruped robot research and development and SLAM algorithm Verification platform. The controller adopts STM32 and Raspberry Pi, which are currently more developed in China. At the same time, the hardware architecture will be used in subsequent robot projects. The OLDX-FC flight control project will be transplanted first to achieve real
The purpose of a multi-purpose board. The circuit board hardware adopts an integrated design and adopts the smallest Raspberry Pi A3+. It has the characteristics of small size and high performance, which can provide development possibilities for subsequent images and lidar SLAM.
The control board uses STM32F4 as the processor. The onboard 3-axis accelerometer, 3-axis gyroscope, 3-axis magnetometer and barometer are designed with internal shock absorption. The circuit mounting holes can be perfectly installed with the Raspberry Pi and provide a 3D printed shell.
The control board uses an external replaceable power supply module to supply power. It has 12 PWM outputs and 4 AD sampling, and has an external expansion of 4 serial ports to be compatible with the wireless debugging launched by Pioneering Atom. The robots are different in debugging.

**Controller hardware parameters:**

Project|Parameter
-------------|-------------
Processor|STM32F405RGT6
Processor performance|32Bit ARM Cortex-M4 168MH
Gyro Accelerometer|LSM6DS33 
Magnetometer|LIS3MDL
Barometer|MS5611
Reserved interface|GPS-1 serial port-4 AD sampling-4 ground switch input-4
PWM output channel | 12 channel output 
Power supply|5V input servo external power supply AD sensor 3.3/5V power supply option
Image processor|Raspberry Pi A3+ 1.4G 4 core with independent power supply switch
Remote control mode|2.4G radio frequency SBUS model airplane remote control
Ground station|QGround anonymous ground station (requires additional OLDX-REMOTE monitor)

**Robot parameters:**

Project|Parameter
-------------|-------------
Foot robot type|8 degrees of freedom parallel robot
Size|30cm * 20cm *10cm  
Full leg length|8cm
Weight|600g
Power supply|7.4V 18650 * 2 (3000mah battery life>35 min) with switch 
Gait support|Tort gait Fly-Trot gait fluctuation gait
Maximum moving speed|0.4m/s
Maximum steering speed|30 degrees/s
Gait cycle|>0.35 s
Servo performance|Kpower 12g metal steering gear*8 (6V-60 degrees/0.035s 9kg torque)
Foot sensor | membrane pressure / micro switch (default no sensor)
Control mode|Remote control mode Attitude balance mode (with/without touch sensor) Driving mode  


<br>
**Controller PCB interface description:**
<div align=center><img width="540" height="460" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/fc.jpg"/>< /div>
<br>

Interface|Description (from a picture perspective)|Support module
-------------|-------------|-------------
Reset|controller reset button|
Wireless download|From left to right: GND SCK SWD 5V|Connect to the punctual atomic wireless debugging module
SWD|From left to right: GND SWD SCK 5V|Connect to Stlink adapter board
GPS|From left to right: 5V RX2 TX2 SCL SDA GND|Connect to MINIGPS
Foot sensor|From top to bottom: VCC GND AD1/SCL1 KEY_GROUND1/SDA1|Connect to pressure/switch/vibration/ranging
Power supply selection|Horizontal jumper from top to bottom: 3.3V 5V|
SBUS|From left to right: GND 5V Signal|WBUS Futaba SBUS receiver
Optical flow|from left to right: RX4 TX4 GND 5V| to optical flow sensor
Digital transmission|From left to right: TX1 RX1 GND 5V|Anonymous data transmission 
Raspberry Pi|From left to right: TX3 RX3 GND 5V|Raspberry Pi A3
Leg 1|From left to right: GND VCC PWM From top to bottom: outer steering gear, inner steering gear manipulator arm/pan head|steering gear external power supply
Power|from top to bottom: BEEP servo power supply enable GND GND 5V VCC servo VCC servo VCC battery|power module 

**Note: The controller body is RGB LED direction. The above table only shows the pin sequence of leg 1 servo interface and sensor interface. The remaining three legs are mirrored and symmetrical.
That is, taking leg 2 as an example, from top to bottom: the outer servo, the inner servo gimbal, from left to right: PWM VCC GND, the leg 2 sensor from top to bottom:
KEY_GROUND2/SDA2 AD2/SCL2 GND VCC**

#3 DEMO test
____This project provides a free basic test 3D printer rack that can be downloaded for free and self-printed. The official will launch a full carbon version of the rack please
Pay attention to the third-party expansion modules launched by Taobao store, the controller also serves as the required screw models of each version of the rack as follows:

Metal Parts|Quantity (3D Printer Stand)|Assembly
-------------|-------------|----------
M3*30|8|Install arm
M2*10|8|Install the servo
M3*5|8|Mounting legs
M3*10|4|Install the sole
M3*15 copper column|4|central body support
M1.4*4|8|Fixed gear
M2.5*3|4|Install battery box
M2.5*5|8|Fixed controller
M2.5 copper pillar*4|4|controller support

Metal Parts|Quantity (Official Carbon Rack)|Assembly
-------------|-------------|----------
M2*16|8|Fixed arm
M2*5|8|Install the servo
M3*5|8|Mounting legs
M3*10|4|Install the sole
M3*32 Copper column|4|Central body support
M3*4|8|Central body fixed
M1.4*4|8|Fixed gear
M2.5*3|4|Install battery box
M2.5*5|8|Fixed controller

## 3.1 Controller assembly (Raspberry Pi A3)
(1) Install Raspberry Pi:<br>
Put the Raspberry Pi into the bottom case in the correct direction. In order to avoid installing the camera in this step, it is best to install the CSI cable corresponding to the Raspberry Pi camera**, and place the support pillars from the bottom.
Fix it with the Raspberry Pi by M2.5, and lead the CSI cable out of the corresponding slot of the shell.
<div align=center><img width="540" height="400" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/pi1.jpg"/>< /div>
<br>

(2) Install the controller:<br>
Lead the controller NRF antenna out of the corresponding round hole on the bottom shell wall, and install it on the Raspberry Pi in the correct orientation. Note that there is only a 4P female port on the control board. The **head pin does not correspond to the Raspberry Pi head pin* *And the pin header is on the outside, the correct installation should be as shown in the figure below:
<div align=center><img width="540" height="360" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/pi2.jpg"/>< /div>
<br>

(3) Install the upper cover: <br>
Install the upper cover on the control board in the correct direction. The main center slot should completely wrap the IMU board. After completion, the upper cover is also fixed with M2.5 screws to complete the controller assembly.
<div align=center><img width="540" height="460" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/pi3.jpg"/>< /div>
<br>

## 3.2 Robot assembly (official rack)
### 3.2.1 Assembly of arm and center body
(1) To assemble the arm, 2 arm carbon sheets, semicircular 3D printing parts, and center body 3D printing support parts are required. Note that the robot arm needs to have an expansion angle of 5°. Note that the upper board card slot is L-shaped, The lower board card slot is T-shaped, and 2mm screws are used to reinforce the arm. The result is shown in the figure below:
<div align=center><img width="540" height="200" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/a1.jpg"/>< /div>
<br>
(2) Assemble the center body, the upper and lower center body carbon sheets and 4 M3*32 copper pillars are required. First, fix the copper pillars on the bottom plate.
<br>
(3) Install the arms, fix the four arms on the lower board slot, pay attention to use 5° to expand, the result is shown in the following figure:
<div align=center><img width="540" height="200" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/a3.jpg"/>< /div>

### 3.2.2 Install battery compartment, control board and steering gear
(1) First install the battery compartment, which is installed at the bottom of the center body and the switch direction is toward the rear. The result is shown in the following figure:
<div align=center><img width="480" height="380" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/a4.jpg"/>< /div>
<br>
(2) Install the steering gear. The steering gear is installed with the bearing down, and the outer steering gear bearing faces the inside of the body, and the inner steering gear bearing is out of the way to ensure that the legs will not be hindered by the body when moving. It is fixed on with 2mm self-locking screws. On the carbon plate, the result is shown in the figure below:
<div align=center><img width="540" height="460" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/a5.jpg"/>< /div>
<br>
(3) Install the control board, and install the control board above the center body (**3M glue with lock or Velcro is recommended to facilitate subsequent SD card replacement**), mainly if the controller shell is not installed, the controller needs to be 3D The prints are heightened to prevent short circuit between the bottom of the circuit board and the carbon plate and the estimated screws of the battery compartment. The result is shown in the figure below:
<div align=center><img width="540" height="460" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/a6.jpg"/>< /div>
<br>
(4) Connect the steering gear wire, wind the wire from the inside of the arm to connect the steering gear wire to the control board. Please refer to the subscript for the specific connection sequence. In addition, note that the PWM pin is facing the inside of the body for 5V power supply. Refer to the specific pin power supply sequence PCB IO diagram:

**Servo Wiring**
||
-------------|-------------|----------
Sch.PWM3 TIM3_3 [D_LEG] External servo 1|Airframe right|Sch.PWM6 TIM4_2 [D_LEG] External servo 2
Sch.PWM2 TIM3_2 [X_LEG] Internal servo 1| |Sch.PWM5 TIM4_1 [X_LEG] Internal servo 2
Sch.PWM1 TIM3_1 NS| |Sch.PWM4 TIM3_4 NS
**Head**||Tail
Sch.PWM7 TIM4_3 NS| |Sch.PWM10 TIM8_1 NS
Sch.PWM8 TIM4_4 [X_LEG] Inner servo 3| |Sch.PWM11 TIM8_2 [X_LEG] Inner servo 4
Sch.PWM9 TIM1_1 [D_LEG] External Servo 3|Body Left|PSch.WM12 TIM8_3 [D_LEG] External Servo 4

### 3.2.3 Connect the electronic module and the upper board
(1) Weld the step-down module with the battery compartment, paying attention to the input and output relationship and the positive and negative poles. The results are shown in the following figure:
<br>
(2) Connect the step-down module through the XH2.8-8 line, and insert the step-down module into the card slot at the back of the base plate. The result is shown in the following figure:
<div align=center><img width="540" height="300" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/a8.jpg"/>< /div>
<br>
(3) Install the upper plate, install the upper plate corresponding to the step-down module and the arm card slot, and fix the upper plate with the 4 supporting copper pillars with screws. The result is as follows:
<div align=center><img width="540" height="360" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/a9.jpg"/>< /div>





## 3.3 Leg installation, deviation calibration and IMU sensor calibration (**very important**)
<br>**Video tutorial link: https://www.bilibili.com/video/av47485521** <br>

**Quick calibration using OLDX-Remoter**<br>

Conditions|Description
-------------|-------------
Dial 1 on the right, and long press the joystick switch on the right for 6 seconds | Calibrate the accelerometer and gyroscope after the beep (need to stand still horizontally)
Dial 1 is on the left and the other is on the right. Long press the remote switch for 6s|The yellow light is always on, the buzzer sounds intermittently, and the controller rotates 360° on each side for 12s to complete the magnetometer calibration
Dials 1 and 2 are both on the left and the other is on the right and fluctuates vertically for several times. The lever | enters the servo calibration mode after the prompt sound, the servo is automatically powered (the legs need to be removed), and the buzzer prompts the servo number by times , Chang the remote lever to change the selection of the steering gear, and fine-tune the steering gear deviation up and down


**note! ! ! ：Since the IMU is not firmly connected to the control board with a flexible wire or is squeezed, SPI communication errors will occur, so when the circuit board is working normally, both module.flash and module.nrf are 1 and
When the controller is placed flat, the original value of acceleration in mems is about [0 0 4096]. If it is abnormal, please power off and reconnect the cable to DEBUG to confirm. In addition, if the circuit board is powered on, it will be more than 90% if it is PX4 boot
The probability is normal. At this time, you only need to check the mems sensor data for errors! ! **

(1) Assemble the leg support structure. First, do not connect the 3D printed parts of the sole to facilitate subsequent leg deviation installation. Assemble the gear arm attached to the steering gear with the 3D printed parts and fix them with 1.25mm screws to prevent slipping. The results are shown in the following figure. Show:
<div align=center><img width="540" height="300" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/a10.JPG"/>< /div>
<br>
(2) Connect the downloader to check whether the power supply of the robot power supply test system is normal, and the DEBUG program will calibrate the deviation of the leg structure: (The firmware above 1.3 supports remote control calibration, please refer to the word document)<br>
a. First add force_dj_off_reset under vmc_demo.c file to watch and set it to 1 in DEBUG to reset the servo deviation. <br>
b. Set vmc_all.sita_test[4] to 1, and the rudder will be powered and turned to 90° under the default deviation. <br>
c. Install the front and rear steering gear rocker arms so that they are level with the body, as shown in the figure below:
<div align=center><img width="650" height="460" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/a11.jpg"/>< /div>
d. Modify the deviation parameter in vmc[i].param.PWM_OFF, [0] corresponds to the angle of the outer steering gear [1] is the inner steering gear. Through adjusting the parameters and visual calibration to ensure that the steering gear is 90°absolutely vertical, the alignment is completed for 4 After all the angles are calibrated, fix the bearing with the supporting screws of the steering gear. <br>
e. Set mems.Gyro_CALIBRATE to 1, and store the current calibration deviation in FLASH. <br>
f. Reset the chip to check whether the read deviation is consistent.
<br>

(3) Calibrate the IMU sensor, place the robot horizontally in the battery compartment for sensor zero-bias calibration, set mems.Gyro_CALIBRATE and mems.Acc_CALIBRATE to 1 respectively to complete the calibration of the gyroscope and accelerometer. <br>
**Note: When calibrating the acceleration, you will find that the posture calculation is 0°, and if there is a deviation, the stability of the robot will deviate to the left or right during the movement. So in the follow-up to some
It is recommended to set only mems.Gyro_CALIBRATE to 1 when the control parameters and servo deviation calibration are performed in FLASH maintenance.**
<br>

(4) Install the plantar 3D printed parts, if there is a plantar sensor connected to the corresponding sensor on the control board, if not, the shock-absorbing plantar will be used by default. Cut the 15*8mm round cushion into a semicircle and stick it side by side to the bottom ,As shown below:
<div align=center><img width="480" height="360" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/leg_end.jpg"/>< /div>

**Note that the oblique tips of the 4 soles should face the nose when installing the sole! ! ! **, finally complete the assembly of the whole machine:
<div align=center><img width="480" height="250" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/a12.jpg"/>< /div>


## 3.4 Introduction of OLDX-Remote
____OLDX-Remote is a universal remote control and data monitor in the OLDX robot development platform. It can be compatible with OLDX-FC flight controllers and can also be used as a remote control for quadruped robots.
In addition to using the somatosensory mode to remotely control the robot, it can also replace the PC ground station for real-time parameter adjustment. In addition, it can return the running status of the SDK main state machine and sub-state machine in real time.
It is expected that the altitude and speed information and route waypoint information can understand the current SDK operating status and the target tasks of the aircraft at the next moment in real time, and solve the problem that the internal status of the robot is difficult to obtain in the autonomous control of the robot.
<div align=center><img width="640" height="300" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/tunning.jpg"/>< /div>

### 3.4.1 Interface Introduction
____OLDX-Remote has three types of interfaces after booting up, (1) Main interface: displays common data such as robot height, posture, voltage and time (2) PID parameter interface and PC terminal ground station correspond to display robot internal parameters (current version is temporarily useless) (3)
SDK interface: display route and autonomous task commands, state machine status. The English abbreviations in each interface are as follows:

Main Interface|Description
-------------|-------------
P|OLDX-Remote pitch remote stick value (somatosensory) 
R|OLDX-Remote Rolling remote stick value (somatosensory)
T|OLDX-Remote throttle stick value (remote stick) **Front and rear control of quadruped robot**
Y|OLDX-Remote heading stick value (remote stick) **Quadruped robot rotation control**
SR|Communication strength between the robot and OLDX-Remote
BR|OLDX-Remote battery
BF|Robot battery (the buzzer will alarm when the battery is low)
M|Robot Mode Mual->Manual  
Bad Pos|Positioning mode status Bad Pos->No positioning GPS->Number of satellites (recommended more than 8 to take off) Opt->Optical flow quality
lock|Robot lock status
P|Pitch angle in rectangular interface
R|Roll angle in rectangular interface
Y|Heading angle in the rectangular interface
0:0|Time in the rectangular interface
X, Y, Z| local X axis position in the rectangular interface (m)
Three square dots in the rectangular interface | From top to bottom * GPS connection * Image device connection * Optical flow module connection
Center circle and solid point | Pixel position of line patrol or visual tracking target

There are 4 DIP switches on the remote control, of which **only No. 1 and No. 2** are useful. By default, they correspond to KEY[0] and KEY[1] in the program. In addition, the remote sensing push button corresponds to KEY_SEL. DIP switch
The left is 1 and the right is 0. The corresponding functions of the DIP switch in the default program are as follows:

DIP switch|Description
-------------|-------------
No. 1: Left No. 2: Right | Attitude and pitch Sin tracking mode
No. 1: Right No. 2: Left|Vehicle mode, the current speed is saved after returning to the center of the positioning speed increment
No. 1: Left No. 2: Left|**Forced Power Off** Used to protect the steering gear from blocking, if it is useless, turn off the battery switch directly


<br><br>

PID parameter interface (**to be updated**)|Description
-------------|-------------
PIDX-X|Corresponding parameters in the host computer
Toggle the remote to feel the bottom 2s to switch the interface|
Select PID parameters left, right, up and down|
Click on the remote control to appear* then select the parameter | select the next to increase 100 parameters, up and down increase 10 parameters
Select and click again to exit the selection—|The remote control reads the flight control data every 5s and does not read when selected 
Select the middle and long press the remote sensing remote control to make a BB sound, and the parameter writes | For the CHE remote control channel, the write needs to go to the main interface to perform the same operation and the remote control IMU will be calibrated

<br><br>

SDK interface (**to be updated**)|Description
-------------|-------------
main|Idle (The default state of the state machine will only run the SDK after the state automatically takes off) Mission (SDK mode) Safe (The state machine protection enters the normal hovering mode and needs to be landed and locked and reset the remote control switch to be cleared)
RC right %|aircraft battery
Subs|Sub State Machine State
Way|Number of remaining waypoints
Left vertical bar | Height expectation and feedback position mode (the horizontal line is expected to be the center of the rectangle) Speed ​​mode (displays the expected speed of ascent and descent)
Rectangular box|The position expectation and feedback center is the current aircraft position, the origin is the desired position (the coordinate system is xy east-north), the coordinate system scale is automatically scaled, and the small horizontal line indicates the aircraft body direction
Dis|Next waypoint distance (m)


## 3.5 Mobile Test
(1) Flat ground test<br>
____ Carry out a movement test on flat ground to verify the gait algorithm logic, forward and rotation speed control and other navigation trajectory control. <br>

(2) Up and down steps ahead<br>
____ Step up from a book or multiple books on the flat ground to verify the ability of attitude control and ground-like following. <br>

(3) Up and down steps on the left and right sides<br>
____ Steps made up of one or more books on the flat ground, ensure that only the left or right legs go up the steps to verify the stability of the posture control of the roll axis. <br>

(4)Up the slope and lower the platform<br>
____A 15°~20° slope on the flat ground to verify the ability of the gait to resist slipping uphill, and directly descend the steps from the top of the slope to verify the different adjustment abilities of the legs suddenly asymmetrically reduced. <br>

(5) Platform posture self-stabilization<br>
____ Place the robot on a tiltable platform, and verify the reliability of attitude control by artificially tilting it. <br>

## 3.6 Use ST-link or punctual atomic wireless downloader to debug
____Use the downloader adapter board to connect to the control board, you need 4P double-head card slot line, USB adapter small board and large board, and a micro USB cable. The results after connection are as follows:
<div align=center><img width="500" height="460" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/download.jpg"/>< /div>

If you use the Punctual Atom wireless downloader, please refer to the instruction pdf file attached to the project. The other two download methods cannot be used at the same time. <br>

**Note: KEIL5 related software and chip library download: **<br>
```
Link: https://pan.baidu.com/s/1UNnMhOecPHUjdAQqanoYLQ 
Extraction code: az23 
```

## 3.7 Height and attitude Sin expectation tracking test
____The most important thing about the basic stability of the foot-type robot is to realize the attitude self-stabilization based on the IMU. By modifying gait_test[i] in the DEBUG mode, it can realize the tracking of the Sin trajectory at a given height and pitch angle.
To verify the reliability of the controller and parameters. By verifying in situ and re-moving, it can be seen that the performance gap of the asynchronous state algorithm can be realized under the premise of only relying on the open loop of the steering gear and no ground sensor.
The optimal gait effect is as follows:<br>
(1) Continuously straddling the legs, the body height can change smoothly<br>
(2) Continuously straddling the legs, the pitch angle can change smoothly<br>
(3) Realize the above process while moving<br>
(4) Verify the attitude balance on the tiltable board, the board tilting robot reacts quickly to ensure the body balance<br>

gait_test parameter|description
-------------|-------------
0|1 height______2 pitch angle______3 moment test
2|Angle increment
3|Amplitude

**Note: The torque test is to verify whether the torque closed loop can ensure the toe tracking the given curve by adjusting vmc_all.pid[Zr][P]/[D] to adapt to different drive systems, and modify UART_UP_LOAD_SEL=1
The expected toe feedback curve can be viewed on the host computer**


## 3.8 Parameter adjustment
### 3.8.1 Introduction to Parameters
Parameter VMC_ALL|Description|Default value
-------------|-------------|-------------
end_sample_dt|Planar velocity differential time interval|0.01
ground_dump|Legs are instantly retracted when they touch the ground, the percentage of full leg length|0.06
trig_ground_st_rate|The percentage of time that the sensor can touch the ground when crossing the leg|0.63
ground_rst|Operation period of self-grounding logic|0.06
k_auto_time|Enable dynamic adjustment of gait period with attitude|0
trig_mode|cross-leg trajectory mode|1
out_range_force|Retraction force when out of legroom|5
use_att|Use attitude control|1
use_ground_sensor|Use ground sensor|0
l1|Calf Length|
l2|Thigh length|
H|Body length|
W|body width|
mess|Total weight of robot|
cog_off|F deviation of the center of gravity when advancing, B backward deviation, 2 forward pitch angle tilt amplitude 3 steering roll angle tilt amplitude|
kp_touch|The cross-leg completion without underground exploration gain|0.3
kp_trig|0 Cross-leg speed error gain|0.125
pid[Xr]|Front and rear speed error PID|
pid[Zr]|Z-axis toe error PID|
pid_att[YAWr]|Heading control PID|

### 3.8.2 Host computer returns data
____If the user has purchased a remote monitor, he can connect to the PC through the virtual USB serial port on it and use the anonymous host computer to refer to the internal parameters of the robot in real time and draw waveforms to speed up parameter adjustment
And the verification of gait algorithm, by setting UART_UP_LOAD_SEL, you can realize the choice of uploading different parameters:

UART_UP_LOAD_SEL|Description
-------------|--------------
0|Landing signal
1|Toe speed and feedback
2|Laser ranging results
3|Heading control
4|Roll axis control
5|Body acceleration
6|Planar pressure value
7|Pressure and ground contact signal
8|High mismatch

In addition, users can send the function in the code and add the corresponding frame protocol to transmit the data they want to display:
```
ucos_task.c 256 lines
			case user-defined:
			data_per_uart_rc(
			Data 1, data 2, data 3,
			Data 4, data 5, data 6,
			Data 7, data 8, data 9,
			(int16_t)(0*10),(int16_t)(0*10.0),(int16_t)(0*10.0),0,0,0/10,0);break;
			default:break;
```		

**Note:** The sent is an integer, so the float parameter needs to be expanded by a multiple of 10 to be displayed normally!

## 3.9 DEMO program optimization and follow-up development suggestions
The ____DEMO program provides users with the simplest and four-legged robot development framework, including posture calculation and core gait algorithms. The provided gait algorithm is based on a virtual model
The torque is output through the Jacobian matrix, and because the steering gear is used, it is only converted to the steering gear rotation speed by a simple PD controller. The entire state machine scheduling of DEMO imaging in terms of gait
The structure is the same as the robot library, except that the attitude control strategy adopts a simpler and more direct force control method, which is easy to understand the most basic control theory of quadruped robots and can be further carried out.
Optimization and improvement. If you want to develop this project in depth and use it in a large robot, it is recommended to further study the SLIP model, impedance control and compliance control theory, and the four-legged robot
More than 60% of the core is reliable servo drive. The efficient servo drive system can achieve reliable movement under the simplest Sin trajectory + CPG algorithm. This has been reported by many papers.
It is proved that the design of the robot leg structure can also provide its own stability to a certain extent. In summary, the follow-up development suggestions are as follows:<br>
(1) Optimize the leg structure to increase passive buffering, build a 12-degree-of-freedom robot, and increase ground sensor feedback<br>
(2) Introduce the robot inverted pendulum model and optimized control and torque control based on the prior knowledge of the model<br>
(3) The replacement drive system adopts brushless direct drive or planetary deceleration structure to improve the consistency of each leg<br>
(4) Introduce visual navigation information to improve the stability of robot movement and the rationality of foot trajectory


# 4 SDK development
____The project adopts the open source + robot control library authorization method, that is, to provide a basic project source code that can run the robot platform and provide a high-performance closed-source robot control library.
The open source program can be modified and the secondary development official will continue to improve and be consistent with the closed source robot library. The closed source robot library reserves a complete calling interface based on the SDK
Quickly control the robot speed, posture, foot trajectory, torque, etc. Read the robot pose fusion data, you can use the current SLAM car chassis development method to carry out the robot
Control, the specific functions of the two are as follows:

**The difference between OLDX_VMC quadruped robot math library and the open source DEMO version:**

Project|Open Source Program|Closed Source Robot Library
-------------|--------------|-------------
Gait Algorithm|Trot|Fly-Tort Tort Wave Gait
Gait Theory|Virtual Model|Virtual Model
Cross-leg trajectory | cycloid | minimum Jerk trajectory ground angle dynamic adjustment
Plantar Sensor|None|Pressure Micro Switch Vibration Sensor Distance Sensor
Image Navigation|SDK Development|SDK Development
Attitude control | virtual model closed loop | position + virtual model closed loop
Speed ​​closed loop|mechanical parameter estimation|optical flow/vision+mechanical parameter estimation
Position closed loop|None|Minimum Jerk trajectory planning + ADRC trajectory tracking
Navigation|Optical Flow UWB GPS|Optical Flow UWB GPS

## 4.1 Get authorization library
____The four-legged robot math library currently only supports STM32F4 series single-chip microcomputers. The math library and the single-chip ID binding can be used forever and enjoy subsequent version library updates.
Before obtaining the math library, you first need to download the source code, search for and provide 3 id corresponding to vmc_all.param.board_id in watch. Provide the ID to the official and obtain the corresponding license
After modifying the following positions in the source code, it can be used normally. Otherwise, the legs will not be powered and planned gait when using vmc.lib:

```
void vmc_init(void)
{char i;
	get_license();
	//Set the license
	vmc_all.param.your_key[0]=143;
	vmc_all.param.your_key[1]=42;
	vmc_all.param.your_key[2]=180;
```		

Note: To use the Demo program, you need to define #define USE_DEMO 1 in vmc.h, otherwise the math library is used by default

## 4.2 Gait test
____The testing method when using the VMC gait library is the same as the DEMO program. Sin trajectory tracking can also be performed to test the improvement of gait performance. In addition, a more practical way is to move and
Uphill and downhill testing, users can build stairs, slopes and obstacles by using books with a high number of gait, verify and test the gait algorithm on the desktop or indoors, or build an arena
PK with friends in the subsequent image autonomous control, as shown in the following figure:

<div align=center><img width="440" height="220" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/playground.jpg"/>< /div>


## 4.3 Parameter adjustment

Parameters|Description|Default Value
-------------|-------------|-------------
att_pid_all[PITr]|Pitch axis PID|i=8 d=0.001
att_pid_all[ROLr]|Roll axis PID|i=10 d=0.002
h_pid_all|Height PID|p=2 i=15 d=4

## 4.4 Use the controller with other quadruped robots
(1) Foot track output<br> 
____ If the steering gear driven robot with other leg configurations is used, it can program itself to convert vmc[i].epos into the corresponding angle and send it to the user's drive controller through serial communication. <br>  
(2) Torque output<br> 
____ If the brushless drive robot with the same configuration is used, the vmc[i].torque torque can be sent to the servo drive via communication as an output, and the torque unit can be matched by adjusting vmc_all.gain_torque.

## 4.5 SDK Development
____Robot development adopts ROS framework robot controller, which will be used as a hardware node to solve the top-level control commands, and control the attitude, speed and position of the center point of the body.
Directly assign value to foot trajectory and moment.
<br>

set_body_spd|Set body speed
-------------|-------------
x|Front and rear speed
y|Left and right speed
z|Up and down speed  
Completion Conditions|None


# 5 Donation and the follow-up development plan of the project
____The team plans to launch a 5kg~10kg-level footed robot development chassis in the later stage, and support RPlidar lidar navigation for SLAM algorithm verification, which can replace the current similar four-wheeled vehicle platforms on the market such as Autolabor at the same price.
 <div align=center><img width="800" height="300" src="https://github.com/golaced/OLDX-FC_QUADRUPED_QUADROTOR/blob/rmd/support_file/img_file1/r1.jpg"/>< /div>
____If you think this project is helpful to you, and for better project promotion and software and hardware updates, please donate the project through WeChat if you want!
<div align=center><img width="440" height="300" src="https://github.com/golaced/OLDX_DRONE_SIM/blob/master/support_file/img_file/pay.png"/></div >
