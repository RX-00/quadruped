#!/usr/bin/env python

import maestro


servo1 = maestro.Controller() # create servo object

servo1.setAccel(0,4)      # set servo 0 acceleration to 4
servo1.setTarget(0,6000)  # set servo to move to center position
servo1.setSpeed(1,10)     # set speed of servo 1
x = servo1.getPosition(1) # get the current position of servo 1

print('Servo 1 pos: ', x) # print current position of servo 1

servo1.close()
