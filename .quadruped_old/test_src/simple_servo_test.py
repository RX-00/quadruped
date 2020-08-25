#!/usr/bin/env python

import maestro

PORT_0 = 0

servo = maestro.Controller() # create servo object

for i in range(0, 18):
    servo.setAccel(i, 4)            # set servo # acceleration to 4
    servo.setTarget(i, 6000)        # set servo # to move to center position
    servo.setSpeed(i, 10)           # set speed of servo #
    x = servo.getPosition(1)        # get the current position of servo #
    printf('Servo %d pos: %d', i, x) # print current position of servo 1

servo.close()
