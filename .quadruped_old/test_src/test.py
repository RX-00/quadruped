#!/usr/bin/env python
# -*- coding: utf-8 -*-

'''# just some test code I found, testing it to see what's up with the serial port'''
import maestro
import time

#define servo

#servo = maestro.Controller(ttyStr='/dev/ttyACM0')
#min_imp = 2000
#max_imp = 10000
#step_imp = 100
'''
#infinite loop
while 1:
    #user input
    msg = input("Enter servo id and command: YxZZZZ: \n")
    #convert msg into id and cmd
    sep = msg.find("x")
    m1 = msg[:sep]
    m2 = msg[sep+1:]
    servoId = int(m1)
    servoCmd = int(m2)
    #saturate input
    servoId = min(max(0, servoId), 6);
    servoCmd = min(max(min_imp, servoCmd), max_imp);
    print(msg)
    print("servoId : {}     servoCmd : {} \n".format(servoId, servoCmd))
    servo.setTarget(servoId, servoCmd)
    time.sleep(0.1)
servo.close
'''

# my own crap I'm testing out:
#for i in range(0, 19):
#    print("servo: {}     servo pos: {} \n".format(i, 1600))
#    servo.setTarget(i, 2000)
#    time.sleep(1)

#servo.close

PORT = 0
servo = maestro.Controller()
servo.setAccel(PORT, 0)
servo.setSpeed(PORT, 0)
servo.setTarget(PORT, 4000)

# test ports 0 - 18
for j in range(0, 19):
    PORT = j
    for i in range(0, 10):
        tarPos = i * 1000
        servo.setTarget(PORT, tarPos)
        print("servo: {}    servo target: {} \n".format(PORT, tarPos))
        #print("servo: {}    servo pos: {} \n".format(PORT, servo.getPosition(6)))
        #servo.setTarget(0, tarPos)
        #print("servo: {}    servo target: {} \n".format(0, tarPos))
        #print("servo: {}    servo pos: {} \n".format(0, servo.getPosition(6)))
        time.sleep(0.5)

servo.close()

