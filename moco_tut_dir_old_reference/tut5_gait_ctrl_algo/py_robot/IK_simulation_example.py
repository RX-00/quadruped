#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb 20 15:16:47 2020

@author: linux-asd
"""

import pybullet as p
import numpy as np
import time
import pybullet_data
import IK_solver as IK
import geometrics as geo
        
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)

p.loadURDF("plane.urdf")
cubeStartPos = [0,0,0.2]
boxId = p.loadURDF("C:/Users/golaced/Desktop/py_robot/4leggedRobot.urdf",cubeStartPos, useFixedBase=False)
jointIds = []
paramIds = [] 

for j in range(p.getNumJoints(boxId)):
#    p.changeDynamics(boxId, j, linearDamping=0, angularDamping=0)
    info = p.getJointInfo(boxId, j)
    print(info)
    jointName = info[1]
    jointType = info[2]
    jointIds.append(j)
    
footFR_index = 3
footFL_index = 7
footBR_index = 11
footBL_index = 15   


#Camera paramers to be able to yaw pitch and zoom the camera (Focus remains on the robot) 
cyaw=90
cpitch=-7
cdist=0.66
time.sleep(0.5)
#OCU界面读取
xId = p.addUserDebugParameter("x" , -0.10 , 0.10 , 0.)
yId = p.addUserDebugParameter("y" , -0.10 , 0.10 , 0.)
zId = p.addUserDebugParameter("z" , -0.10 , 0.10 , 0.)
rollId = p.addUserDebugParameter("roll" , -np.pi/4 , np.pi/4 , 0.)
pitchId = p.addUserDebugParameter("pitch" , -np.pi/4 , np.pi/4 , 0.)
yawId = p.addUserDebugParameter("yaw" , -np.pi/4 , np.pi/4 , 0.)
Kp_att = p.addUserDebugParameter("Kp_att" , 0 , 10 , 1)
Flt_att_param = p.addUserDebugParameter("Flt_att_param" , 0 , 1 , 0)
#robot properties
"""initial safe position"""
targetAngs = np.matrix([0 , np.pi/4 , -np.pi/2, 0 ,#BR  # 侧摆 大腿  小腿
                        0 , np.pi/4 , -np.pi/2, 0 ,#BL
                        0 , np.pi/4 , -np.pi/2, 0 ,#FL
                        0 , np.pi/4 , -np.pi/2, 0 ])#FR
#FR_0  to FR_4 initial vectors
#FRcoord = np.matrix([0. , -3.6 , -0.15])
#FLcoord = np.matrix([0. ,  3.6 , -0.15])
#BRcoord = np.matrix([0. , -3.6 , -0.15])
#BLcoord = np.matrix([0. ,  3.6 , -0.15])

orientation = np.matrix([0. , 0. , 0.])
position = np.matrix([0. , 0. , 0.])

"""IS units (m,kg,rad...) """
L = 0.19 #length of robot joints  长度
W = 0.11 #width of robot joints   宽度

"initial foot position"
#distance to floor
Ydist = 0.182 #foot separation on y axis (0.182 -> tetta=0)
Xdist = L
height = 0.15 #distance to floor

#body_frame to coxa_frame vector  #跨关节相对机体位置
bodytoFR0 = np.matrix([ L/2, -W/2 , 0])#相对质心坐标/2
bodytoFL0 = np.matrix([ L/2,  W/2 , 0])
bodytoBR0 = np.matrix([-L/2, -W/2 , 0])
bodytoBL0 = np.matrix([-L/2,  W/2 , 0])
 
#body_frame to foot_frame vector  #足底相对机体位置  机体中心[0 ,0 ,0]
bodytoFR4 = np.matrix([ Xdist/2 , -Ydist/2 , -height])
bodytoFL4 = np.matrix([ Xdist/2 ,  Ydist/2 , -height])
bodytoBR4 = np.matrix([-Xdist/2 , -Ydist/2 , -height])
bodytoBL4 = np.matrix([-Xdist/2 ,  Ydist/2 , -height])



USE_ATT = 1
cnt1=0
orn_c = [0,0,0]
attitude_flt= [0,0,0]
FLT_ATT = 0.015
while(True):
    #传感器参数
    cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
    attitude = p.getEulerFromQuaternion (cubeOrn)
    #attitude = geo.q_to_eular(cubeOrn)

    p.resetDebugVisualizerCamera( cameraDistance=cdist, cameraYaw=cyaw, cameraPitch=cpitch, cameraTargetPosition=cubePos)
    keys = p.getKeyboardEvents()
    #按键操作
    if keys.get(100):  #D
        cyaw+=1
    if keys.get(97):   #A
        cyaw-=1
    if keys.get(99):   #C
        cpitch+=1
    if keys.get(102):  #F
        cpitch-=1
    if keys.get(122):  #Z
        cdist+=.01
    if keys.get(120):  #X
        cdist-=.01
    if keys.get(27):  #ESC
        break
    #从界面读取参数
    KP = p.readUserDebugParameter(Kp_att)
    FLT_ATT= p.readUserDebugParameter(Flt_att_param)
    pos = [p.readUserDebugParameter(xId),p.readUserDebugParameter(yId), p.readUserDebugParameter(zId)]
    orn = [p.readUserDebugParameter(rollId),p.readUserDebugParameter(pitchId), p.readUserDebugParameter(yawId)]
    undoPos = [-p.readUserDebugParameter(xId), -p.readUserDebugParameter(yId), -p.readUserDebugParameter(zId)]
    undoOrn = [-p.readUserDebugParameter(rollId), -p.readUserDebugParameter(pitchId), -p.readUserDebugParameter(yawId)]
    
    target_att = orn

    for i in range(3) :
        if FLT_ATT==0 :
            attitude_flt[i]=attitude[i]
        else :
            attitude_flt[i]=attitude_flt[i]+(attitude[i]-attitude_flt[i])*FLT_ATT

    err_att = [target_att[0]-attitude_flt[0],target_att[1]-attitude_flt[1],target_att[2]-attitude_flt[2]]

    orn_c[0] = orn_c[0]+err_att[0]*KP*0.005
    orn_c[1] = orn_c[1]+err_att[1]*KP*0.005
    orn_c[2] = orn[2]
    undoOrn_c = [-attitude_flt[0],-attitude_flt[1],-orn[2]*0]

    cnt1=cnt1+1
    if cnt1>20:
        cnt1=0
        print("attitude:",attitude_flt," orn: ",orn)
      
    if USE_ATT:
      orn=orn_c
      undoOrn=undoOrn_c

    "defines the 4 vertices which rotates with the body"  #使用RT将机体跨关节转换到全局坐标
    _bodytoFR0 = geo.transform(bodytoFR0 ,  orn, pos)
    _bodytoFL0 = geo.transform(bodytoFL0 ,  orn, pos)
    _bodytoBR0 = geo.transform(bodytoBR0 ,  orn, pos)
    _bodytoBL0 = geo.transform(bodytoBL0 ,  orn, pos)
    "defines coxa_frame to foot_frame leg vector neccesary for IK"#跨关节全局向量减去足端向量=全局坐标系下足到跨的向量
    FRcoord = bodytoFR4 - _bodytoFR0
    FLcoord = bodytoFL4 - _bodytoFL0
    BRcoord = bodytoBR4 - _bodytoBR0
    BLcoord = bodytoBL4 - _bodytoBL0
    "undo transformation made on the body to keep foot still"#用RT将该向量转换到机体
    _FRcoord = geo.transform(FRcoord , undoOrn, undoPos)
    _FLcoord = geo.transform(FLcoord , undoOrn, undoPos)
    _BRcoord = geo.transform(BRcoord , undoOrn, undoPos)
    _BLcoord = geo.transform(BLcoord , undoOrn, undoPos)
    "solves the angles needed to reach the desire foot position(in the leg_frame)"#计算在各腿下对应关节角度
    FR_angles = IK.solve_R(_FRcoord)
    FL_angles = IK.solve_L(_FLcoord)
    BR_angles = IK.solve_R(_BRcoord)
    BL_angles = IK.solve_L(_BLcoord)
    
    #move the movable joints  发送舵机角度
    for i in range(0, footFR_index):
        targetAngs[0,i] = FR_angles[0,i - footFR_index]
        p.setJointMotorControl2(boxId, i, p.POSITION_CONTROL, targetAngs[0,i])
    for i in range(footFR_index + 1, footFL_index):
        targetAngs[0,i] = FL_angles[0,i - footFL_index]
        p.setJointMotorControl2(boxId, i, p.POSITION_CONTROL, targetAngs[0,i])
    for i in range(footFL_index + 1, footBR_index):
        targetAngs[0,i] = BR_angles[0,i - footBR_index]
        p.setJointMotorControl2(boxId, i, p.POSITION_CONTROL, targetAngs[0,i])
    for i in range(footBR_index + 1, footBL_index):
        targetAngs[0,i] = BL_angles[0,i - footBL_index]
        p.setJointMotorControl2(boxId, i, p.POSITION_CONTROL, targetAngs[0,i])
        
        
    p.stepSimulation()
    time.sleep(1./500.)
p.disconnect()


