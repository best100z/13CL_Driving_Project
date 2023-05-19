# -*- coding: utf-8 -*-
"""
Created on Wed May 17 11:17:10 2023

@author: jaspe
"""


import u3
import time
import threading
import queue
import numpy as np


handle1 = u3.U3(serial = "320105375")
handle1.configU3()


handle2= u3.U3(serial="320109684")
handle2.configU3()

handle3=u3.U3(serial = "320109643")
handle3.configU3()


"State Variables"

class SteeringAngle:
    def __init__(self):
        self.angle = 0
        self.direction = 1
    def update_angle(self,angle):
        self.angle = angle

Steering_Angle = SteeringAngle() 


class ScopeAngle:
    def __init__(self):
        self.angle = 0
        self.direction = 1
    def update_angle(self,angle):
        self.angle = angle
        
scopeangle = ScopeAngle()

class AvoidSteps:
    def __init__(self):
        self.turnangle = 0 
        self.turnstraightsteps = 0
        self.straightsteps = 0
    def update_turnangle(self,angle):
        self.turnangle = angle
    def updateanglesteps(self):
        self.turnstraightsteps += 1
    def updatestraightsteps(self):
        self.straightsteps += 1
    def resetstraightsteps(self):
        self.straightsteps = 0
    def resetanglesteps (self):
        self.turnstraightsteps = 0
    def resetangle(self):
        self.turnangle = 0
        
avoidlogic = AvoidSteps()

"Drive Motors"

def setFIODrive(channel, state):
    handle1.setDOState(channel, state);

    
def DriveMotor(Steps = 0, Direction = "Forward"): #360 Degrees = 22cm of movement
    if Direction == "Forward":
        setFIODrive(5, 1)
    if Direction == "Forward":
        setFIODrive(5,0)
    for i in range(Steps):
        setFIODrive(4,1)
        time.sleep(0.01)
        setFIODrive(4, 0)
        time.sleep(0.01)
        i += 1
    
def setFIOhandle2(channel, state):
    handle2.setDOState(channel, state);
    
    
def setFIOsTurn(arg):
    stateList = [int(i) for i in arg]
    setFIOhandle2(4, stateList[0]);
    setFIOhandle2(5, stateList[1]);
    setFIOhandle2(6, stateList[2]);
    setFIOhandle2(7, stateList[3]);
    
def TurnMotor(angle = 0, Direction = "Left"):   
    if Direction == "Left":
        Steering_Angle.update_angle(Steering_Angle.angle-angle)  
    if Direction == "Right":
        Steering_Angle.update_angle(Steering_Angle.angle+angle)  
    if Steering_Angle.angle > 65:
        UhOh()
        print("Too Many Degrees Dont Break the Car")
        Steering_Angle.update_angle(Steering_Angle.angle-angle)
        return
    if Steering_Angle.angle < -65:
        UhOh()
        print("Too Many Degrees Dont Break the Car")
        Steering_Angle.update_angle(Steering_Angle.angle+angle)
        return
    stepAngle = 360/4096*8;
    actualTicks = angle/stepAngle;
    if Direction == "Left":
        while actualTicks>0:
            setFIOsTurn("1100");
            time.sleep(0.0001);
            setFIOsTurn("0110");
            time.sleep(0.0001);
            setFIOsTurn("0011");
            time.sleep(0.0001);
            setFIOsTurn("1001");
            time.sleep(0.0001);        
            actualTicks = actualTicks - 1;
    if Direction == "Right":
        while actualTicks>0:
            setFIOsTurn("1001");
            time.sleep(0.0001);
            setFIOsTurn("0011");
            time.sleep(0.0001);
            setFIOsTurn("0110");
            time.sleep(0.0001);
            setFIOsTurn("1100");
            time.sleep(0.0001);        
            actualTicks = actualTicks - 1;
    setFIOsTurn("0000");

def setFIOhandle3(channel, state):
    handle3.setDOState(channel, state);
    
    
def setFIOsScope(arg):
    stateList = [int(i) for i in arg]
    setFIOhandle3(4, stateList[0]);
    setFIOhandle3(5, stateList[1]);
    setFIOhandle3(6, stateList[2]);
    setFIOhandle3(7, stateList[3]);


def TurnScope(angle = 0, Direction = "Left"):   
    if Direction == "Left":
        scopeangle.update_angle(scopeangle.angle-angle)  
    if Direction == "Right":
       scopeangle.update_angle(scopeangle.angle+angle)  
    stepAngle = 360/4096*8;
    actualTicks = angle/stepAngle;
    if Direction == "Left":
        while actualTicks>0:
            setFIOsScope("1100");
            time.sleep(0.0001);
            setFIOsScope("0110");
            time.sleep(0.0001);
            setFIOsScope("0011");
            time.sleep(0.0001);
            setFIOsScope("1001");
            time.sleep(0.0001);        
            actualTicks = actualTicks - 1;
    if Direction == "Right":
        while actualTicks>0:
            setFIOsScope("1001");
            time.sleep(0.0001);
            setFIOsScope("0011");
            time.sleep(0.0001);
            setFIOsScope("0110");
            time.sleep(0.0001);
            setFIOsScope("1100");
            time.sleep(0.0001);        
            actualTicks = actualTicks - 1;
    setFIOsScope("0000");
    
    
def TurntoAngle(angle):
    current_angle = scopeangle.angle
    final_dest = angle - current_angle
    if final_dest > 0:
        TurnScope(abs(final_dest), "Right")
        return
    else:
        TurnScope(abs(final_dest), "Left")
        return


def ScopeScan():
    angle_increment = 20
    num_steps = 180//angle_increment
    ir_data = np.zeros(num_steps)
    for i in range(num_steps):
        time.sleep(0.1)
        ir_data[i] = VoltagetoDistance(1)
        scopeangle.add_angle(angle_increment)
        TurnScope(angle_increment, "Left")
    TurnScope(180, "Right")
    scopeangle.add_angle(-180)
    min_value = np.min(ir_data)
    min_index = np.argmin(ir_data)
    print(ir_data, min_value, min_index)
    return min_value, min_index


"Queue Commands"
def DriveStraight(cm = 1, Direction = "Forward"):
    if Steering_Angle.angle >0:
        TurnMotor(Steering_Angle.angle, "Left")
    if Steering_Angle.angle <0:
        TurnMotor(abs(Steering_Angle.angle), "Right")
    DriveMotor(cm, Direction)
    
def ReverseTurn90 (Direction= "Right"):
    if Direction == "Right":
        TurnMotor(65, "Left")
        DriveMotor(360, "Backward")
        TurnMotor(130, "Right")
        DriveMotor(760, "Forward")
        TurnMotor(65, "Left")
    if Direction == "Left":
        TurnMotor(65, "Right")
        DriveMotor(360, "Backward")
        TurnMotor(130, "Left")
        DriveMotor(760, "Forward")
        TurnMotor(65, "Right")
        
def DrivingTurn (Distance = 22, Direction = "Right"): #Need 34cm to avoid obstacle using this
    if Direction == "Right":
        TurnMotor(65, "Right")
        DriveMotor(Distance, "Forward")
    if Direction == "Left":
        TurnMotor(65, "Left")
        DriveMotor(Distance, "Forward")
        
        
def ReverseTurn45 (Direction = "Right"):
    if Direction == "Right":
        TurnMotor(65, "Left")
        DriveMotor(180, "Backward")
        TurnMotor(130, "Right")
        DriveMotor(360, "Forward")
        TurnMotor(65, "Left")
    if Direction == "Left":
        TurnMotor(65, "Right")
        DriveMotor(180, "Backward")
        TurnMotor(130, "Left")
        DriveMotor(360, "Forward")
        TurnMotor(65, "Right")
        
def UhOh():   
    return
    
def reset():
    UhOh()
    if Steering_Angle.angle >0:
        TurnMotor(Steering_Angle.angle, "Left")
    if Steering_Angle.angle <0:
        TurnMotor(abs(Steering_Angle.angle), "Right")
        
def InitiateTurn(Direction = "Right"):
    if Direction == "Right":
        TurnMotor(65, "Right")
    if Direction == "Left":
        TurnMotor(65, "Left")

def AvoidObstacle(Angle):
    global stop_flag 
    stop_flag = True
    i = 0
    while True:
        front_dist = VoltagetoDistance(0)
        if front_dist < 100:
            DriveStraight(1,"Forward")
            i += 1
        if front_dist >= 100:
            TurntoAngle(((Angle/abs(Angle)))*90)
            side_dist = VoltagetoDistance(1)
            if side_dist < 100:
                DriveStraight(1,"Forward")
                i += 1
            if side_dist >= 100:
                if Steering_Angle.angle > 0 :
                    TurnMotor(130, "Left")
                    DriveStraight(2*i, "Forward")
                    TurnMotor(65, "Right")
                    DriveStraight(i, "Forward")
                if Steering_Angle.angle < 0:
                    TurnMotor(130, "Right")
                    DriveStraight(2*i, "Forward")
                    TurnMotor(65, "Left")
                    DriveStraight(i, "Forward")
                    TurntoAngle(0)
                break
    stop_flag = False
    sensor_loop()
            
        

"Scan Code"

def getAINhandle1 (n=0):
    ain0bits, = handle1.getFeedback(u3.AIN(n));
    ainValue = handle1.binaryToCalibratedAnalogVoltage(ain0bits, isLowVoltage=False, channelNumber = 0);
    return ainValue

def getAINhandle2 (n=0):
    ain0bits, = handle2.getFeedback(u3.AIN(n));
    ainValue = handle2.binaryToCalibratedAnalogVoltage(ain0bits, isLowVoltage=False, channelNumber = 0);
    return ainValue

def VoltagetoDistance(n=0):
    Voltage = getAINhandle1(n)
    if Voltage > 0.46:
        x = np.log((Voltage-0.46)/3.67)/(-0.068)
        return x
    else:
        x = 100
        return x
    




"Queue"

event_queue = queue.Queue()

stop_flag = False

def sensor_loop():
    front_min = 40 # cm
    diag_min = 40 #cm
    while not stop_flag:
        front_dist = VoltagetoDistance()
        if front_dist >= front_min:
            print("Continuing straight...")
            event_queue.put("Drive")
        else:
            diag_right_dist = VoltagetoDistance(2)
            diag_left_dist = VoltagetoDistance(3)
            diag_left_dist = 10
            diag_right_dist = 10
            print("Stopping the robot...")
            event_queue.put("Stop")
            
            if diag_min <= diag_right_dist and diag_min <= diag_left_dist:
                print("Turning right...")
                event_queue.put("Right Turn")
                break
            elif diag_min <= diag_right_dist:
                print("Turning right...")
                event_queue.put("Right Turn")
                break
            elif diag_min <= diag_left_dist:
                print("Turning left...")
                event_queue.put("Left Turn")
                break
            else:
                array_data = ScopeScan()
                decision_array = [array_data[4],array_data[3],array_data[5],array_data[2],array_data[6],array_data[1],array_data[7],array_data[0],array_data[8]]
                for i, dist in enumerate(decision_array):
                    if front_min <= dist:
                        if i % 2 == 0:
                            if i == 0:
                                print ("straight")
                                event_queue.put("Drive")
                                break
                            else:
                                print(f"Turning to angle {(((i)/2)*(-1)**i) * 20 } degrees...")
                                angle = (((i)/2)*(-1)**i) * 20
                                event_queue.put("Angle Turn", angle)
                                break
                        else:
                            print(f"Turning to angle {(((i+1)/2)*(-1)**i) * 20 } degrees...")
                            break
                    elif i+1 == len(decision_array):
                        print("didn't find acceptable range before middle of array, turn around")
                        break

def avoid_loop(angle, n):
    global stop_flag 
    stop_flag = True
    i = 0
    while True:
       while i < n:
           front_dist = VoltagetoDistance()
           if angle > 0:
               TurnMotor(65, "Right")
               avoidlogic.update_turnangle(65)
           elif angle < 0:
               TurnMotor(65, "Left")
               avoidlogic.update_turnangle(-65)
           if 10 < front_dist < 100:
               DriveMotor(1, "Forward")
               i += 1
               avoidlogic.updateanglesteps()
           if front_dist < 10:
               print("ima kms")
               break
           if front_dist >= 100:
               if angle > 0:
                   TurnScope(65, "Right")
               elif angle < 0:
                   TurnScope(65, "Left")
               side_dist = VoltagetoDistance()
               if side_dist < 100:
                   DriveMotor(1, "Forward")
                   i += 1
               avoidlogic.updateanglesteps()    
               if side_dist >= 100:
                   event_queue.put("RtC")
       while i == n: 
           if Steering_Angle.angle > 0:
               TurnMotor(65, "Left")
               TurntoAngle(-90)
           if Steering_Angle.angle < 0:
               TurnMotor(65, "Right")
               TurntoAngle(65)
           side_dist = VoltagetoDistance()  
           if side_dist < 100:
               DriveMotor(1, "Forward")
               avoidlogic.updatestraightsteps()
           if side_dist >= 100:
               event_queue.put("RtC")
                      
                  
        
def RtC():
    while True:
        front_dist = VoltagetoDistance()
        if front_dist >= 100:
            if avoidlogic.turnangle > 0 :
                TurnMotor(2*avoidlogic.turnangle, "Left")
            else:
                TurnMotor(2*avoidlogic.turnangle, "Right")
            DriveMotor(2*avoidlogic.turnstraightsteps, "Forward")
            DriveMotor(1*avoidlogic.straightsteps, "Forward")
            if avoidlogic.turnangle > 0 :
                TurnMotor(2*avoidlogic.turnangle, "Right")
            else:
                TurnMotor(2*avoidlogic.turnangle, "Left")
            DriveMotor(avoidlogic.turnstraightsteps, "Forward")
            TurntoAngle(0)
            reset()
            avoidlogic.resetanglesteps()
            avoidlogic.resetanglesteps()
            avoidlogic.resetangle()
            break
        if front_dist < 100:
            print("Ima kms")
            UhOh()
    global stop_flag
    stop_flag = False
    sensor_loop()
    return
            
def event_loop():
    while True:
        event = event_queue.get()   
        if event == "Drive":
            DriveStraight(1, "Forward")
        if event == "Stop":
            UhOh()
        if event == "Right Turn":
            InitiateTurn(90, "Right")
            time.sleep(0.5)
            AvoidObstacle()
        if event == "Left Turn":
            InitiateTurn(90, "Left")
            time.sleep(0.5)
            AvoidObstacle()
        if event [0]== "Angle Turn":
            angle = event[1]
            avoid_loop(angle)
        if event == "RtC":
            RtC()
                

sensor_thread = threading.Thread(target=sensor_loop)
event_thread = threading.Thread(target=event_loop)




def GOGOGO():
    sensor_thread.start()
    event_thread.start()

def Stop():
    sensor_thread.stop()
    event_thread.stop()
    reset()
    
    
    
    

