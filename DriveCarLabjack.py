# -*- coding: utf-8 -*-
"""
Created on Wed Apr 26 13:50:17 2023

@author: jaspe
"""
import u3


handle1 = u3.U3()
handle1.configU3()


handle2= u3.U3(serial="320109684")
handle2.configU3()



import u3
import time

##d = u3.U3();
##d.configIO(FIOAnalog = 0);


class SteeringAngle:
    def __init__(self):
        self.angle = 0
        self.direction = 1
    def update_angle(self,angle):
        self.angle = angle

Steering_Angle = SteeringAngle() 

"Turning Code"
def setFIO(channel, state):
    handle1.setDOState(channel, state);

def setFIOs(arg):
    stateList = [int(i) for i in arg]
    setFIO(4, stateList[0]);
    setFIO(5, stateList[1]);
    setFIO(6, stateList[2]);
    setFIO(7, stateList[3]);
    
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
            setFIOs("1100");
            time.sleep(0.0001);
            setFIOs("0110");
            time.sleep(0.0001);
            setFIOs("0011");
            time.sleep(0.0001);
            setFIOs("1001");
            time.sleep(0.0001);        
            actualTicks = actualTicks - 1;
    if Direction == "Right":
        while actualTicks>0:
            setFIOs("1001");
            time.sleep(0.0001);
            setFIOs("0011");
            time.sleep(0.0001);
            setFIOs("0110");
            time.sleep(0.0001);
            setFIOs("1100");
            time.sleep(0.0001);        
            actualTicks = actualTicks - 1;
    setFIOs("0000");
    
    
'Driving Code'    
def setFIODrive(channel, state):
    handle2.setDOState(channel, state);

def setFIOsDrive(arg):
    stateList = [int(i) for i in arg]
    setFIODrive(4, stateList[0]);
    setFIODrive(5, stateList[1]);
    setFIODrive(6, stateList[2]);
    setFIODrive(7, stateList[3]);
    
def DriveMotor(angle = 0, Direction = "Forward"): #360 Degrees = 22cm of movement
    stepAngle = 360/4096*8;
    actualTicks = angle/stepAngle;
    if Direction == "Backward":
        while actualTicks>0:
            setFIOsDrive("1100");
            time.sleep(0.0001);
            setFIOsDrive("0110");
            time.sleep(0.0001);
            setFIOsDrive("0011");
            time.sleep(0.0001);
            setFIOsDrive("1001");
            time.sleep(0.0001);        
            actualTicks = actualTicks - 1;
    if Direction == "Forward":
        while actualTicks>0:
            setFIOsDrive("1001");
            time.sleep(0.0001);
            setFIOsDrive("0011");
            time.sleep(0.0001);
            setFIOsDrive("0110");
            time.sleep(0.0001);
            setFIOsDrive("1100");
            time.sleep(0.0001);        
            actualTicks = actualTicks - 1;
    setFIOsDrive("0000");



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
        DriveMotorCM(Distance, "Forward")
        TurnMotor(65, "Left")
    if Direction == "Left":
        TurnMotor(65, "Left")
        DriveMotorCM(Distance, "Forward")
        TurnMotor(65, "Right")
        
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
    setFIOsDrive("0000")  
    setFIOs("0000")
    
def reset():
    UhOh()
    if Steering_Angle.angle >0:
        TurnMotor(Steering_Angle.angle, "Left")
    if Steering_Angle.angle <0:
        TurnMotor(abs(Steering_Angle.angle), "Right")
    
    
def DriveMotorCM (cm = 0, Direction = "Forward"): #360 Degrees = 22cm of movement
    stepAngle = 360/4096*8;
    angle = (cm/22)*360
    actualTicks = angle/stepAngle;
    if Direction == "Backward":
        while actualTicks>0:
            setFIOsDrive("1100");
            time.sleep(0.0001);
            setFIOsDrive("0110");
            time.sleep(0.0001);
            setFIOsDrive("0011");
            time.sleep(0.0001);
            setFIOsDrive("1001");
            time.sleep(0.0001);        
            actualTicks = actualTicks - 1;
    if Direction == "Forward":
        while actualTicks>0:
            setFIOsDrive("1001");
            time.sleep(0.0001);
            setFIOsDrive("0011");
            time.sleep(0.0001);
            setFIOsDrive("0110");
            time.sleep(0.0001);
            setFIOsDrive("1100");
            time.sleep(0.0001);        
            actualTicks = actualTicks - 1;
    setFIOsDrive("0000");
