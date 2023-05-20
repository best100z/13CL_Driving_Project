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


handle1 = u3.U3(serial = "320105375") #Attach to Motor
handle1.configU3()


handle2= u3.U3(serial="320109684") #attach to steering
handle2.configU3()

handle3=u3.U3(serial = "320109643") #attach to scope
handle3.configU3()


"State Variables"

class SteeringAngle: #Super simple class, all it does is record the turn angle of the steering. This is exclusively used to prevent the steering from going anymore than 65 degrees in either direction so the robot cant break itself.
    def __init__(self):
        self.angle = 0
        self.direction = 1
    def update_angle(self,angle):
        self.angle = angle

Steering_Angle = SteeringAngle() 


class ScopeAngle: #Keeps track of the angle of the scope relative to the car, allows "TurntoAngle()" to work, as that relies on the current scope angle
    def __init__(self):
        self.angle = 0
        self.direction = 1
    def update_angle(self,angle):
        self.angle = angle
    def add_angle(self, angle):
        self.angle += angle
        
scopeangle = ScopeAngle()

class AvoidSteps: #This is my return to og path class, in theory it keeps track of the angle the wheel was turned, how many steps it takes with the wheel turned, and how many steps it takes with the wheel straight. The turn angle is very likely unecessary. 
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

def setFIODrive(channel, state): #Labjack specific
    handle1.setDOState(channel, state);

    
def DriveMotor(Steps = 0, Direction = "Forward"): #'drive' means this command controls the driving motor. Needs power to both DIR and STEP pin on DRV8825, will turn the motor by step, not by distance
    if Direction == "Forward":
        setFIODrive(5, 1)
    if Direction == "Backward":
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
    
def TurnMotor(angle = 0, Direction = "Left"): #the 'Turn' refers to the function of this motor. Class updates automatically so wont break the car. 
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


def TurnScope(angle = 0, Direction = "Left"):#same deal, this will turn the scope to as many degrees in a direciton that you give it. It wont turn to a specific angle
    if Direction == "Right":
        scopeangle.update_angle(scopeangle.angle+angle)  
    if Direction == "Left":
       scopeangle.update_angle(scopeangle.angle-angle)  
    stepAngle = 360/4096*8;
    actualTicks = angle/stepAngle;
    if Direction == "Right":
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
    if Direction == "Left":
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
    
    
def TurntoAngle(angle): #This code turns the scope to a specific angle relative to the car
    current_angle = scopeangle.angle
    final_dest = angle - current_angle
    if final_dest > 0:
        TurnScope(abs(final_dest), "Right")
        return
    else:
        TurnScope(abs(final_dest), "Left")
        return


def ScopeScan(): #This code will take an array of data in a 20 degree range. I think it can be imporved by using TurntoAngle(), as this array isnt great in terms of where the scope is pointing as there is no perfect middle measurement
    angle_increment = 20
    num_steps = 180//angle_increment
    ir_data = np.zeros(num_steps)
    TurnScope(90, "Left")
    scopeangle.add_angle(-90)
    for i in range(num_steps):
        time.sleep(0.1)
        ir_data[i] = VoltagetoDistance(1)
        scopeangle.add_angle(angle_increment)
        TurnScope(angle_increment, "Right")
    TurnScope(90, "Left")
    scopeangle.add_angle(-90)
    min_value = np.min(ir_data)
    min_index = np.argmin(ir_data)
    print(ir_data, min_value, min_index)
    return ir_data


"Queue Commands"

#All of the commented out stuff is pretty useless. Not only did it have no use orignally, with the new motor and proportions of the car everything is SUPER irrelevant. I keep it just for inspiration if anything needs to be rewritten.
"""
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
"""
        
def UhOh():  #Turns off all the motor so nothing overheats. Make sure to run it after a keyboard interrupt
    setFIOsScope("0000")
    setFIOsTurn("0000")
    return
    
def reset(): #UhOh, then will reset the turning to centerline
    UhOh()
    if Steering_Angle.angle >0:
        TurnMotor(Steering_Angle.angle, "Left")
    if Steering_Angle.angle <0:
        TurnMotor(abs(Steering_Angle.angle), "Right")
  
    #This was an early attempt at pathfinding, probably unecessary, as I like avoid_loop() a lot more and i think it can be applied to every situation
"""        
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
            
"""        

"Scan Code"

def getAINhandle1 (n=0): 
    ain0bits, = handle1.getFeedback(u3.AIN(n));
    ainValue = handle1.binaryToCalibratedAnalogVoltage(ain0bits, isLowVoltage=False, channelNumber = 0);
    return ainValue

def getAINhandle3 (n=0):
    ain0bits, = handle3.getFeedback(u3.AIN(n));
    ainValue = handle3.binaryToCalibratedAnalogVoltage(ain0bits, isLowVoltage=False, channelNumber = 0);
    return ainValue

def VoltagetoDistance(n=0):
    Voltage = getAINhandle3(n)
    if Voltage > 0.46:
        x = np.log((Voltage-0.46)/3.67)/(-0.068)
        return x
    else:
        x = 100
        return x
    

#last three funcitons just call AIN, VoltagetoDistance will convert that measurement to distance using Spencers data. The if statement is to prevent the function from returning nan for a logarithim that isnt real valued. Essentially, if this function returns 100 there is nothing there. You will see a lot of if statements using 100 for this reason.


"Queue"

event_queue = queue.Queue() #defines the Queue variable

stop_flag = False #This stopflag is used to turn off the sensor loop whenever something like avoid_loop() is called

def sensor_loop(): #This is the main sensing decision making code
    front_min = 45 # cm Minimum values telling the car when to stop
    diag_min = 45 #cm Minimum values for the diagonal turns. These are the two sensors pointing off to the sides
    global stop_flag
    while not stop_flag: #do this loop when the stop flag is false
        front_dist = VoltagetoDistance(0) #scan the front 
        if front_dist >= front_min: #if we still have space drive forward
            print("Continuing straight...") #for me to know what the robot is thinking
            event_queue.put("Drive") #passes drive to the queue loop, see the queue loop for more detail
            time.sleep(0.1) #This sleeps the code to give the motor time to do its thing. If it doesnt sleep it will make a massive queue of drives, so when the stop command comes its so late that the robot will just crash lol
        else: #front sensor isnt happy anymore. Passes to next layer of decision making.
            #diag_right_dist = VoltagetoDistance(2) #These would be the AIN readings from the left and right sensons once mounted, respectively
            #diag_left_dist = VoltagetoDistance(3)
            diag_left_dist = 10 #I have these set arbitrarily to always trigger the next loop where the scanner decides
            diag_right_dist = 10
            print("Stopping the robot...")
            event_queue.put("Stop") 
            
            if diag_min <= diag_right_dist and diag_min <= diag_left_dist: #If both left and right look okay, arbitrarily go right
                print("Turning right...")
                event_queue.put("Right Turn")
                break
            elif diag_min <= diag_right_dist: #If right is good, go right
                print("Turning right...")
                event_queue.put("Right Turn")
                break
            elif diag_min <= diag_left_dist: #If left is good, go left
                print("Turning left...")
                event_queue.put("Left Turn")
                break
            else: #If neither is good, go onto the scanning loop
                array_data = ScopeScan() 
                decision_array = [array_data[4],array_data[3],array_data[5],array_data[2],array_data[6],array_data[1],array_data[7],array_data[0],array_data[8]] #This will take the IR data and list it in a way that goes center, then direcetly to the right of center, directly to the left, second from the right and so on. This allows the robot to pick the most optimal path
                for i, dist in enumerate(decision_array):
                    if front_min <= dist:
                        if i % 2 == 0: #This modulo is essentially saying right or left. All odd arrays correspond to right turns, all even correspond to left turns
                            if i == 0: #This is because the scanner is behind the front IR sensor. Sometimes, the scanner will read an ok value in front, when in reality there isnt. Therefore, the code just ignores it.
                                continue 
                            else:
                                print(f"Turning to angle {(((i)/2)*(-1)**i) * 20 } degrees...") #This tells the angle of the turn, measured from centerline. Negative values correspond to left turns
                                angle = (((i)/2)*(-1)**i) * 20
                                event_queue.put("Angle Turn", angle) #I want to pass both a queue event and a variable to an event queue, but it doesnt work just yet.
                                stop_flag = True #This will stop the scanning queue, allowing a different loop to take over
                                break
                        else: #same as above but for the other direction
                            print(f"Turning to angle {(((i+1)/2)*(-1)**i) * 20 } degrees...")
                            angle = (((i)/2)*(-1)**i) * 20
                            event_queue.put("Angle Turn", angle)
                            stop_flag = True
                            break
                    elif i+1 == len(decision_array): #I havent coded this yet, but this is the option for going backwards cause the robot messed up
                        print("didn't find acceptable range before middle of array, turn around")
                        break

def avoid_loop(angle):
    i = 0 #This i value is used to make sure that once the car is turned 90 degrees itll stop turning and just go straight
    while True:
       if angle > 0: #These just turn the car in the right direction 
           TurnMotor(65, "Right")
           avoidlogic.update_turnangle(65)
       elif angle < 0:
           TurnMotor(65, "Left")
           avoidlogic.update_turnangle(-65)
       while i < 500: #So while the car isnt turned 90 yet
           print("Starting to Turn")
           front_dist = VoltagetoDistance(0)
           if 10 < front_dist < 100: #Drive forward, ie turn
               DriveMotor(1, "Forward")
               i += 1
               avoidlogic.updateanglesteps() #This is keeping track of the amount of steps that the car has turned at an angle, used to retrace later
           if front_dist < 10: #If in this process the robot gets to close to an obstace the robot gets sad. We can code something here to have it reverse or something
               print("ima kms")
               break
           if front_dist >= 100: #Once the car has turned far enough that the front is clear, we now will drive until we are fully past the obstace. NOTHING FROM HERE ON HAS BEEN TESTED AND FUNCITONAL
               print("Checking the Hip")
               if angle > 0:
                   TurnScope(abs(angle), "Left") #This turns the scope to look at the obstace, and measure if the side is clear. 
               elif angle < 0:
                   TurnScope(abs(angle), "Right")
               side_dist = VoltagetoDistance(1)
               if abs(scopeangle.angle) > 0: 
                   front_dist = VoltagetoDistance(0)
                   if side_dist < 100: #If the side isnt clear, keep turning the car
                       DriveMotor(1, "Forward")
                       i += 1
                       avoidlogic.updateanglesteps()    
                   if side_dist >= 100: #Now that the scanner cant see anything, it passes the return to course loop. I have no clue if this will even work
                       print("returning to course")
                       event_queue.put("RtC")
       while i == 500:  #Once the car has turned 90 degress
           print("made it to 90")
           if Steering_Angle.angle > 0: #This two commands will turn the steering straight, then will turn the scope to look directly to the side where the obstacle would be
               TurnMotor(65, "Left")
               TurntoAngle(-90)
           if Steering_Angle.angle < 0:
               TurnMotor(65, "Right")
               TurntoAngle(65)
           side_dist = VoltagetoDistance()   
           if side_dist < 100: #Drive until the scope doesnt see anything on the side
               DriveMotor(1, "Forward")
               avoidlogic.updatestraightsteps() #Keep track of the amount of steps taken directly straight, used for return to course
           if side_dist >= 100:
               event_queue.put("RtC") #Side is clear, go back to the og course
               print("returning to course")
                      
                  
        
def RtC(): #I think there is absolutely no reason to explore this code, as it will change by wednesday when we get the new parts. For now, it will just retrace the steps used to avoid the obstaclce, but in reverse so it will go around the obstacle. I dont think we need to care about it, everything higher up is more of a priority to get working,.
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
            avoidlogic.resetanglesteps() #these three erase all the avoid steps taken, so that if another obstacle comes up it will start keeping new data for the steps taken to avoid
            avoidlogic.resetanglesteps()
            avoidlogic.resetangle()
            break
        if front_dist < 100:
            print("Ima kms")
            UhOh()
    global stop_flag
    stop_flag = False #This turns the stop flag back to false, meaning the car has avoided the pbstacle and is returning to normal functionality.
    sensor_loop()
    return
            
def event_loop():
    while True:
        event = event_queue.get()   
        if event == "Drive":
            DriveMotor(1, "Forward")
        if event == "Stop":
            UhOh()
        if event == "Right Turn":  #The right turn and left turn is using the older avoid code, I think we should just switch it to the avoid loop function
            InitiateTurn(90, "Right")
            time.sleep(0.5)
            AvoidObstacle()
        if event == "Left Turn":
            InitiateTurn(90, "Left")
            time.sleep(0.5)
            AvoidObstacle()
        if event == "Angle Turn": #This doesnt work yet. Need to find a way to actually queue it.
            print("Going to avoid loop")
            angle = event[1]
            avoid_loop(angle)
        if event == "RtC":
            RtC()
                

sensor_thread = threading.Thread(target=sensor_loop)
event_thread = threading.Thread(target=event_loop)




def GOGOGO(): #This starts the streaming of both the event queue and streaming
    sensor_thread.start()
    event_thread.start()

def Stop(): #This just doesnt work at all lol
    sensor_thread.stop()
    event_thread.stop()
    reset()
    
    
    
    

