# -*- coding: utf-8 -*-
"""
Created on Tue May 30 09:57:35 2023

@author: jaspe
"""

import numpy as np
import RPi.GPIO as gpio
import u3
import time
import threading
import queue



"""
work everything in a class
"""

class piRobot():
  def __init__(self):
    gpio.setmode(gpio.BCM)
    gpio.setwarnings(False)
    for i in range(28):
      gpio.setup(i, gpio.OUT)
    self.pinStates = np.zeros(28)
    self.irangle = 0 ###This is the angle IR sensor is facing
    self.destination = np.array([0,0]) ###(x,y) coordinate as the robot is going to move to
    self.heading = 0 ###the direction the car is driving relative to +x-axis; -is to the left, +is to the right
    self.labjack = u3.U3()
    self.labjack.configU3()    
    self.angle = 0  ###angle for the front wheel, it only turns to 65 degree to either left or right; act as a protection and as a reminder when forget to return the front wheel to orignial position
    self.position = np.array([0,0]) ###This variable updates as the car drives.
    """
    skeptical if we need to keep it
    """
    #self.sensor_thread = threading.Thread(target=self.sensor_loop)
    #self.event_thread = threading.Thread(target=self.event_loop)
    #self.event_queue = queue.Queue() #defines the Queue variable

  """
  getDsitance
  """
  def getDistance(self):
    sqr = self.destination * self.destination;
    return (np.sum(sqr))**(1/2);

  """
  Casual setting functions
  """
  
  def update_angle(self, angle):
    self.angle = angle
      
  def update_irangle(self, angle):
    self.irangle = angle
    
  def update_destination(self, x, y):
    self.destination = [x,y]

  def update_heading(self, angle):
    self.heading = angle


  
  """
  pinOnOff:
  input: array of pins
  of the array of pins, turn it on if off, turn it off if on
  """
  def pinOnOff(self, numbers):
    for number in numbers:
      if self.pinStates[number]==0:
        gpio.output(number, gpio.HIGH)
        self.pinStates[number]=1
      else:
        gpio.output(number, gpio.LOW)
        self.pinStates[number]=0
  
  """
  UhOh:
  turn all the pins off
  """
        
  def UhOh(self):   
    for i in range(28):
      if self.pinStates[i] == 1:
        self.pinOnOff(i)
        
  """
  TurnMotor(Front Wheel)
  
  this code is only used to turn the front wheel to either 65 degrees to the left or to the right
  
  *** has protection for the angle so it doesn't break the front motor
  """
  
  def TurnMotor(self, angle = 0, Direction = "Left"):   
    if Direction == "Left":
        self.update_angle(self.angle-angle)  
    if Direction == "Right":                                   
        self.update_angle(self.angle+angle)  
    if self.angle > 65:
        self.UhOh()
        print("Too Many Degrees Dont Break the Car")
        self.update_angle(self.angle-angle)
        return
    if self.angle < -65:
        self.UhOh()
        print("Too Many Degrees Dont Break the Car")
        self.update_angle(self.angle+angle)
        return
    stepAngle = 360/4096*8;
    actualTicks = angle/stepAngle;
    if Direction == "Left":
        self.pinOnOff([9, 25])
        time.sleep(0.01);
        self.pinOnOff([9, 11]);
        time.sleep(0.01);
        self.pinOnOff([25, 8]);
        time.sleep(0.01);
        self.pinOnOff([9, 11]);
        time.sleep(0.01);        
        actualTicks = actualTicks - 1;
        while actualTicks>0:
            self.pinOnOff([8, 25]);
            time.sleep(0.01);
            self.pinOnOff([9, 11]);
            time.sleep(0.01);
            self.pinOnOff([8, 25]);
            time.sleep(0.01);
            self.pinOnOff([9, 11]);
            time.sleep(0.01);        
            actualTicks = actualTicks - 1;
    if Direction == "Right":
        self.pinOnOff([9, 8])
        time.sleep(0.01)
        self.pinOnOff([9, 11])
        time.sleep(0.01)
        self.pinOnOff([25, 8])
        time.sleep(0.01)
        self.pinOnOff([9, 11])
        time.sleep(0.01)       
        actualTicks = actualTicks - 1
        while actualTicks>0:
            self.pinOnOff([25, 8])
            time.sleep(0.01)
            self.pinOnOff([9, 11])
            time.sleep(0.01)
            self.pinOnOff([25, 8])
            time.sleep(0.01)
            self.pinOnOff([9, 11])
            time.sleep(0.01)         
            actualTicks = actualTicks - 1
    for i in [9, 25, 11, 8]:
      if self.pinStates[i] == 1:
        self.pinOnOff([i])
        
        
  """
  This cluster of Code is for the (Back Wheel) to turn
  
  DriveMotor:
  input # of steps, direction
  drive the back motor by number of steps
  
  This is really only used for DriveMotorCM
  
  DriveMotorCM:
    input cm: distance in cm; direction
    drive the car certain cm 
    updates the distance to the destination 
  """
  
  def setFIODrive(self, channel, state): #Labjack specific
    self.labjack.setDOState(channel, state);
  
  def DriveMotor(self, Steps = 0, Direction = "Forward"): 
    if Direction == "Forward":
        self.setFIODrive(5, 1)
    if Direction == "Backward":
        self.setFIODrive(5,0)
    for i in range(Steps):
        self.setFIODrive(4,1)
        time.sleep(0.05)
        self.setFIODrive(4, 0)
        time.sleep(0.05)  
  
  def DriveMotorCM(self, cm = 0, Direction = "Forward"):
    steps = int(6*cm)
    self.DriveMotor(steps, Direction)
    xmovement =  np.cos(self.heading)*cm
    ymovement = np.sin(self.heading)*cm
    if Direction == "Forward":
        self.update_destination(self.destination[0]-xmovement, self.destination[1]-ymovement)  
    if Direction == "Backward":
        self.update_destination(self.destination[0]+xmovement, self.destination[1]+ymovement)  
    actualtravel = steps/6;
    return actualtravel;
  
  """
  WALK:
    return boolean
    this determines if the destination is further than 5cm of the end goal
  """  
  def WALK(self):
      within = 25;
      yes = (self.destination[0]**2 + self.destination[1]**2)>=within;
      return yes;        
  
  
  """
  TurnInPlace:
    input: the (angle) of turning and to which (direction)
    this has an error of -2.5 ~ 0, always might be slightly less
  """
  
  def TurnInPlace(self, angle=0, Direction="Right"):
    distance = ((angle/2)*np.pi/180)*45
    self.TurnMotor(65, Direction)
    actualtravel = self.DriveMotorCM(distance, "Forward");
    actualangle=((actualtravel/45)*2)*180/np.pi
    print(actualtravel);
    if Direction == "Right":
      self.heading += actualangle
      self.TurnMotor(130, "Left")
    else:
      self.heading -= actualangle
      self.TurnMotor(130, "Right")
    self.DriveMotorCM(distance, "Backward")
    if Direction == "Right":
      self.TurnMotor(65, "Right")
    else:
      self.TurnMotor(65, "Left")
    return actualangle  
        
  """
  TurnTODest:
      Turn the robot to the destination;
      updates the heading
      Used in: avoid loop
  """


  def TurnToDest(self):
    tantheta = self.destination[1]/self.destination[0];
    theta = -np.arctan(tantheta) * 180/np.pi; ###here is minus because positive value of arctan is to the left of the +x-axis, so now, we make it negative, so it is represented in our sense
    turnangle = theta - self.heading; ### here the minus sign represents we are finding the distance from dest to where we are right now
    direction = "Left";
    if turnangle > 0:
        direction = "Right";
    self.TurnInPlace(abs(turnangle), direction); ###TurnInPlace updates the head angle       
  
  
  """
  irMotor:
    input: (angle) to turn the ir motor and (direction) to turn
    Turn the motor in the direction for the given angle and then update the self.irangle
  """      
  def irMotor(self, angle = 0, Direction = "Left"):
    stepAngle = 360/4096*8;
    if Direction == "Left":
        self.update_irangle(self.irangle-angle)  
    if Direction == "Right":
        self.update_irangle(self.irangle+angle)  
    actualTicks = angle/stepAngle;
    if Direction == "Left":
        self.pinOnOff([4, 15])
        time.sleep(0.01);
        self.pinOnOff([4, 17]);
        time.sleep(0.01);
        self.pinOnOff([15, 18]);
        time.sleep(0.01);
        self.pinOnOff([4, 17]);
        time.sleep(0.01);        
        actualTicks = actualTicks - 1;
        while actualTicks>0.5:
            self.pinOnOff([15, 18]);
            time.sleep(0.01);
            self.pinOnOff([4, 17]);
            time.sleep(0.01);
            self.pinOnOff([15, 18]);
            time.sleep(0.01);
            self.pinOnOff([4, 17]);
            time.sleep(0.01);        
            actualTicks = actualTicks - 1;
    if Direction == "Right":
        self.pinOnOff([4, 18])
        time.sleep(0.01)
        self.pinOnOff([4, 17])
        time.sleep(0.01)
        self.pinOnOff([15, 18])
        time.sleep(0.01)
        self.pinOnOff([4, 17])
        time.sleep(0.01)       
        actualTicks = actualTicks - 1
        while actualTicks>0.5:
            self.pinOnOff([15, 18])
            time.sleep(0.01)
            self.pinOnOff([4, 17])
            time.sleep(0.01)
            self.pinOnOff([15, 18])
            time.sleep(0.01)
            self.pinOnOff([4, 17])
            time.sleep(0.01)         
            actualTicks = actualTicks - 1
    for i in [4, 15, 17, 18]:
      if self.pinStates[i] == 1:
        self.pinOnOff([i])  
  
  '''
  irToDest:
      Turns the IR sensor towards the destination
      Updates the ir angle
      Used in: avoid loop
  '''
  
  def irToDest(self):
    tantheta = self.destination[1]/self.destination[0];
    theta = -np.arctan(tantheta) * 180/np.pi; ###here is minus because positive value of arctan is to the left of the +x-axis, so now, we make it negative, so it is represented in our sense
    headangle = theta - self.heading; ### here the minus sign represents we are finding the distance from dest to where we are right now
    turnangle = self.irangle - headangle
    direction="Left"
    if turnangle > 0:
        direction = "Right";
    self.irMotor(abs(turnangle), direction) ###irMotor updates the irangle
    time.sleep(0.1);

  """
  reset
    stop all motors,
    move the front wheel back to original position
  """


  def reset(self):
      self.UhOh()
      if self.angle >0:
          self.TurnMotor(self.angle, "Left")
      if self.angle <0:
          self.TurnMotor(abs(self.angle), "Right")
      self.TurntoAngle(0)
        
        
  """
  VoltagetoDistance
    input n = 0 --> the front IR sensor
          n = 1 --> the scanning IR sensor
    this function let us know the distance in CM detected by either sensor
      if either the voltage getting in is too small or the distance is greater than 50cm then return 100 stand for invalid
  """      
  def getAIN(self, n=0):
      ain0bits, = self.labjack.getFeedback(u3.AIN(n));
      ainValue = self.labjack.binaryToCalibratedAnalogVoltage(ain0bits, isLowVoltage=False, channelNumber = 0);
      return ainValue


  def VoltagetoDistance(self, n=0):
      Voltage = self.getAIN(n)
      if Voltage <=0.46:
          return 100, Voltage
      x = np.log((Voltage-0.46)/3.67)/(-0.068)
      if x>50:
          x = 100
      return x, Voltage        
  
  """
  TurntoAngle
    input: angle: positive to the right, negative to the left
    make the scanning ir sensor pointing to the angle
  """
        
  def TurntoAngle(self,angle): #This code turns the scope to a specific angle relative to the car
      current_angle = self.irangle
      final_dest = angle - current_angle
      if final_dest > 0:
          self.irMotor(abs(final_dest), "Right")
      else:
          self.irMotor(abs(final_dest), "Left")
          
          
  """
  ScopeSCan:
    return an array of scan
    slicing the entire front space of 180 degree to 18 pieces
    scan each piece and determine the distance to the obstacles, return all the distance in an array
    ****subfunction Mode and IRValMode
        since the IR sensor is not always working well, 
        so i used the (mode) to sort out the most abundant ones, 
        then average over all the voltage, 
        then convert the average voltage to distance
        this works good enough for it always returns the real distance +/- 2.5cm
  """        
      
  def mode(self, array, error):
    counter = 0;
    maxv = -100;
    maxI = -1;
    for i in range(len(array)):
      counter = 0;
      for j in range(len(array)):
        if j!=i and abs(array[i] - array[j])<=error:
          counter = counter + 1;
        if maxv < counter:
          maxv = counter;
          maxI = i;
    return array[maxI];
  
  def IRValMode(self, ain = 0, slp = 30):
    temp = 0;
    tempArray = np.zeros(slp);
    tempsum = 0;
    data = 0;
    counter = 0;
    for j in range(slp):
      time.sleep(0.01);
      temp, tempArray[j] = self.VoltagetoDistance(ain);
    data = self.mode(tempArray, 0.04);
    for j in range(slp):
      if abs(data-tempArray[j]) <=0.04:
        tempsum = tempsum + tempArray[j];
        counter = counter + 1;
    data = tempsum/counter;
    if data < 0.46:
      return 100;
    data = np.log((data-0.46)/3.67)/(-0.068)  
    if data > 50:
      data = 100;
    return data;

  def ScopeScan(self):
    angle_increment = 10
    num_steps = 180//angle_increment; 
    ir_data = np.zeros(num_steps);
    ir_vol = np.zeros(num_steps);
    self.TurntoAngle(-90)
    for i in range(num_steps):
        ir_data[i] = self.IRValMode(1);
        self.irMotor(angle_increment, "Right")
    self.TurntoAngle(0);
    print(ir_data);
    return ir_data          






  """
  sensor_loop:
    the main loop that will run the car to the destination set by destination(x,y);
    !!!!always should end here
    this starts by turning to the direction of the end point
  """

  def sensor_loop(self):
    front_min = 40 # cm Minimum values telling the car when to stop
    self.TurnToDest();
    while self.WALK(): ###true when the car is not within 5cm radius of the end point
        front_dist = self.IRValMode(0,5) #scan the front 
        print(front_dist)
        if front_dist >= front_min or front_dist >= self.getDistance(): #if we still have space drive forward or if the destination is close enough
            self.event_queue.put("Drive", 1) #passes drive to the queue loop, see the queue loop for more detail. This doesnt work yet, I am just including it in case we can get it to work.
            self.DriveMotorCM(1, "Forward") #Drive the motor one step
            print("vroom vroom") #I think this is a funny way to tell us what it is doing.
            time.sleep(0.05) #This sleeps the code to give the motor time to do its thing. If it doesnt sleep it will make a massive queue of drives, so when the stop command comes its so late that the robot will just crash lol
        else: #front sensor isnt happy anymore. Passes to next layer of decision making.
            #diag_right_dist = VoltagetoDistance(2) #These would be the AIN readings from the left and right sensons once mounted, respectively
            #diag_left_dist = VoltagetoDistance(3)
            diag_left_dist = 10 #I have these set arbitrarily to always trigger the next loop where the scanner decides
            diag_right_dist = 10
            
            self.event_queue.put("Stop", 1) #Obviously still doesnt do anything. 
            
            array_data = self.ScopeScan() 
            decision_array = [array_data[9],array_data[8],array_data[10],array_data[7],array_data[11],array_data[6],array_data[12],array_data[5],array_data[13], array_data[4], array_data[14], array_data[3], array_data[15],array_data[2], array_data[16], array_data[1], array_data[17], array_data[0]] #This will take the IR data and list it in a way that goes center, then direcetly to the right of center, directly to the left, second from the right and so on. This allows the robot to pick the most optimal path
            for i, dist in enumerate(decision_array):
                if 100 <= dist:
                    if i % 2 == 0: #This modulo is essentially saying right or left. All odd arrays correspond to right turns, all even correspond to left turns
                        print(f"Turning to angle {(((i+2)//2)*(-1)**i) * 10 + 5} degrees...") #This tells the angle of the turn, measured from centerline. Negative values correspond to left turns
                        angle = (((i+2)//2)*(-1)**i) * 10 + 5
                        #self.event_queue.put("AngleTurn", angle) #I want to pass both a queue event and a variable to an event queue, but it doesnt work just yet.
                         #This will stop the scanning queue, allowing a different loop to take over
                        i+=1
                        return self.avoid_loop_better(angle, front_dist)
                    else: #same as above but for the other direction
                        print(f"Turning to angle {(((i+2)//2)*(-1)**i) * 10 - 5} degrees...")
                        angle = (((i+2)//2)*(-1)**i) * 10 - 5
                        #self.event_queue.put("AngleTurn", angle)
                        i+=1
                        return self.avoid_loop_better(angle, front_dist)
                elif i+1 == len(decision_array): #I havent coded this yet, but this is the option for going backwards cause the robot messed up
                    print("didn't find acceptable range before middle of array, turn around")
                    self.DriveMotor(100, "Backward")
                    return self.sensor_loop()
   
                
   
    
"""
avoid_loop_better
    
Used to get around an obstacle. This is called from scope scan, and is passed the angle that will 
allow the car to most efficiently avoid the obstacle, as well as how far away the obstacle is. 
From this data, it calls TurnInPlace, then drives until it gets to the obstacle. Once there, it will 
turn the scope to point at the destination. If it is clear to return to the path, it will call sensor_loop
and continue on its way. Otherwise, it will continue to drive until it is clear to return, unless it runs 
into another obstacle, which we should discuss a way to figure out

"""
  
  
  def avoid_loop_better(self, angle, dist):
    print(angle);  
    i = 0 #This i value is used to make sure that the car drives at least the distance of the obstacle before checking to see if the side is clear.
    if angle > 0: #These just turn the car to point away from the obstacle
       self.TurnInPlace(abs(angle), "Right")
    elif angle < 0:
       self.TurnInPlace(abs(angle), "Left")
    while i*np.cos(angle/180*np.pi) < dist:
       front_dist = self.VoltagetoDistance(0)
       self.DriveMotorCM(1, "Forward")
       i += 1
       if 20 < front_dist < 100: #Drive forward, if the car is happy
           self.DriveMotorCM(1, "Forward")
           i += 1
       if front_dist < 30: #If the front sensor gets upset, default back to the sensor loop. 
           return self.sensor_loop() 
    if i*np.cos(angle/180*np.pi) > dist:
        self.irToDest() #Turn the scope to look at destination
        self.VoltagetoDistance(1)
        while self.VoltagetoDistance(1) < 100:
            self.DriveMotorCM(3, "Forward")
            self.irToDest()
        if self.VoltagetoDistance(1) >= 100:
            return self.sensor_loop()
        

"""
DrivetoDest:
Heres my solution to running into another obstace. I dont think sensor loop should always start by turning to the obstace, 
as there are moments where we need its functionality even if we cant go to the obstace. Thats what this code is for.
Essentially, I think that in the second part of avoid loop, where it is checking if it can return to the course,
if it can, it calls this, which will turn the car to the angle, then this calls sensor loop which gets the car moving agian.
If avoid loop runs into another obstacle before it is clear to return to course, then it will just call sensor loop
without turning the car, so it can avoid the obstacle. This is just an idea, happy to hear your guys' thoughts.

"""
        
        
  def DrivetoDest(x,y):
      self.TurnToDest()
      while self.WALK():
          return self.sensor_loop():
      else:
          return
        


         
                
                