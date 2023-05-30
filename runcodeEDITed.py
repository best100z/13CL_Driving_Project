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
    self.destination = [0,0] ###(x,y) coordinate as the robot is going to move to
    self.heading = 0 ###the direction the car is driving relative to +x-axis; -is to the left, +is to the right
    self.labjack = u3.U3()
    self.labjack.configU3()    
    self.angle = 0  ###angle for the front wheel, it only turns to 65 degree to either left or right; act as a protection and as a reminder when forget to return the front wheel to orignial position
    """
    skeptical if we need to keep it
    """
    #self.sensor_thread = threading.Thread(target=self.sensor_loop)
    #self.event_thread = threading.Thread(target=self.event_loop)
    #self.event_queue = queue.Queue() #defines the Queue variable
  
  
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
    self.update_destination(self.destination[0]-xmovement, self.destination[1]-ymovement)        
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
    turnangle = theta - self.heading); ### here the minus sign represents we are finding the distance from dest to where we are right now
    direction = "Left";
    if turnangle > 0:
        direction = "Right";
    self.TurnInPlace(abs(turnangle), direciton); ###TurnInPlace updates the head angle       
        
  
  
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
          return 100
      x = np.log((Voltage-0.46)/3.67)/(-0.068)
      if x>50:
          x = 100
      return x        
  
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
  """        
          
  def ScopeScan(self):
    angle_increment = 10
    num_steps = 180//angle_increment 
    ir_data = np.zeros(num_steps)
    self.TurntoAngle(-90)
    for i in range(num_steps):
        time.sleep(0.1)
        ir_data[i] = self.VoltagetoDistance(1)
        self.update_irangle(self.irangle + angle_increment)
        self.irMotor(angle_increment, "Right")
    self.TurntoAngle(0);
    print(ir_data);
    return ir_data          



















          
