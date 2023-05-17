import numpy as np
import RPi.GPIO as gpio
import u3
import time
import threading
import queue
import numpy as np

#pin configuration

class RobotLocation():
    def __init__(self):
        self.frontvision = 0
    def update_sight(self,frontvision):
        self.frontvision = frontvision
   
class piRobot():
  def __init__(self):
    gpio.setmode(gpio.BCM)
    for i in range(28):
      gpio.setup(i, gpio.OUT)
    self.pinStates = np.zeros(28)
    self.angle = 0
    self.direction = 1
    self.labjack = u3.U3()
    self.labjack.configIO(FIOAnalog = 0)
    self.frontvision = 0
    self.location = RobotLocation()
    
  def pinOnOff(self, numbers):
    for number in numbers:
      if self.pinStates[number]==0:
        gpio.output(number, gpio.HIGH)
        self.pinStates[number]=1
      else:
        gpio.output(number, gpio.LOW)
        self.pinStates[number]=0
  
  #pins number 4, 15, 17, 18 will be used for the IR sensor motor
  #pins number 9, 25, 11, 8 will be used for the turning motor
  #pins number 19, 16, 26, 20 will be used for the driving motor
  #any time setFIOs used a 4-digit code, I used the order I named them above
  
  def update_angle(self, angle):
    self.angle = angle
      
  def UhOh(self):   
    for i in range(28)
      if self.pinStates[i] == 1:
        self.pinOnOff(i)
  
  #Turning Code
  
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
        time.sleep(0.0001);
        self.pinOnOff([9, 11]);
        time.sleep(0.0001);
        self.pinOnOff([25, 8]);
        time.sleep(0.0001);
        self.pinOnOff([9, 11]);
        time.sleep(0.0001);        
        actualTicks = actualTicks - 1;
        while actualTicks>0:
            self.pinOnOff([8, 25]);
            time.sleep(0.0001);
            self.pinOnOff([9, 11]);
            time.sleep(0.0001);
            self.pinOnOff([8, 25]);
            time.sleep(0.0001);
            self.pinOnOff([9, 11]);
            time.sleep(0.0001);        
            actualTicks = actualTicks - 1;
    if Direction == "Right":
        self.pinOnOff([9, 8])
        time.sleep(0.0001)
        self.pinOnOff([9, 11])
        time.sleep(0.0001)
        self.pinOnOff([25, 8])
        time.sleep(0.0001)
        self.pinOnOff([9, 11])
        time.sleep(0.0001)       
        actualTicks = actualTicks - 1
        while actualTicks>0:
            self.pinOnOff([25, 8])
            time.sleep(0.0001)
            self.pinOnOff([9, 11])
            time.sleep(0.0001)
            self.pinOnOff([25, 8])
            time.sleep(0.0001)
            self.pinOnOff([9, 11])
            time.sleep(0.0001)         
            actualTicks = actualTicks - 1
    for i in [9, 25, 11, 8]:
      if self.pinStates(i) == 1:
        self.pinOnOff([i])
    
  #Driving Code

  def DriveMotor(self, distance = 0, Direction = "Forward"): #360 Degrees = 22cm of movement
      angle = distance*360/22
      stepAngle = 360/4096*8;
      actualTicks = angle/stepAngle;
      if Direction == "Backward":
          self.pinOnOff([19, 16])
          time.sleep(0.0001)
          self.pinOnOff([19, 26])
          time.sleep(0.0001)
          self.pinOnOff([16, 20])
          time.sleep(0.0001)
          self.pinOnOff([19, 26])
          time.sleep(0.0001)         
          actualTicks = actualTicks - 1
          while actualTicks>0:
              self.pinOnOff([16, 20])
              time.sleep(0.0001)
              self.pinOnOff([19, 26])
              time.sleep(0.0001)
              self.pinOnOff([16, 20])
              time.sleep(0.0001)
              self.pinOnOff([19, 26])
              time.sleep(0.0001)         
              actualTicks = actualTicks - 1
      if Direction == "Forward":
          self.pinOnOff([19, 20])
          time.sleep(0.0001)
          self.pinOnOff([19, 26])
          time.sleep(0.0001)
          self.pinOnOff([16, 20])
          time.sleep(0.0001)
          self.pinOnOff([19, 26])
          time.sleep(0.0001)         
          actualTicks = actualTicks - 1
          while actualTicks>0:
              self.pinOnOff([16, 20])
              time.sleep(0.0001)
              self.pinOnOff([19, 26])
              time.sleep(0.0001)
              self.pinOnOff([16, 20])
              time.sleep(0.0001)
              self.pinOnOff([19, 26])
              time.sleep(0.0001)       
              actualTicks = actualTicks - 1;
      for i in [19, 16, 26, 20]:
        if self.pinStates(i) == 1:
          self.pinOnOff([i])

  def ReverseTurn90(self, Direction= "Right"):
      if Direction == "Right":
          self.TurnMotor(65, "Left")
          self.DriveMotor(360, "Backward")
          self.TurnMotor(130, "Right")
          self.DriveMotor(760, "Forward")
          self.TurnMotor(65, "Left")
      if Direction == "Left":
          self.TurnMotor(65, "Right")
          self.DriveMotor(360, "Backward")
          self.TurnMotor(130, "Left")
          self.DriveMotor(760, "Forward")
          self.TurnMotor(65, "Right")

  def DrivingTurn(self, Distance = 22, Direction = "Right"): #Need 34cm to avoid obstacle using this
      if Direction == "Right":
          self.TurnMotor(65, "Right")
          self.DriveMotorCM(Distance, "Forward")
      if Direction == "Left":
          self.TurnMotor(65, "Left")
          self.DriveMotorCM(Distance, "Forward")

  def ReverseTurn45(self, Direction = "Right"):
      if Direction == "Right":
          self.TurnMotor(65, "Left")
          self.DriveMotor(180, "Backward")
          self.TurnMotor(130, "Right")
          self.DriveMotor(360, "Forward")
          self.TurnMotor(65, "Left")
      if Direction == "Left":
          self.TurnMotor(65, "Right")
          self.DriveMotor(180, "Backward")
          self.TurnMotor(130, "Left")
          self.DriveMotor(360, "Forward")
          self.TurnMotor(65, "Right")

  def reset(self):
      self.UhOh()
      if self.angle >0:
          self.TurnMotor(Steering_Angle.angle, "Left")
      if self.angle <0:
          self.TurnMotor(abs(Steering_Angle.angle), "Right")

  def DriveStraight(self, cm = 1, Direction = "Forward"):
      if self.angle >0:
          self.TurnMotor(Steering_Angle.angle, "Left")
      if self.angle <0:
          self.TurnMotor(abs(Steering_Angle.angle), "Right")
      selt.DriveMotorCM(cm, Direction)
   


  "Pathfinding Code"


  def getAIN(self, n=0):
      ain0bits, = self.labjack.getFeedback(u3.AIN(n));
      ainValue = self.labjack.binaryToCalibratedAnalogVoltage(ain0bits, isLowVoltage=False, channelNumber = 0);
      return ainValue
 

       

  def sensor_loop():
      while True:
          sensor_data = VoltagetoDistance()
          robotloc.update_sight(sensor_data)
          if sensor_data >= 36:
              event_queue.put("forward")
              time.sleep(1.25)
              print(sensor_data)
              print("Forward")
          elif sensor_data < 36:
              event_queue.put("turn")
              time.sleep(1.25)
              print(sensor_data)
              print("Turn")


  def event_loop():
      while True:
          event = event_queue.get()   
          if event == "forward":
              DriveStraight(1, "Forward")
          if event == "turn":
              DrivingTurn(1,"Right")


  sensor_thread = threading.Thread(target=sensor_loop)
  event_thread = threading.Thread(target=event_loop)


  def VoltagetoDistance():
      Voltage = getAINhandle1()
      x = np.log((Voltage-0.46)/3.67)/(-0.068)
      return x

  def GOGOGO():
      sensor_thread.start()
      event_thread.start()

  def Stop():
      sensor_thread.stop()
      event_thread.stop()
      reset()
 
#event_queue = queue.Queue()

myRobot = piRobot()
myRobot.TurnMotor(65, "Left")

