# -*- coding: utf-8 -*-
"""
Created on Wed May 17 14:59:09 2023

@author: jaspe
"""

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
   
class piRobot():
  def __init__(self):
    gpio.setmode(gpio.BCM)
    for i in range(28):
      gpio.setup(i, gpio.out)
    self.pinStates = np.zeros(28)
    self.angle = 0
    self.direction = 1
    self.labjack = u3.U3()
    self.labjack.configIO(FIOAnalog = 0)
    self.frontvision = 0
    self.location = RobotLocation()
    self.turnangle = 0 
    self.turnstraightsteps = 0
    self.straightsteps = 0
    
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
 
  def VoltagetoDistance(self, n=0):
      Voltage = getAIN(n)
      x = np.log((Voltage-0.46)/3.67)/(-0.068)
      return x
       

  event_queue = queue.Queue()

  stop_flag = False

  def sensor_loop(self):
      front_min = 40 # cm
      diag_min = 40 #cm
      while not stop_flag:
          front_dist = VoltagetoDistance()
          if front_dist >= front_min:
              print("Continuing straight...")
              event_queue.put("Drive")
          else:
              #diag_right_dist = VoltagetoDistance()
              #diag_left_dist = VoltagetoDistance()
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
                  middle_index = len(array_data) // 2
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

  def avoid_loop(self,angle, n):
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
                        
                    
          
  def RtC(self):
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
              
  def event_loop(self):
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




  def GOGOGO(self):
      sensor_thread.start()
      event_thread.start()

  def Stop():
      sensor_thread.stop()
      event_thread.stop()
      reset()
 

  