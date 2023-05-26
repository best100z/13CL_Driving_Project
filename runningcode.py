
import numpy as np
import RPi.GPIO as gpio
import u3
import time
import threading
import queue
import numpy as np

#pin configuration
   
class piRobot():
  def __init__(self):
    gpio.setmode(gpio.BCM)
    for i in range(28):
      gpio.setup(i, gpio.OUT)
    self.pinStates = np.zeros(28)
    self.angle = 0
    self.irangle = 0
    self.direction = 1
    self.labjack = u3.U3()
    self.labjack.configU3()
    self.frontvision = 0
    self.sensor_thread = threading.Thread(target=self.sensor_loop)
    self.event_thread = threading.Thread(target=self.event_loop)
    self.event_queue = queue.Queue() #defines the Queue variable

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
      
  def update_irangle(self, angle):
    self.irangle = angle
  
  def UhOh(self):   
    for i in range(28):
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
    
  #Driving Code

  def setFIODrive(self, channel, state): #Labjack specific
    self.labjack.setDOState(channel, state);

  def DriveMotor(self, Steps = 0, Direction = "Forward"): #'drive' means this command controls the driving motor. Needs power to both DIR and STEP pin on DRV8825, will turn the motor by step, not by distance
    if Direction == "Forward":
        self.setFIODrive(5, 1)
    if Direction == "Backward":
        self.setFIODrive(5,0)
    for i in range(Steps):
        self.setFIODrive(4,1)
        time.sleep(0.01)
        self.setFIODrive(4, 0)
        time.sleep(0.01)
      
    

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


  def DriveStraight(self, cm = 1, Direction = "Forward"):
      if self.angle >0:
          self.TurnMotor(self.angle, "Left")
      if self.angle <0:
          self.TurnMotor(abs(self.angle), "Right")
      self.DriveMotorCM(cm, Direction)
      
  
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
        while actualTicks>0:
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
        while actualTicks>0:
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
 

  def reset(self):
      self.UhOh()
      if self.angle >0:
          self.TurnMotor(self.angle, "Left")
      if self.angle <0:
          self.TurnMotor(abs(self.angle), "Right")
      self.TurntoAngle(0)

  "Pathfinding Code"
    
  def DriveCar(self, distance = 0):
    pass  

 
    
    
    
  def getAIN(self, n=0):
      ain0bits, = self.labjack.getFeedback(u3.AIN(n));
      ainValue = self.labjack.binaryToCalibratedAnalogVoltage(ain0bits, isLowVoltage=False, channelNumber = 0);
      return ainValue
 

  def VoltagetoDistance(self, n=0):
      Voltage = self.getAIN(n)
      if Voltage > 0.46:
          x = np.log((Voltage-0.46)/3.67)/(-0.068)
          return x
      else:
          x = 100
          return x      

  def TurntoAngle(self,angle): #This code turns the scope to a specific angle relative to the car
      current_angle = self.irangle
      final_dest = angle - current_angle
      if final_dest > 0:
          self.irMotor(abs(final_dest), "Right")
          return
      else:
          self.irMotor(abs(final_dest), "Left")
          return


  def ScopeScan(self): #This code will take an array of data in a 20 degree range. I think it can be imporved by using TurntoAngle(), as this array isnt great in terms of where the scope is pointing as there is no perfect middle measurement
    angle_increment = 20
    num_steps = 180//angle_increment
    ir_data = np.zeros(num_steps)
    self.irMotor(90, "Left")
    self.update_irangle(self.irangle-90)
    for i in range(num_steps):
        time.sleep(0.1)
        ir_data[i] = self.VoltagetoDistance(1)
        self.update_irangle(self.irangle + angle_increment)
        self.irMotor(angle_increment, "Right")
    self.irMotor(90, "Left")
    self.update_irangle(self.irangle-90)
    min_value = np.min(ir_data)
    min_index = np.argmin(ir_data)
    print(ir_data, min_value, min_index)
    return ir_data
   
  



  def sensor_loop(self): #This is the main sensing decision making code
    front_min = 35 # cm Minimum values telling the car when to stop
    diag_min = 45 #cm Minimum values for the diagonal turns. These are the two sensors pointing off to the sides
    i = 0
    while i < 1: #do this loop when the stop flag is false
        front_dist = self.VoltagetoDistance(0) #scan the front 
        print(front_dist)
        if front_dist >= front_min: #if we still have space drive forward
              #for me to know what the robot is thinking
            self.event_queue.put("Drive", 1) #passes drive to the queue loop, see the queue loop for more detail
            self.DriveMotor(1, "Forward")
            print("vroom vroom")
            time.sleep(0.1)
             #This sleeps the code to give the motor time to do its thing. If it doesnt sleep it will make a massive queue of drives, so when the stop command comes its so late that the robot will just crash lol
        else: #front sensor isnt happy anymore. Passes to next layer of decision making.
            #diag_right_dist = VoltagetoDistance(2) #These would be the AIN readings from the left and right sensons once mounted, respectively
            #diag_left_dist = VoltagetoDistance(3)
            diag_left_dist = 10 #I have these set arbitrarily to always trigger the next loop where the scanner decides
            diag_right_dist = 10
            
            self.event_queue.put("Stop", 1) 
            
            if diag_min <= diag_right_dist and diag_min <= diag_left_dist: #If both left and right look okay, arbitrarily go right
                print("Turning right...")
                self.event_queue.put("Right Turn", 1)
                break
            elif diag_min <= diag_right_dist: #If right is good, go right
                print("Turning right...")
                self.event_queue.put("Right Turn", 1)
                break
            elif diag_min <= diag_left_dist: #If left is good, go left
                print("Turning left...")
                self.event_queue.put("Left Turn", 1)
                break
            else: #If neither is good, go onto the scanning loop
                array_data = self.ScopeScan() 
                decision_array = [array_data[4],array_data[3],array_data[5],array_data[2],array_data[6],array_data[1],array_data[7],array_data[0],array_data[8]] #This will take the IR data and list it in a way that goes center, then direcetly to the right of center, directly to the left, second from the right and so on. This allows the robot to pick the most optimal path
                for i, dist in enumerate(decision_array):
                    if front_min <= dist:
                        if i % 2 == 0: #This modulo is essentially saying right or left. All odd arrays correspond to right turns, all even correspond to left turns
                            if i == 0: #This is because the scanner is behind the front IR sensor. Sometimes, the scanner will read an ok value in front, when in reality there isnt. Therefore, the code just ignores it.
                                continue 
                            else:
                                print(f"Turning to angle {(((i)/2)*(-1)**i) * 20 } degrees...") #This tells the angle of the turn, measured from centerline. Negative values correspond to left turns
                                angle = (((i)/2)*(-1)**i) * 20
                                self.event_queue.put("AngleTurn", angle) #I want to pass both a queue event and a variable to an event queue, but it doesnt work just yet.
                                 #This will stop the scanning queue, allowing a different loop to take over
                                i+=1
                                self.avoid_loop(angle)
                                
                                return
                        else: #same as above but for the other direction
                            print(f"Turning to angle {(((i+1)/2)*(-1)**i) * 20 } degrees...")
                            angle = (((i)/2)*(-1)**i) * 20
                            self.event_queue.put("AngleTurn", angle)
                            i+=1
                            self.avoid_loop(angle)
                            return
                    elif i+1 == len(decision_array): #I havent coded this yet, but this is the option for going backwards cause the robot messed up
                        print("didn't find acceptable range before middle of array, turn around")
                        break

                        
  def avoid_loop(self, angle):
    i = 0 #This i value is used to make sure that once the car is turned 90 degrees itll stop turning and just go straight
    while True:
       if angle > 0: #These just turn the car in the right direction 
           self.TurnMotor(65, "Right")
       elif angle < 0:
           self.TurnMotor(65, "Left")
       while i < 500: #So while the car isnt turned 90 yet
           print("Starting to Turn")
           front_dist = self.VoltagetoDistance(1)
           if 10 < front_dist < 100: #Drive forward, ie turn
               self.DriveMotor(1, "Forward")
               i += 1
               #selfavoidlogic.updateanglesteps() #This is keeping track of the amount of steps that the car has turned at an angle, used to retrace later
           if front_dist < 10: #If in this process the robot gets to close to an obstace the robot gets sad. We can code something here to have it reverse or something
               print("ima kms")
               break
           if front_dist >= 100: #Once the car has turned far enough that the front is clear, we now will drive until we are fully past the obstace. NOTHING FROM HERE ON HAS BEEN TESTED AND FUNCITONAL
               print("Checking the Hip")
               self.TurntoAngle(-2*angle) #Tursn scope to look at obstacle
               side_dist = self.VoltagetoDistance(1)
               if side_dist < 100:
                  self.DriveMotor(1, "Forward")
                  i += 1
               if side_dist >= 100:
                  self.irMotor(180, "Right")
                  self.irMotor(180, "Left")
                  
                  self.reset()
                  self.sensor_loop()
                  return
       while i == 500:  #Once the car has turned 90 degress
           return
           print("made it to 90")
           if self.angle > 0: #This two commands will turn the steering straight, then will turn the scope to look directly to the side where the obstacle would be
               self.TurnMotor(65, "Left")
               self.TurntoAngle(-90)
           if self.angle < 0:
               self.TurnMotor(65, "Right")
               self.TurntoAngle(90)
           side_dist = self.VoltagetoDistance(1)   
           if side_dist < 100: #Drive until the scope doesnt see anything on the side
               self.DriveMotor(1, "Forward")
               #avoidlogic.updatestraightsteps() #Keep track of the amount of steps taken directly straight, used for return to course
           if side_dist >= 100:
               self.event_queue.put("RtC") #Side is clear, go back to the og course
               print("returning to course")                     

  def event_loop(self):
      while True:
          event_data = self.event_queue.get()  
          event = event_data[0]
          value = event_data[1]
          if event == "Drive":
              print("drive")
              self.DriveMotor(1, "Forward")
          if event == "Stop":
              print("stop")
              self.UhOh()
          if event == "AngleTurn":
              angle = value
              print("going to avoid loop")
              self.avoid_loop(angle)
          if event == "RtC":
              RtC()
                  



 


  def GOGOGO(self):
      self.event_thread.start()

  def Stop(self):
      self.sensor_thread.stop()
      self.event_thread.stop()
      self.reset()
 
#event_queue = queue.Queue()

myRobot = piRobot()
myRobot.irMotor(5, "Left")                  




