
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
    gpio.setwarnings(False)
    for i in range(28):
      gpio.setup(i, gpio.OUT)
    self.pinStates = np.zeros(28)
    self.angle = 0
    self.irangle = 0
    self.labjack = u3.U3()
    self.labjack.configU3()
    self.destination = [0,0]
    self.heading = 0 #This is the direction that the robot is facing relative to its starting direction
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
    
  def update_destination(self, x, y):
    self.destination = [x,y]

  def update_heading(self, angle):
    self.heading = angle
  
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
        time.sleep(0.05)
        self.setFIODrive(4, 0)
        time.sleep(0.05)
      
  def DriveMotorCM(self, cm = 0, Direction = "Forward"): #this is the same as above, but with cm instead of steps
    steps = (6*cm)-4
    self.DriveMotor(steps, Direction)
    xmovement =  np.cos(self.heading)*cm
    ymovement = np.sin(self.heading)*cm
    self.update_destination(self.destination[0]-xmovement, self.destination[1]-ymovement)
  
  """
    WALK:
    return boolean
    this determines if the destination is within 5cm of the end goal
  """  
  def WALK(self):
      within = 25;
      yes = (self.destination[0]**2 + self.destination[1]**2)>=within;
      return yes;
   
  def TurnInPlace(self, angle=0, Direction="Right"):
    distance = int(((angle/2)*np.pi/180)*45)
    actualangle=((distance/45)*2)*180/np.pi
    self.TurnMotor(65, Direction)
    self.DriveMotorCM(distance, "Forward")
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
    
  def Turn5DegIncs(self, numIncs=0, direction="Right"):
    actualangle=0
    for i in range(numIncs):
        actualangle+=self.TurnInPlace(5, direction)
    return actualangle
         
  def Turn10DegIncs(self, numIncs=0, direction="Right"):
    actualangle=0
    for i in range(numIncs):
        actualangle+=self.TurnInPlace(10, direction)
    return actualangle

  def Turn30DegIncs(self, numIncs=0, direction="Right"):
    actualangle=0
    for i in range(numIncs):
        actualangle+=self.TurnInPlace(30, direction)
    return actualangle

  """
  TurnTODest:
      Turn the robot to the destination;
      updates the heading
      Used in: avoid loop
  """


  def TurnToDest(self):
    tantheta = self.destination[1]/self.destination[0];
    theta = np.arctan(tantheta) * 180/np.pi;
    turnangle = theta - self.heading;
    direction = "Left";
    if turnangle > 0:
        direction = "Right";
    self.TurnInPlace(abs(turnangle), direciton);
  
  '''
  irToDest:
      Turns the IR sensor towards the destination
      Updates the ir angle
      Used in: avoid loop
  '''
  
  def irToDest(self):
    tantheta = self.destination[1]/self.destination[0];
    theta = np.arctan(tantheta) * 180/np.pi;
    headangle = theta - self.heading;
    turnangle = self.irangle - headangle
    direction="Left"
    if turnangle > 0:
        direction = "Right";
    self.irMotor(abs(turnangle), direction)
    
  def MovingScan(self, Direction="Forward"):
    stepAngle = 360/4096*8;
    scansteps = 90/stepAngle
    ticker = 0
    drivecounter=0
    scancounter=0
    scanarray = np.zeros(16)
    if Direction == "Forward":
        self.setFIODrive(5, 1)
    if Direction == "Backward":
        self.setFIODrive(5,0)
    if self.irangle <= 0:
      leftmove=45-abs(self.irangle)
      actualTicks = abs(leftmove/stepAngle)
      if leftmove>=0:
         self.pinOnOff([4, 15])
         time.sleep(0.01)
         self.pinOnOff([4, 17])
         time.sleep(0.01)
         self.pinOnOff([15, 18])
         time.sleep(0.01)
         self.pinOnOff([4, 17])
         time.sleep(0.01)        
         actualTicks -= 1
         while actualTicks>0:
             self.pinOnOff([15, 18])
             time.sleep(0.01)
             self.pinOnOff([4, 17])
             time.sleep(0.01)
             self.pinOnOff([15, 18])
             time.sleep(0.01)
             self.pinOnOff([4, 17])
             time.sleep(0.01)        
             actualTicks -= 1
             if ticker%2==0:
                self.setFIODrive(4,1)
             else:
                self.setFIODrive(4,0)
                drivecounter+=1
             ticker+=1
             for i in [4, 15, 17, 18]:
                if self.pinStates[i] == 1:
                    self.pinOnOff([i])
      else:
         self.pinOnOff([4, 18])
         time.sleep(0.01)
         self.pinOnOff([4, 17])
         time.sleep(0.01)
         self.pinOnOff([15, 18])
         time.sleep(0.01)
         self.pinOnOff([4, 17])
         time.sleep(0.01)       
         actualTicks -=1
         while actualTicks>0:
             self.pinOnOff([15, 18])
             time.sleep(0.01)
             self.pinOnOff([4, 17])
             time.sleep(0.01)
             self.pinOnOff([15, 18])
             time.sleep(0.01)
             self.pinOnOff([4, 17])
             time.sleep(0.01)         
             actualTicks -=1
             if ticker%2==0:
                self.setFIODrive(4,1)
             else:
                self.setFIODrive(4,0)
                drivecounter+=1
             ticker+=1
             for i in [4, 15, 17, 18]:
                if self.pinStates[i] == 1:
                    self.pinOnOff([i])
      self.irangle=-45
    else:
      rightmove=45-self.irangle
      actualTicks = abs(rightmove/stepAngle)
      if rightmove<=0:
         self.pinOnOff([4, 15])
         time.sleep(0.01)
         self.pinOnOff([4, 17])
         time.sleep(0.01)
         self.pinOnOff([15, 18])
         time.sleep(0.01)
         self.pinOnOff([4, 17])
         time.sleep(0.01)        
         actualTicks -= 1
         while actualTicks>0:
             self.pinOnOff([15, 18])
             time.sleep(0.01)
             self.pinOnOff([4, 17])
             time.sleep(0.01)
             self.pinOnOff([15, 18])
             time.sleep(0.01)
             self.pinOnOff([4, 17])
             time.sleep(0.01)        
             actualTicks -= 1
             if ticker%2==0:
                self.setFIODrive(4,1)
             else:
                self.setFIODrive(4,0)
                drivecounter+=1
             ticker+=1
             for i in [4, 15, 17, 18]:
                if self.pinStates[i] == 1:
                    self.pinOnOff([i])
      else:
         self.pinOnOff([4, 18])
         time.sleep(0.01)
         self.pinOnOff([4, 17])
         time.sleep(0.01)
         self.pinOnOff([15, 18])
         time.sleep(0.01)
         self.pinOnOff([4, 17])
         time.sleep(0.01)       
         actualTicks -=1
         while actualTicks>0:
             self.pinOnOff([15, 18])
             time.sleep(0.01)
             self.pinOnOff([4, 17])
             time.sleep(0.01)
             self.pinOnOff([15, 18])
             time.sleep(0.01)
             self.pinOnOff([4, 17])
             time.sleep(0.01)         
             actualTicks -=1
             if ticker%2==0:
                self.setFIODrive(4,1)
             else:
                self.setFIODrive(4,0)
                drivecounter+=1
             ticker+=1
             for i in [4, 15, 17, 18]:
                if self.pinStates[i] == 1:
                    self.pinOnOff([i])
      self.irangle = 45
    if self.irangle == -45:
      self.pinOnOff([4, 18])
      time.sleep(0.01)
      self.pinOnOff([4, 17])
      time.sleep(0.01)
      self.pinOnOff([15, 18])
      time.sleep(0.01)
      self.pinOnOff([4, 17])
      time.sleep(0.01)       
      scansteps -=1
      while scansteps>0:
          self.pinOnOff([15, 18])
          time.sleep(0.01)
          self.pinOnOff([4, 17])
          time.sleep(0.01)
          self.pinOnOff([15, 18])
          time.sleep(0.01)
          self.pinOnOff([4, 17])
          time.sleep(0.01)
          if scansteps%8==0:
               scanarray[scancounter]=self.VoltagetoDistance(1)
               scancounter+=1
          scansteps -=1
          if ticker%2==0:
             self.setFIODrive(4,1)
          else:
             self.setFIODrive(4,0)
             drivecounter+=1
          ticker+=1
          for i in [4, 15, 17, 18]:
                if self.pinStates[i] == 1:
                    self.pinOnOff([i])
    else:
         self.pinOnOff([4, 15])
         time.sleep(0.01)
         self.pinOnOff([4, 17])
         time.sleep(0.01)
         self.pinOnOff([15, 18])
         time.sleep(0.01)
         self.pinOnOff([4, 17])
         time.sleep(0.01)        
         actualTicks -= 1
         while actualTicks>0:
             self.pinOnOff([15, 18])
             time.sleep(0.01)
             self.pinOnOff([4, 17])
             time.sleep(0.01)
             self.pinOnOff([15, 18])
             time.sleep(0.01)
             self.pinOnOff([4, 17])
             time.sleep(0.01)        
             if scansteps%8==0:
               scanarray[scancounter]=self.VoltagetoDistance(1)
               scancounter+=1
             scansteps -=1
             if ticker%2==0:
                self.setFIODrive(4,1)
             else:
                self.setFIODrive(4,0)
                drivecounter+=1
             ticker+=1
             for i in [4, 15, 17, 18]:
                if self.pinStates[i] == 1:
                    self.pinOnOff([i])
    distancetraveled = (drivecounter-4)/6
    return scanarray, distancetraveled
        
            
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
 

  def reset(self):
      self.UhOh()
      if self.angle >0:
          self.TurnMotor(self.angle, "Left")
      if self.angle <0:
          self.TurnMotor(abs(self.angle), "Right")
      self.TurntoAngle(0)

  "Pathfinding Code"    
    
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
    angle_increment = 10
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
    ###here is just for trial
    print(ir_data);
    return ir_data
   
  



  
  
  def sensor_loop(self): #This is the main sensing decision making code
    front_min = 40 # cm Minimum values telling the car when to stop
    while self.WALK(): #I am using this value as an internal stopflag because I couldnt get the global one to work. 
        front_dist = self.VoltagetoDistance(0) #scan the front 
        print(front_dist)
        if front_dist >= front_min: #if we still have space drive forward
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
                    DriveMotor(100, "Backward")
                    break

                        
 
               
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
       '''
       if 20 < front_dist < 100: #Drive forward, if the car is happy
           self.DriveMotorCM(1, "Forward")
           i += 1
       if front_dist < 20: #If the front sensor gets upset, default back to the sensor loop. 
           self.sensor_loop() 
           return
       '''
    print("Checking the Hip")
    self.irToDest() #Turn the scope 
    while self.VoltagetoDistance(1) < 100:
       self.DriveMotorCM(3, "Forward")
       self.irToDest()
    self.reset()
    self.sensor_loop()
    return
              
               

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





