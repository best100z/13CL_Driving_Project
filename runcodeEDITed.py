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
    self.fineslice = 10; ###this is how fine do we want to slice the 180degree scan to be;
    self.sensorloopTolorance = 30;
    """
    skeptical if we need to keep it
    """
    #self.sensor_thread = threading.Thread(target=self.sensor_loop)
    #self.event_thread = threading.Thread(target=self.event_loop)
    #self.event_queue = queue.Queue() #defines the Queue variable
  
  """
  @1: Casual setting functions
  """
  
  def update_angle(self, angle):
    self.angle = angle
      
  def update_irangle(self, angle):
    self.irangle = angle
    
  def update_destination(self, x, y):
    self.destination = [x,y]

  def update_heading(self, angle):
    self.heading = angle
  
  def update_fineslice(self, angle):
    self.fineslice = angle;
  
  def update_sensorloopTolorance(self, dis):
    self.sensorloopTolorance = dis;

  '''
    pinOnOff:
      input: array of pins
      Detail: of the array of pins, turn it on if off, turn it off if on
  '''
  def pinOnOff(self, numbers):
    for number in numbers:
      if self.pinStates[number]==0:
        gpio.output(number, gpio.HIGH)
        self.pinStates[number]=1
      else:
        gpio.output(number, gpio.LOW)
        self.pinStates[number]=0
  
  """
  @2: Turning Motors
  """
  
  '''
    irMotor:
      input: (angle) to turn the ir motor and (direction) to turn
      detail: Turn the motor in the direction for the given angle
              update the self.irangle
  '''      
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
    TurntoAngle
      input: angle: positive to the right, negative to the left
      detail: make the scanning ir sensor pointing to the angle
  '''
        
  def TurntoAngle(self,angle): #This code turns the scope to a specific angle relative to the car
      current_angle = self.irangle
      final_dest = angle - current_angle
      if final_dest > 0:
          self.irMotor(abs(final_dest), "Right")
      else:
          self.irMotor(abs(final_dest), "Left")
          
  
  '''
    TurnMotor(Front Wheel)
      detail: this code is only used to turn the (front wheel) to either 65 degrees to the left or to the right
      ***has protection for the angle so it doesn't break the front motor
  '''
  
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
        
        
  '''
    #This cluster of Code is for the (Back Wheel) to turn
    DriveMotor:
      input # of steps, direction
      detail: drive the back motor by number of steps
      #This is really only used for DriveMotorCM
  
    DriveMotorCM:
      input cm: distance in cm; direction
      detail: drive the car certain cm either forward or backward
      ***updates the distance to the destination 
  '''
  
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
  

  
  
  '''
    TurnInPlace: (Turning)
      input: the (angle) of turning and to which (direction)
      detail: turn towards direction certain angles;
              ***this has an error of -2.5 ~ 0, always might be slightly less
  '''
  
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
  @3: Towards the destination
  """
  
  '''
    HeadingtoDest:
      detail: This provides how much do we need to turn to destination wrt our front
  '''
  def HeadingtoDest(self):
    tantheta = self.destination[1]/self.destination[0];
    theta = -np.arctan(tantheta) * 180/np.pi; ###here is minus because positive value of arctan is to the left of the +x-axis, so now, we make it negative, so it is represented in our sense
    turnangle = theta - self.heading; ### here the minus sign represents we are finding the distance from dest to where we are right now
    return turnangle;
  
  
  '''
    TurnTODest:
      detail: Turn the robot to the destination;
              updates the heading
  '''
  def TurnToDest(self):
    turnangle = self.HeadingtoDest();
    direction = "Left";
    if turnangle > 0:
        direction = "Right";
    if abs(turnangle) <3:
        return;
    self.TurnInPlace(abs(turnangle)); ###TurnInPlace updates the head angle       
  
  '''
  irToDest:
      Turns the IR sensor towards the destination
      Updates the ir angle
  '''
  def irToDest(self):
    headangle = self.HeadingtoDest(); 
    turnangle = self.irangle - headangle
    direction="Left"
    if turnangle > 0:
        direction = "Right";
    self.irMotor(abs(turnangle), direction) ###irMotor updates the irangle

  """
  @4: Reset
  """

  '''
    UhOh:
      detail: turn all the pins off
  '''
  def UhOh(self):   
    for i in range(28):
      if self.pinStates[i] == 1:
        self.pinOnOff(i)
        
  '''
    reset:
      detail: stop all motors,
              move the front wheel back to original position
  '''      
  def reset(self):
      self.UhOh()
      if self.angle >0:
          self.TurnMotor(self.angle, "Left")
      if self.angle <0:
          self.TurnMotor(abs(self.angle), "Right")
      self.TurntoAngle(0)
        
  """
  @5: scanning
  """
  
  def getAIN(self, n=0):
      ain0bits, = self.labjack.getFeedback(u3.AIN(n));
      ainValue = self.labjack.binaryToCalibratedAnalogVoltage(ain0bits, isLowVoltage=False, channelNumber = 0);
      return ainValue
  
  '''
    VoltagetoDistance
      input: n = 0 --> the front IR sensor
             n = 1 --> the scanning IR sensor
      detail: this function let us know the distance in CM detected by either sensor
              if either the voltage getting in is too small or the distance is greater than 50cm then return 100 stand for invalid
  '''
  def VoltagetoDistance(self, n=0):
      Voltage = self.getAIN(n)
      if Voltage <=0.46:
          return 100, Voltage
      x = np.log((Voltage-0.46)/3.67)/(-0.068)
      if x>50:
          x = 100
      return x, Voltage        
  
          
  '''
    #This cluster of codes is to scan the 180 degree:
    mode:
      input: an array of value and the tolorance for error;
      output: the value that has the most occurence within the error;
    IRValMode:
      input: which (AIN) are we trying to detect; how many data (slp) do we want to take; error of tolorance
      output: average of all the value within error
      detail: scan and sleep 0.01s for slp times;
              find the mode;
              return the average of all within the mode
    ScopeSCan:
      return an array of scan
      detail: slicing the entire front space of 180 degree to 18 pieces centered on towards, which is the direction relative the heading
              scan each piece and determine the distance to the obstacles,
              return all the distance in an array
      ****subfunction Mode and IRValMode
        since the IR sensor is not always working well, 
        so i used the (mode) to sort out the most abundant ones, 
        then average over all the voltage, 
        then convert the average voltage to distance
        this works good enough for it always returns the real distance +/- 2.5cm
  '''        
      
  def mode(self, array, error=0):
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
  
  def IRValMode(self, ain = 0, slp = 30, error = 0.04):
    temp = 0;
    tempArray = np.zeros(slp);
    tempsum = 0;
    data = 0;
    counter = 0;
    for j in range(slp):
      time.sleep(0.01);
      temp, tempArray[j] = self.VoltagetoDistance(ain);
    data = self.mode(tempArray, error);
    for j in range(slp):
      if abs(data-tempArray[j]) <=error:
        tempsum = tempsum + tempArray[j];
        counter = counter + 1;
    data = tempsum/counter;
    if data < 0.46:
      return 100;
    data = np.log((data-0.46)/3.67)/(-0.068)  
    if data > 50:
      data = 100;
    return data;

  def ScopeScan(self, towards = 0):
    angle_increment = self.fineslice;
    num_steps = 180//angle_increment; 
    ir_data = np.zeros(num_steps);
    ir_vol = np.zeros(num_steps);
    self.TurntoAngle(towards - 90);
    for i in range(num_steps):
        ir_data[i] = self.IRValMode(1);
        self.irMotor(angle_increment, "Right")
    self.TurntoAngle(0);
    print(ir_data);
    return ir_data         
  
  """
  @4: Logic and loops
  """

  '''
    WALK:
      return boolean
      detail: this determines if the destination is further than 5cm of the end goal
              true for not; false it is within 5cm;
  '''  
  def WALK(self):
    within = 25;
    yes = (self.destination[0]**2 + self.destination[1]**2)>=within;
    return yes;        
  
  '''
    Reacable:
      input: distance theoretically will travel;
      output: boolean if within the steps the car will reach within the 5cm radius of the end point
      detail: each time just theoretically move 1 cm forward, and use WALK to check if still need to walk
              True for reachable; false for non reachable
  '''
  
  def Reachable(self, distance):
    d = distance;
    x = self.destination[0];
    y = self.destination[1];
    while d>0:
      if self.WALK()==False:
        return True;
      x = x - np.cos(self.heading * np.pi / 180);
      y = y - np.sin(self.heading * np.pi / 180);
      d = d - 1;
    return False
  

  '''
    FindMinimumAngle
      input the tolorance of distance we can travel and towards which direction do we care about.
        here I set it to be 100, which is just greater than our scanning radius;
      detail:
        find two consecutive ones that we can move so it is going to be 20 degrees here always pick the second one (middle);
        from left to right to see which angle that is closest to the center, so only if the next one is closer--> change the index -->prefer (left)
  '''
  def FindMinimumAngle(self, towards = 0, tolorance = 100):
    irdata = self.ScopeScan(towards);
    minI = -1000;
    for i in range(len(irdata)-1):
      if i!=9 and i+1!=9 and i-1!=9 :
        if irdata[i]>=tolorance and irdata[i+1]>=tolorance:
          if abs(minI-9) > abs(i + 1 -9):
            minI = i + 1;
    return minI * 10 - 90;
    
  '''
    sensor_loop:
      the main loop that will run the car to the destination set by destination(x,y);
      !!!!always should end here
      1. this starts by turning to the direction of the end point
      2. then it goes into a while loop where only stops when reaching the destination
      3. scan the forward direection:
        case1: no barrier close enough within our scan--> move forward;
        case2: it seems like a block infront, but it is really a glitch in IR sensor --> move forward;
        case3: really something close to us-->avoid_loop_better
  '''

  def sensor_loop(self):
    front_min = self.sensorloopTolorance; # cm Minimum values telling the car when to stop
    self.TurnToDest(); ### always trying to move in a straight line to the destination
    temp = 0;
    while self.WALK(): ###true when the car is not within 5cm radius of the end point
        front_dist, temp = self.VoltagetoDistance(); #scan the front 
        print(front_dist)
        if front_dist >= front_min or self.Reachable(front_dist): ###if we still have space drive forward or if the destination is close enough
          self.DriveMotorCM(1, "Forward") ###Drive forward 1cm;
          print("vroom vroom") ###I think this is a funny way to tell us what it is doing.
          ###skeptical if I need to delete this
          time.sleep(0.05) ###This sleeps the code to give the motor time to do its thing. If it doesnt sleep it will make a massive queue of drives, so when the stop command comes its so late that the robot will just crash lol
        else: ###check again to make sure it is not the IR sensor giving us trouble
          front_dist = self.IRValMode(0);
          if front_dist >= front_min or self.Reachable(front_dist): ###if it is just the sensor is giving us trouble, then still marching forward
            self.DriveMotorCM(1, "Forward") ###Drive forward 1cm;
            print("vroom vroom") ###I think this is a funny way to tell us what it is doing.
            ###skeptical if I need to delete this
            time.sleep(0.05) ###This sleeps the code to give the motor time to do its thing. If it doesnt sleep it will make a massive queue of drives, so when the stop command comes its so late that the robot will just crash lol
          else: #front sensor isnt happy anymore. Passes to next layer of decision making.
            front_dist = front_dist - 10;
            return self.avoid_loop_better(front_dist);
  """
  """        
  def avoid_loop_better(self, dist):
    angle = (self.FindMinimumAngle(self.HeadingtoDest())) + self.HeadingtoDest(); #This tells the angle of the turn, measured from centerline. Negative values correspond to left turns
    i = 0;
    front_min = 20;
    temp = 0;
    print(f"Turning to angle {angle} degrees...");
    if angle > 0: #These just turn the car to point away from the obstacle
      self.TurnInPlace(abs(angle) + 10, "Right")
    elif angle < 0:
      self.TurnInPlace(abs(angle) + 10, "Left")
    while i*np.cos(angle/180*np.pi) < dist:
      front_dist, temp = self.VoltagetoDistance(0)
      if front_dist >= front_min: 
        self.DriveMotorCM(1, "Forward")
        i += 1;
      else:
        front_dist = self.IRValMode(0);
        if front_dist >= front_min:
          self.DriveMotorCM(1, "Forward");
          i += 1;
        else:
          return self.avoid_loop_better(front_dist);
    self.TurnToDest();
    front_dist = self.IRValMode(0);
    if front_dist >= self.sensorloopTolorance or self.Reachable(front_dist):
      return self.sensor_loop();
    return self.avoid_loop_better(front_dist);
          
          
          

p = piRobot();
p.update_destination(170, 0);
time.sleep(5);
print("go");
p.sensor_loop();
