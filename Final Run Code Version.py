import numpy as np
import RPi.GPIO as gpio
import u3
import time
import threading
import queue



"""
Define everything in a class for ease of storing state variables
Check Detailed_Class_Code.py for more detail
"""

class piRobot():
  def __init__(self):
    gpio.setmode(gpio.BCM)
    gpio.setwarnings(False)
    for i in range(28):
      gpio.setup(i, gpio.OUT)
    self.pinStates = np.zeros(28)
    self.irangle = 0 
    self.destination = np.array([0,0]) 
    self.heading = 0 
    self.labjack = u3.U3()
    self.labjack.configU3()    
    self.angle = 0
    self.scanslice = 10 
    self.sensorloopTolerance = 30 
  
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
  
  def update_scanslice(self, angle):
    self.scanslice = angle
  
  def update_sensorloopTolerance(self, dis):
    self.sensorloopTolerance = dis

  '''
    pinOnOff:
      Input: An array of pins
      Detail: Changes the state of every pin in the array
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
      Input: (angle) to turn the ir motor and (direction) to turn
      Detail: Turn the motor in the direction for the given angle
              update the self.irangle
  '''      
  def irMotor(self, angle = 0):
    stepAngle = 360/4096*8
    self.update_irangle(self.irangle+angle)
    print("ir to", self.irangle); 
    actualTicks = abs(angle)/stepAngle
    if angle >= 0:
        self.pinOnOff([4, 15])
        time.sleep(0.01)
        self.pinOnOff([4, 17])
        time.sleep(0.01)
        self.pinOnOff([15, 18])
        time.sleep(0.01)
        self.pinOnOff([4, 17])
        time.sleep(0.01);       
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
    if angle <= 0:
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
      input: angle: positive to the right, negative to the left, measured from centerline of the car.
      detail: make the scanning ir sensor turn to the angle
  '''
        
  def TurntoAngle(self,angle): #This code turns the scope to a specific angle relative to the car
      current_angle = self.irangle
      final_dest = angle - current_angle
      self.irMotor(final_dest)
          
  
  '''
    TurnMotor(Front Wheel)
      detail: this code is only used to turn the (front wheel) to either 65 degrees to the left or to the right
      ***has protection for the angle so it doesn't break the front motor
  '''
  
  def TurnMotor(self, angle = 0,):   
    self.update_angle(self.angle+angle)  
    print("front motor to", self.angle)
    if abs(self.angle) > 65:
        self.UhOh()
        print("Too Many Degrees Dont Break the Car")
        self.update_angle(self.angle-angle)
        return
    stepAngle = 360/4096*8;
    actualTicks = abs(angle)/stepAngle
    if angle >= 0:
        self.pinOnOff([9, 25])
        time.sleep(0.01)
        self.pinOnOff([9, 11])
        time.sleep(0.01)
        self.pinOnOff([25, 8])
        time.sleep(0.01)
        self.pinOnOff([9, 11])
        time.sleep(0.01)
        actualTicks = actualTicks - 1
        while actualTicks>0:
            self.pinOnOff([8, 25])
            time.sleep(0.01)
            self.pinOnOff([9, 11])
            time.sleep(0.01)
            self.pinOnOff([8, 25])
            time.sleep(0.01)
            self.pinOnOff([9, 11])
            time.sleep(0.01)
            actualTicks = actualTicks - 1
    if angle < 0:
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
      #Note, this calls the Labjack we used for this project. For future iterations, this should be converted to raspberry pi code. 
  
    DriveMotorCM:
      input cm: distance in cm; direction
      detail: drive the car certain cm either forward or backward
      ***updates the distance to the destination 
  '''
  
  def setFIODrive(self, channel, state): #Labjack specific
    self.labjack.setDOState(channel, state)
  
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
    self.setFIODrive(4,0)
    self.setFIODrive(5,0)
  
  def DriveMotorCM(self, cm = 0, Direction = "Forward", boolean = True):
    steps = int(6*cm)
    self.DriveMotor(steps, Direction)
    xmovement =  np.cos(self.heading*np.pi/180)*cm
    ymovement = np.sin(self.heading*np.pi/180)*cm
    if boolean:
      self.update_destination(self.destination[0]-xmovement, self.destination[1] - ymovement)
    print("destination is", self.destination)       
    actualtravel = steps/6
    return actualtravel
  

  '''
    TurnInPlace: (Turning)
      input: the (angle) of turning, measured in the standard way for this project.
      detail: Performs a two part turn, going forwards then backwards, leaving the car in the same position it iniated heading towards the input angle. 
      Updates the heading:
              ***this has an error of -2.5 ~ 0, always might be slightly less
  '''
  
  def TurnInPlace(self, angle=0,  boolean = False):
    if abs(angle) <= 3:
      return 0
    distance = ((angle/2)*np.pi/180)*45
    print(distance)
    self.TurnMotor(65*angle/abs(angle))
    actualtravel = self.DriveMotorCM(abs(distance), "Forward", boolean)
    actualangle=((actualtravel/45)*2)*180/np.pi
    self.heading += actualangle
    self.TurnMotor(130*-1*angle/abs(angle))
    print("heading toward", self.heading)
    self.DriveMotorCM(abs(distance), "Backward", boolean)
    self.TurnMotor(65*angle/abs(angle))
    return actualangle
        
  
  """
  @3: Pathfinding Towards the Destination
  """
  
  '''
    HeadingtoDest:
      detail: This provides how much do we need to turn to destination wrt our front
  '''
  def HeadingtoDest(self):
    if self.destination[0] == 0:
        turnangle = -self.heading
        return turnangle
    else:
        tantheta = self.destination[1]/self.destination[0];
        theta = np.arctan(tantheta) * 180/np.pi ###here is minus because positive value of arctan is to the left of the +x-axis, so now, we make it negative, so it is represented in our sense
        turnangle = theta - self.heading ### here the minus sign represents we are finding the distance from dest to where we are right now
        return turnangle
  
  
  '''
    TurnToDest:
      detail: Turn the robot to the destination and updates the heading
  '''
  def TurnToDest(self):
    turnangle = self.HeadingtoDest()
    print(turnangle)
    if abs(turnangle) < 3:
        return
    self.TurnInPlace(turnangle); ###TurnInPlace updates the head angle       
  
  '''
  irToDest:
      Turns the IR sensor towards the destination
      Updates the ir angle
  '''
  def irToDest(self):
    headangle = self.HeadingtoDest();
    self.TurntoAngle(headangle)
    

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
      self.TurnMotor(-self.angle)
      
  """
  @5: scanning
  """
  
  def getAIN(self, n=0): #again, this is labjact specific code
      ain0bits, = self.labjack.getFeedback(u3.AIN(n))
      ainValue = self.labjack.binaryToCalibratedAnalogVoltage(ain0bits, isLowVoltage=False, channelNumber = 0)
      return ainValue
  
  '''
    VoltagetoDistance
      input: n = 0 --> the front IR sensor
             n = 1 --> the scanning IR sensor
      detail: this function let us know the distance in CM detected by either sensor
              if either the voltage getting in is too small or the distance is greater than 50cm then return 100 stand for invalid.
              We included these due to the physical limitations of our IR sensors. Also, the function comes from our experimental fits, 
              it may vary depending on type of IR sensor. 
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
    #This cluster of code provides the scanning funcationality of our car:
    mode:
      input: an array of value and the tolerance for error
      output: the value that has the most occurence within the error
    IRValMode:
      input: which (AIN) are we trying to detect; how many data (slp) do we want to take; error of tolerance
      output: average of all the value within error
      detail: scan and sleep 0.01s for slp times
              find the mode
              return the average of all within the mode
    ScopeScan:
      return an array of scan
      detail: slicing the entire front space of 180 degree to 18 pieces centered on an angle (towards), which is the direction relative the heading
              scan each piece and determine the distance to the obstacles,
              return all the distance in an array
      ****subfunction Mode and IRValMode
        since the IR sensor is very susceptible to small voltage changes, 
        this code uses the (mode) to sort out the most abundant ones, 
        then average over all the voltage, 
        then convert the average voltage to distance
        this works good enough for it always returns the real distance +/- 2.5cm
  '''        
      
  def mode(self, array, error=0):
    counter = 0
    maxv = -100
    maxI = -1
    for i in range(len(array)):
      counter = 0
      for j in range(len(array)):
        if j!=i and abs(array[i] - array[j])<=error:
          counter = counter + 1
        if maxv < counter:
          maxv = counter
          maxI = i
    return array[maxI]
  
  def IRValMode(self, ain = 0, slp = 30, error = 0.04):
    temp = 0
    tempArray = np.zeros(slp)
    tempsum = 0
    data = 0
    counter = 0
    for j in range(slp):
      time.sleep(0.01)
      temp, tempArray[j] = self.VoltagetoDistance(ain)
    data = self.mode(tempArray, error)
    for j in range(slp):
      if abs(data-tempArray[j]) <=error:
        tempsum = tempsum + tempArray[j]
        counter = counter + 1
    data = tempsum/counter
    if data < 0.46:
      return 100
    data = np.log((data-0.46)/3.67)/(-0.068)  
    if data > 50:
      data = 100
    return data

  def ScopeScan(self, towards = 0):
    num_steps = 180//self.scanslice
    ir_data = np.zeros(num_steps)
    ir_vol = np.zeros(num_steps)
    self.TurntoAngle(towards + 90)
    for i in range(num_steps):
        ir_data[i] = self.IRValMode(1)
        self.irMotor(-self.scanslice)
    self.TurntoAngle(0)
    print(ir_data)
    return ir_data         
  
  """
  @6: Logic and loops
  """

  '''
    WALK:
      return boolean
      detail: this determines if the destination is further than 5cm of the end goal
              true for not; false it is within 5cm
  '''  
  def WALK(self):
    within = 25
    yes = (self.destination[0]**2 + self.destination[1]**2)>=within
    return yes
  
  '''
    Reacable:
      input: distance theoretically will travel
      output: boolean if within the steps the car will reach within the 5cm radius of the end point
      detail: each time just theoretically move 1 cm forward, and use WALK to check if still need to walk
              True for reachable; false for non reachable
  '''
  
  def Reachable(self, distance):
    d = distance
    x = self.destination[0]
    y = self.destination[1]
    while d>0:
      if self.WALK()==False:
        return True
      x = x - np.cos(self.heading * np.pi / 180)
      y = y - np.sin(self.heading * np.pi / 180)
      d = d - 1
    return False
  

  '''
    FindMinimumAngle
      input the tolerance of distance we can travel and towards which direction do we care about.
        here I set it to be 100, which is just greater than our scanning radius
      detail:
        find two consecutive ones that we can move so it is going to be 20 degrees here always pick the second one (middle)
        from left to right to see which angle that is closest to the center, so only if the next one is closer--> change the index -->prefer (left)
  '''
  def FindMinimumAngleOld(self, towards = 0, tolerance = 100):
    irdata = self.ScopeScan(towards)
    minI = -1000
    for i in range(len(irdata)-1):
      if towards !=0 or (i!=9 and i+1!=9 and i-1!=9):
        if irdata[i]>=tolerance and irdata[i+1]>=tolerance:
          if abs(minI-9) > abs(i + 1 -9):
            minI = i + 1
    return -(minI * 10 - 90)


  

  def FindMinimumAngle(self, towards = 0, tolerance = 100):
    ir_data = self.ScopeScan(towards) 
    middle_index = len(ir_data) // 2
    for i in range(len(ir_data)):
        left_index = middle_index - i
        right_index = middle_index + i
        if left_index >= 0 and ir_data[left_index] >= 100:
            return 10 * i + towards
        if right_index < len(ir_data) and ir_data[right_index] >= 100:
            return -10 * i + towards
    return self.escape_loop(self)
    
  '''
    sensor_loop:
      the main loop that will run the car to the destination set by destination(x,y)
      !!!!always should end here
      1. this starts by turning to the direction of the end point
      2. then it goes into a while loop where only stops when reaching the destination
      3. scan the forward direection:
        case1: no barrier close enough within our scan--> move forward
        case2: it seems like a block infront, but it is really a glitch in IR sensor --> move forward
        case3: really something close to us-->avoid_loop
  '''

  def sensor_loop(self):
    print("sensor_loop")
    front_min = self.sensorloopTolerance; # cm Minimum values telling the car when to stop ### always trying to move in a straight line to the destination
    temp = 0
    while self.WALK(): ###true when the car is not within 5cm radius of the end point
        front_dist, temp = self.VoltagetoDistance() #scan the front 
        print(front_dist)
        if front_dist >= front_min or self.Reachable(front_dist): ###if we still have space drive forward or if the destination is close enough
          self.DriveMotorCM(1, "Forward") ###Drive forward 1cm
          print("vroom vroom") ###I think this is a funny way to tell us what it is doing, helpful for debugging.
          time.sleep(0.05) ###This sleeps the code to give the motor time to do its thing
        else: ###check again to make sure it is not the IR sensor giving us trouble
          front_dist = self.IRValMode(0)
          if front_dist >= front_min or self.Reachable(front_dist): ###if it is just the sensor is giving us trouble, then still marching forward
            self.DriveMotorCM(1, "Forward") ###Drive forward 1cm
            print("vroom vroom") 
            time.sleep(0.05)  
          else: #front sensor isnt happy anymore. Passes to next layer of decision making.
            front_dist = front_dist - 10
            return self.avoid_loop(front_dist)
  """
    avoid_loop:
      Allows the car to navigate around an obstacle. Passed the distance to said obstacle. 
      1. Uses ScopeScan to create an array of its surroundigs, and return the first clear angle closest to the destination.
      2. Turns to that angle plus a few degrees to guarantee the car passes
      3. Drives the distance required to pass that obstacle
      4. Turns IR sensor to look in direction of the destination --> If clear, passes DriveToDest
      5. Creates a scan centered on the destination 
      6. Checks again to make sure the direct path is clear --> If clear, passes DriveToDest
      7. Otherwise, will pick the closest clear path, and returns sensor loop to keep car going until it runs into another obstacle. 
  """        
 
  def avoid_loop(self, dist):
    print("avoid_loop") #debugging
    angle = (self.FindMinimumAngle(self.HeadingtoDest())) + self.heading; #This tells the angle of the turn, measured from centerline. Negative values correspond to left turns
    i = 0
    front_min = 20
    temp = 0
    print(f"Turning to angle {angle} degrees...")
    if angle > 0: #These just turn the car to point away from the obstacle
      self.TurnInPlace(abs(angle) + 20)
    elif angle < 0:
      self.TurnInPlace(angle - 20)
    while i*np.cos(angle/180*np.pi) < dist: #Drive the car far enough to pass the distance to the obstacle
      front_dist, temp = self.VoltagetoDistance(0) #Keep scanning as it drives towards the obstacle
      if front_dist >= front_min: #If okay, keep going
        self.DriveMotorCM(1, "Forward")
        i += 1
      else: #If not okay, first ceck again 
        front_dist = self.IRValMode(0)
        if front_dist >= front_min:
          self.DriveMotorCM(1, "Forward")
          i += 1
        else: #If for sure bad, try another avoid loop with the full scan
          return self.avoid_loop(front_dist)
    goAngle = self.HeadingtoDest() #Find the heading to the destination relative to the cars new position
    self.TurntoAngle(goAngle) #Turn scope to look at destination
    front_dist = self.IRValMode(1)
    if front_dist >= 100: #If path is clear, reset the scope and call DriveToDest
        self.TurntoAngle(0)
        return self.DriveToDest(self.destination)
    else: #Scan the environment
        angle = self.FindMinimumAngle(self.HeadingtoDest())
        self.TurnInPlace(angle)
        front_dist = self.IRValMode(0)
        if self.Reachable(front_dist): #If it is okay, drive to the closest angle to the destination
            self.TurnInPlace(angle)
            return self.sensor_loop()
        elif abs(self.HeadingtoDest())<=3 and front_dist >= self.sensorloopTolerance:
          self.DriveToDest(self.destination) #If the car is pretty much pointed at it drive to the destination
          return self.sensor_loop()
        return self.avoid_loop(self.VoltagetoDistance(0)) #Otherwise just try again
    
    
'''
    escape_loop:
    Called when a scope scan returns no acceptable values, meaning that car is surrounded on all sides. 
    Used to hopefully escape said situations, although relies on luck.
    1. If stuck, turn the scope 90 degrees left, a direciton chosen arbitrarily
    2. Reverse until the scope reads it is clear --> If it crashes here, I'm not sure how to adress the situation without either another IR sensor on the back, 
    or wireless tech to know if it crashed. This is achievable, but not in the state of the project as of 6/7/23'
    3. Once clear, drive a foot to the left, or until you run into another obstacle
    4. Turn to face the destination and head off.

'''

  def escape_loop(self):
    self.TurntoAngle(-90) #Arbitrarily choose to look directly left.
    side_dist = self.VoltagetoDistance(1)
    while side_dist <=100: #Reverse until there is a clear path to the left
        side_dist, temp = self.VoltagetoDistance(1)
        self.DriveMotorCM(1, "Backwards")
    self.TurnInPlace(-90)
    self.reset()
    i = 0
    front_dist = self.VoltagetoDistance(1)
    while i<30 and front_dist>= 100:
        self.DriveMotorCM(1, "Forward")
        i += 1
    return self.DriveToDest(self.destination)
        

"""
    DriveToDest:
    Code used to minimize error:
    We start our runs by defining a coordinate system with the car at the origin and the destination on the positive x axis
    As the car drives, the car moves along this grid as it avoids obstacles. We found that often the car was extremely
    effective at avoiding the first obstacle, but would get extremely confused at subsequent ones. Our solution 
    was to redefine the coordinate system after each obstacle. 
    1. The code registers the cars current position and heading
    2. The car then turns to the destination
    3. the car measures the distance to the destination
    4. the car updates the destination so that it is now at (distance, 0), resetting the positive x axis
    5. the car resets its heading, then calls sensor loop
"""
  def DriveToDest(self, args):
    x,y = args
    self.update_destination(x,y)
    distance = np.sqrt(self.destination[0]**2 + self.destination[1]**2)
    print(distance)
    self.TurnToDest()
    self.update_destination(distance,0)
    print(self.destination)
    self.heading = 0
    if self.WALK():
        return self.sensor_loop()
    else:
        return
          
          
'''
Below code can be uncommented when the car is meant to run
The arguments of the DriveToDest call is where you want the car to travel to (the destination)

p = piRobot()
p.DriveToDest([200,0])
'''



