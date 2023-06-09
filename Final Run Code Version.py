import numpy as np
import RPi.GPIO as gpio
import u3
import time
import threading
import queue



"""
@1: Define everything in a class for ease of storing state variables
"""

class piRobot():
  def __init__(self):
    #Sets the numbering scheme for the Raspberry Pi pinouts to the number scheme for just the ones labelled GPIO 0-27 - see diagrams
    #The other option is gpio.BOARD, which numbers every pin 1-40
    gpio.setmode(gpio.BCM)
    
    #This line prevents the RPi from giving a warning every time the code is run more than once without restarting
    #It can be removed with no issue
    gpio.setwarnings(False)
    
    #This loop sets every GPIO pin on the RPi to a digital in/out pin
    for i in range(28):
      gpio.setup(i, gpio.OUT)
      
    #This state variable stores whether each GPIO pin is set to Off (0) or On (1)
    self.pinStates = np.zeros(28)
    
    #This state variable stores the direction the pivoting IR sensor is facing relative to the car
    #A value of 0 is straight ahead, positive is left, and negative is right
    self.irangle = 0 
    
    #This variable stores the destination that the car is moving towards
    #This variable is updated every time the car moves
    self.destination = np.array([0,0])
    
    #This stores the direction that the car is moving in, relative to the x-axis as defined by the destination
    #Sign conventions are the same as the IR sensor
    self.heading = 0 
    
    #These lines initialize the Labjack
    self.labjack = u3.U3()
    self.labjack.configU3()    
    
    #This stores angle for the steering column, it only turns to 65 degree to either left or right; 
    #Acts as a protection and as a reminder if you forget to return the front wheel to orignial position
    self.angle = 0
    
    #This is the angle we want one slice of the 180degree ScopeScan() to be;
    self.scanslice = 10
    
    #This is the distance at which the avoid loop will trigger
    self.sensorloopTolerance = 30 
  
  #All of the below functions will just update the state variables stored in the class
  def update_angle(self, angle):
    self.angle = angle
      
  def update_irangle(self, angle):
    self.irangle = angle
    
  def update_destination(self, x, y):
    self.destination = [x,y]

  def update_heading(self, angle):
    self.heading = angle
  
  def update_fineslice(self, angle):
    self.fineslice = angle
  
  def update_sensorloopTolerance(self, dis):
    self.sensorloopTolerance = dis

  '''
    pinOnOff:
      input: array of pins
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
  #This is the angle of a single step on one of the stepper motors
  stepAngle = 360/4096*8
  
  #Changes the state variable to match
  self.update_irangle(self.irangle+angle)
  
  #Print is for debugging purposes
  print("ir to", self.irangle)
  
  #Finding the number of steps needed
  actualTicks = abs(angle)/stepAngle
  
  #These pinouts are detailed in the diagrams folder
  if angle >= 0:
      #The first step is different because GPIO pins 4 and 15 need to be on for the first step, but 4 is already on at the end
      #So, the second step is different. This is the same for turning right instead of left.
      #This also is the same case for the steering column motor
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
          
  #This block turns all pins off because remaining on could damage the motor
  for i in [4, 15, 17, 18]:
    if self.pinStates[i] == 1:
      self.pinOnOff([i])  

'''
  TurntoAngle
    input: angle: positive to the right, negative to the left, measured from centerline of the car.
    detail: make the scanning ir sensor turn to the angle
'''

def TurntoAngle(self,angle): 
    current_angle = self.irangle
    
    #Finding the turning angle needed
    final_dest = angle - current_angle
    
    #Turns the ir sensor to the angle entered
    self.irMotor(final_dest)


'''
  TurnMotor(Front Wheel)
    detail: this code is only used to turn the (front wheel) to either 65 degrees to the left or to the right
    ***has protection for the angle so it doesn't break the front motor
'''

def TurnMotor(self, angle = 0,):   
  #Updates the state variable
  self.update_angle(self.angle+angle)  
  
  #print for debugging
  print("front motor to", self.angle)
  
  #This block makes sure the steering column doesn't turn too far
  if abs(self.angle) > 65:
      self.UhOh()
      print("Too Many Degrees Dont Break the Car")
      self.update_angle(self.angle-angle)
      return
    
  #For comments on code below this, see irMotor above
  #Pins are changed from the irMotor - see diagrams
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
   
  #This block turns all pins off to make sure none are left on - could screw with stepper motor
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

#This code will either set a Labjack FIO port high (1) or (0)
#Channel 5 corresponds to the direction pin on the DRV8825
#Channel 4 corresponds to the step pin on the DRV8825
def setFIODrive(self, channel, state): 
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
      
  #Turns everything off as a default
  self.setFIODrive(4,0)
  self.setFIODrive(5,0)

def DriveMotorCM(self, cm = 0, Direction = "Forward", boolean = True):
  #This relationship was found by experiment since it is affected by friction
  steps = int(6*cm)
  self.DriveMotor(steps, Direction)
  
  #Finds how far the car has moved relative to the destination
  xmovement =  np.cos(self.heading*np.pi/180)*cm
  ymovement = np.sin(self.heading*np.pi/180)*cm
  
  #This block changes the destination due to the movement of the car
  if boolean:
    self.update_destination(self.destination[0]-xmovement, self.destination[1] - ymovement)
    
  #Print for debugging
  print("destination is", self.destination)       
  
  #Returns the actual distance the car has travelled
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
  #If the angle is less than 3, it causes problems
  if abs(angle) <= 3:
    return 0
  
  #converts the angle to radians and finds distance
  distance = ((angle/2)*np.pi/180)*45
  
  #print for debugging
  print(distance)
  
  #Turns the car one direction
  self.TurnMotor(65*angle/abs(angle))
  
  #Finds the actual distance travelled
  actualtravel = self.DriveMotorCM(abs(distance), "Forward", boolean)
  actualangle=((actualtravel/45)*2)*180/np.pi
  
  #Updates state variable
  self.heading += actualangle
  
  #Turns wheels the other way
  self.TurnMotor(130*-1*angle/abs(angle))
  
  #printing for debugging
  print("heading toward", self.heading)
  
  #Backs up
  self.DriveMotorCM(abs(distance), "Backward", boolean)
  
  #Turns wheels back to straight
  self.TurnMotor(65*angle/abs(angle))
  
  #Returns the actual angle turned
  return actualangle
        
  
  """
  @3: Pathfinding Towards the Destination
  """
  
 '''
  HeadingtoDest:
    detail: This returns the angle that the destination is at, calculated similar to the heading
'''
def HeadingtoDest(self):
  #If the car lies on the x axis, the heading is the angle needed to point to the destination
  if self.destination[0] == 0:
      turnangle = -self.heading
      return turnangle
    
  #If the heading is not on the x axis, we find the angle and return it
  else:
      tantheta = self.destination[1]/self.destination[0];
      theta = np.arctan(tantheta) * 180/np.pi 
      turnangle = theta - self.heading 
      return turnangle


'''
  TurnToDest:
    detail: Turn the robot to the destination and updates the heading
'''
def TurnToDest(self):
  #Uses HeadingtoDest to find the destination
  turnangle = self.HeadingtoDest()
  
  #Print for debugging
  print(turnangle)
  
  #Angle<3 is too small for the car to perform reliably
  if abs(turnangle) < 3:
      return
    
  #Turns the car in place (TurnInPlace updates the angle)
  self.TurnInPlace(turnangle)  

'''
irToDest:
    Turns the IR sensor towards the destination
    Updates the ir angle
'''
def irToDest(self):
  #Uses HeadingtoDest to find destination
  headangle = self.HeadingtoDest()
  
  #Points the IR sensor at destination
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
    #checks to see if pin is on, and if so, it turns it off
    if self.pinStates[i] == 1:
      self.pinOnOff(i)

'''
  reset:
    detail: stop all motors,
            move the front wheel back to original position
'''      
def reset(self):
    #Includes UhOh (pins turn off)
    self.UhOh()
    self.TurnMotor(-self.angle)
      
  """
  @5: scanning
  """
  
  def getAIN(self, n=0):
    #Grabs the value of the analog input DAC(n) on the Labjack
    ain0bits, = self.labjack.getFeedback(u3.AIN(n))
    
    #Converts value above into a Voltage 0-5V and returns
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
    #calls getAIN to find voltage
    Voltage = self.getAIN(n)
    
    #By calibration, this was found to be the voltage that the IR sensors were unreliable at, so we return a maximum value 100
    if Voltage <=0.46:
        return 100, Voltage
      
    #If the voltage is in our usable range, we use the function found from experiment to find the distance
    x = np.log((Voltage-0.46)/3.67)/(-0.068)
    
    #Since we don't need to find distances greater than 50cm, we convert them all to 100
    if x>50:
        x = 100
        
    #Returns distance and voltage
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
  
  #Iterates over each value in the array
  for i in range(len(array)):
    counter = 0
    
    #Iterates over each value again
    for j in range(len(array)):
      
      #Skips the iteration if it's the same value, and checks to see if the difference is within the defined error
      if j!=i and abs(array[i] - array[j])<=error:
        
        #If it is, add a count
        counter = counter + 1
      
      #Checks to see it this is the most frequently within error of all values checked in first iteration so far
      if maxv < counter:
        
        #If it is, set this one to the mode
        maxv = counter
        maxI = i
        
  #Returns the final mode
  return array[maxI]

def IRValMode(self, ain = 0, slp = 30, error = 0.04):
  temp = 0
  tempArray = np.zeros(slp)
  tempsum = 0
  data = 0
  counter = 0
  
  #Takes a certain number of measurements (slp)
  for j in range(slp):
    time.sleep(0.01)
    
    #Finds a distance for each measurement
    temp, tempArray[j] = self.VoltagetoDistance(ain)
  
  #Finds the mode of the measured distances
  data = self.mode(tempArray, error)
  
  #Iterates over every measurement
  for j in range(slp):
    
    #Sums each value that is close enough to the mode
    if abs(data-tempArray[j]) <=error:
      tempsum = tempsum + tempArray[j]
      counter = counter + 1
  
  #Finds the mean of the close values
  data = tempsum/counter
  
  #Same returns as VoltagetoDistance
  if data < 0.46:
    return 100
  data = np.log((data-0.46)/3.67)/(-0.068)  
  if data > 50:
    data = 100
  return data

def ScopeScan(self, towards = 0):
  #Finds the number of scans to take
  num_steps = 180//self.scanslice
  
  ir_data = np.zeros(num_steps)
  ir_vol = np.zeros(num_steps)
  
  #Points IR sensor 90 degrees to left of the wanted angle
  self.TurntoAngle(towards + 90)
  
  #Makes an array of num_steps values using the mode to accoutn for errors
  for i in range(num_steps):
      ir_data[i] = self.IRValMode(1)
      self.irMotor(-self.scanslice)
      
  #Finished back at center
  self.TurntoAngle(0)
  
  #Returns/prints distances
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
    Reachable:
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
        front_dist, temp = self.VoltagetoDistance(0) #scan the front 
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
            return self.avoid_loop(front_dist)
    #As exiting the while loop, we already reached our destination;
    print("Destination has been reached");
  '''
    avoid_loop:
      input: distance to the nearest obstacle
      details:
        Allows the car to navigate around an obstacle. Passed the distance to said obstacle. 
        1. Uses ScopeScan to create an array of its surroundigs, and return the first clear angle closest to the destination.
        2. Turns to that angle plus a few degrees to guarantee the car passes
        3. Drives the distance required to pass that obstacle
        4. Turns IR sensor to look in direction of the destination --> If clear, passes DriveToDest
        5. Creates a scan centered on the destination 
        6. Checks again to make sure the direct path is clear
        7. Otherwise, we are facing a cave like obstacle, we will pass it to the escape loop to escape 
  '''        
 
  def avoid_loop(self, dist):
    print("avoid_loop") #debugging
    angle = (self.FindMinimumAngle(self.HeadingtoDest())) + self.HeadingtoDest(); #This tells the angle of the turn, measured from centerline. Negative values correspond to left turns
    i = 0
    front_min = 20 #in this loop since we only need to avoid the obstacle, so we only need to make sure we can have enough space to turn in place
    temp = 0
    print(f"Turning to angle {angle} degrees...")
    #These just turn the car to point away from the obstacle additional 20 degree to ofset some of the unceratainties, need fruther adjustment to make it more percsie
    if angle > 0: 
      self.TurnInPlace(abs(angle) + 20)
    elif angle < 0:
      self.TurnInPlace(angle - 20)
    while i*np.cos(angle/180*np.pi) < dist + 20: #Drive the car far enough to pass the distance to the obstacle
      front_dist, temp = self.VoltagetoDistance(0) #Keep scanning as it drives towards the obstacle
      if front_dist >= front_min: #If no obstacles, keep going
        self.DriveMotorCM(1, "Forward")
        i += 1
      else: #If there is obstacle, first ceck again 
        front_dist = self.IRValMode(0)
        if front_dist >= front_min:
          self.DriveMotorCM(1, "Forward")
          i += 1
        else: #If for sure bad, try another avoid loop with the full scan
          return self.avoid_loop(front_dist)
    #As exiting the while loop, we are already side to side with the obstacle
    angle = self.FindMinimumAngle(self.HeadingtoDest()) + self.HeadingtoDest(); #find the smallest angle that we can turn to deviated from the destination    
    if angle < -1000: #if there is not escape, then we just turn to escape loop
        self.reset();
        return self.escape_loop();
    self.TurnInPlace(angle) #if there is a place to turn to we turn to that angle
    If abs(self.heading - self.HeadingtoDest())<=3: #if we finaly go back to the origianl angle, just go back to drive straight, which means we already avoided the intened obstacle;
        self.reset();
        return self.sensor_loop();
    front_dist = self.IRValMode(0);
    return self.avoid_loop(50); #if we are not facing the destination, then we try to use the avoid loop to turn back to our original destination

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
    self.TurntoAngle(-90) #Turn the IR sensor towards the left to look for an exit
    side_dist = self.VoltagetoDistance(1) #This returns the dsitance 90 degrees towards the left
    i = 0;
    while i <= 30: #Reverse until there is a clear path of width 30cm to the left
        side_dist, temp = self.VoltagetoDistance(1)
        self.DriveMotorCM(1, "Backwards")
        if side_dist <100:
            i = 0;
        else:
            i = i + 1;
    self.TurnInPlace(-90) #Turn the car towards the opening
    self.reset() #Return all the motor towards the center
    self.DriveMotorCM(70); #Drive Towards the opening for 70cm, then trying to see if we can head back;
    front_dist = self.IRValMode(0); #how much the nearest barrier to our car.
    return self.avoid_loop(front_dist); #if there is barrier, then we try to avoid that, if there is no barrier, then we just try to return to the destination
          
'''
Below code can be uncommented when the car is meant to run
The arguments of the DriveToDest call is where you want the car to travel to (the destination)

p = piRobot()
p.DriveToDest([200,0])
'''


