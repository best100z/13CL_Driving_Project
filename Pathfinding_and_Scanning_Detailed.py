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
