
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
