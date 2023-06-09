'''
This code is taken from the Final Running Code and will detail individual lines. This code can be edited without changing how the robot
will run, making it very useful for testing thigns in the class definition and state variables.
'''

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
        
        
