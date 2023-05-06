import numpy as np
import RPi.GPIO as gpio

class robot():
  def __init__(self):
    gpio.setmode(gpio.BCM)
    for i in range(28):
      gpio.setup(i, gpio.out)
    self.pinStates = np.zeros(28)
    
  def pinOnOff(self, number):
    if self.pinStates[number]==0:
      gpio.output(number, gpio.HIGH)
      self.pinStates[number]=1
    else:
      gpio.output(number, gpio.LOW)
      self.pinStates[number]=0
   
   
  
