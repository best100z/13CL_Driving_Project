# -*- coding: utf-8 -*-
"""
Created on Fri May 12 15:05:03 2023

@author: jaspe
"""
import u3

handle1= u3.U3()

import time

def setFIO(channel, state):
    handle1.setDOState(channel, state);
    
def Step(n):
    handle1.setDOState(5,1)
    i=0
    for i in range (n):
        handle1.setDOState(4,1)
        time.sleep(0.01)
        handle1.setDOState(4,0)
        time.sleep(0.01)
        i += 1
    
    
