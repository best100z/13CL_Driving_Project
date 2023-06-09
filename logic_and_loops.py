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


