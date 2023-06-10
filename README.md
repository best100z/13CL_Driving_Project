# Project Description
The main goal of this project was to build a small scale self-driving car. This car would have several key capabilities. First, we wanted it to be able to drive to a specific set destination. Second, we needed the car to be able to avoid obstacles along the way to this destination. We expected the car to be able to perform all tasks on board. That is, as soon as it was plugged in/turned on, it would travel to its destination without any user input. To do this, we built the car, created a steering column and drive motor, and added several electronic components to perform the needed tasks. This README serves as an introduction into working with the car and as a guide to recreating the project.

So, why a self driving car? As evidenced by the popularity of Tesla, a car that can drive itself is a huge draw for consumers. Not only that, but it also has many possible applications in small scale robotics. However, we decided to focus more on the human aspect of this technology. Our car has a steering column that allows it to drive like an actual car on the road. We really wanted to see how far our car can get on a small scale without the aid of complicated devices like GPS and the other sensors Teslas use. Although our car is still a work in progress, with more testing we believe that it can overcome a large variety of obstacles it may encounter. 

## Table of Contents
1. [Building the Car](#build)
2. [Electronic Components](#components)
3. [Real World Challenges](#challenges)
4. [Pathfinding Code](#code)
5. [Next Steps](#next)
6. [Acknowledgements](#acks)

<a name="build"></a>
# Building the Car
## General Car Construction
To build the platform for the car, we used a heavily modified version of the LEGO 42123 set. The key features that remained the same were the driveshaft and the steering column. Of course, this LEGO set is not required. Beyond these two key details, much of the car is open to innovation. We used the Technic pieces that came with the set to construct the frame and mounted everything on that. To mount, we used a combination of rubber bands, Duct Tape, and minimal 3D printed parts. The main uses of 3D printing were for gears and IR sensor mountings. Rubber bands were used extensively to hold the main driving stepper motor down; however, this still resulted in skipping. Therefore, once the design of the car is finalized, a solution such as strong glue or more 3D printing would be recommended.
<img src="Diagrams_and_Images/Car Diagram.png" width=600>

Above is a diagram of the car as a whole. This diagram shows all of the main components that we will dive into further in section 3. Below are more details about the steering, driving, and IR sensing setups. 

## Steering Setup
<img src="Diagrams_and_Images/Steering Column Diagram.png" width=600>
Our setup uses a stepper motor attached to a gear using LEGO parts. The gray piece is attached to the stepper motor through friction, and then attaches to the gear using lego pieces. This gear then controls a simple lego steering column which is from the LEGO set, but could be adjusted to fit individual needs. The 65 degree protection found in the code is dependent on this specific setup, so adjust that parameter according to individual needs.

## Driveshaft Setup
<img src="Diagrams_and_Images/Drive Motor Diagram.png" width=600>
We ran our NEMA 14 Motor and 3D printed gear to a LEGO gear. This then runs to a T-joint which turns the axle, allowing both wheels to drive with a fixed differential. This was the main limiter for our ability to onboard, future steps would involve mounting this motor in a permanent way with a better connection, as this setup would often experience “skipping”, where the stepper motor would turn without moving the wheels. 

## IR Sensor Setup
<img src="Diagrams_and_Images/IR Sensor Setup Diagram.png" width=250>
One sensor is fixed, facing directly forwards. As these IR sensors essentially work in two dimensions, we did not run into issues with the lower sensor detecting the ground in front and returning faulty readings. The other sensor is attached to a small stepper motor using a 3D printed piece. Our design can be found on GitHub. This is what we refer to as the “scope” in the code, and is key for scanning and decision making functionality of the car. The stepper motor is attached to the chassis of the car through another 3D printed block with a groove that perfectly fits the stepper motor. 

<a name="components"></a>
# Electronic Components
1. [NEMA 14 Motor and DRV8825 Driver Board](#nema)
2. [Smaller Stepper Motors](#steps)
3. [IR Sensors](#ir)
4. [Raspberry Pi](#rpi)
5. [Labjack](#labjack)

<a name="nema"></a>
## NEMA 14 Motor and DRV8825 Driver Board
This setpper motor and driver board were chosen for a few reasons. The NEMA 14 motor has a compact size that allows it to easily fit on the car. The motor also outputs a significant amount of torque for its size, which is needed in order to onboard all of the components of the car. The driver board was chosen because it is very commonly used as an intermediary between microcontrollers such as our Raspberry Pi and the NEMA 14 motor. Lastly, this combination of driver board and motor can run on a very reasonable power supple of ~9V, which allows for the use of a 9V battery. As shown in the car diagram above, we instead used an external power supply for our tests because a battery's lifespan was a concern. Below is the pinout for the DRV8825 driver board and how we set up each pin.
<img src="Diagrams_and_Images/DRV 8825 Pinout.png" width=300>
* VMOT takes DC input from 9-25 Volts. Many document that the board is susceptible to voltage spikes and run a capacitor in parallel before hooking up the VMOT and GND. We did not do that, and ran into no issues with that using a DC Power Supply. 
* GND 1 is Motor Ground, GND 2 is Logic Ground.
* B2 Is second wire of second coil – Blue for the motor we used
* B1 is first wire of second coil – Red for the motor we used
* A1 is first wire of first coil – Black for the motor we used
* A2 is second wire of first coil – Green for the motor we used (This completes NEMA wiring)
* FAULT triggers SLEEP when pulled high. Defaults to low, left unconnected with no issue
* ENABLE – pull high to disable the chip, low to enable. Left unconnected
* MO, M1, M2 – Controls step size, refer to attached image:

<img src="Diagrams_and_Images/DRV 8825 Stepping.png" width=300>

* RESET – Pull low to disable the board, High to enable. Used 5V DC.
* SLEEP – Stops power draw to the motor when pulled low. Pull high to enable power draw, and in turn enable the motor.
* STEP – Send pulse to step motor once
* DIR – Pull high for one direction, leave low for the other. 

**NOTE:** Make sure to adjust the potentiometer in order to control the max current delivered by the board. If you do not do this correctly the motor WILL NOT work. To do this, measure the voltage of the small screw on the board, using a multimeter, with the other end attached to the ground of the board. This voltage reading should be half the maximum current of your stepper motor, i.e. if your stepper motor is rated for 1 Amp, you are aiming for a voltage of about 500mV. If it isn't, adjust this screw carefully until it is.  

<a name="steps"></a>
## Smaller Stepper Motors
For this project, we used two smaller stepper motors as discussed in section 2. These were 28BYJ-48 stepper motors, driven by ULN2003 driver boards. These specific parts were chosen due to size and ease of access. Any small stepper motor will likely work for these functionalities. However, there are some constraints. The stepper motor for the steering column needs to output enough torque to overcome the friction preventing the wheels from turning. Also, you may want to consider a stepper motor/driver board combination that can full step quickly in order to make the car operate at higher speeds. For our project, these two motors were microstepped. This was done due to constraints of the electronics, but it also came with a significant advantage: precision. Weighing precision vs. speed is a key challenge of this project. 
Another advantage of this combination of motor/driver is that it is largely plug and play. The stepper motor has a combined cable that plugs directly into the driver board's output pins, so no testing is needed to ensure correct connections. More details on the pinouts for these driver boards are shown in the Raspberry Pi section.

<a name="ir"></a>
## IR Sensors
The infrared Sensors that we used in this project were Sharp GP2Y0A21YK sensors. These IR sensors were chosen because they provide information in the range 10-80 cm. The range that we will use in the project is around 30-50cm, so this is right in the middle of the range of this sensors. Another reason to use IR sensors instead of another sensor such as ultrasonic sensing it for its directionality. The IR sensor measures in a 10 degree triangle in front of the sensor. Because our project operates only in two dimensions, this is advantageous as the signal won't detect anything not in the plane of the car. 
Once the IR sensors had been chosen, we needed to calibrate them. This was done by measuring the voltage readout of the IR sensor at 5cm increments between 10cm and 80cm. Once this data was obtained, a regression was performed to find a voltage-distance relation. Upon performing fits for both power laws and exponentials, the exponential fit was preferred due to a smaller $\chi^2$. The relationship found was: $V=3.67e^{-0.068d}+0.46$, where V is the voltage and d is the distance. This relationship held under all testing. It worked similarly well for each different IR sensor, so calibration was consistent. Another test was then performed with 30 measurements taken at each distance, with the results shown below. 
<img src="Diagrams_and_Images/IR Sensor Calibration.png" width=600>

The error bars on this plot are $1-\sigma$ error bars for the 30 data points at each distance. Points correspond to the mean of the data points. The error bars are also multiplied by 100 to showcase how accurate this measurement is. Becuase the calibration not only worked for each different sensor, but also entirely different sets of data, we assume that it is very accurate under these controlled conditions. It is worth noting that all of these tests were performed with a cardboard target, which was the material used for the obstacles in our final tests. This relationship is expected to vary slightly depending on the material that is being detected. 

<a name="rpi"></a>
## Raspberry Pi
The Raspberry Pi is the brains of the car in this project. It contains all of the code and will perform all of the actions required of the car once turned on. However, along with the code, there is some setup required. First of all, the Pi needs to run the code on startup. The way we do this is by changing the bash file to run the code whenever a terminal is opened, and we add a line of code that opens a terminal window whenever the Pi is turned on. The reason that we do this instead of just running the code on startup is for debugging purposes. If the UI of the Pi is connected to a monitor in some way, you can view what the Pi is doing through the opened terminal window while it runs. The Raspberry Pi will also need some Python packages that are not included in most versions of Anaconda. However, the Python environment that comes installed on Raspbian OS (which we highly recommend as the OS) contains theses packages. For any other Raspberry Pi OS, you may need different instructions. 

I will detail here the instructions on how to get the Pi to run code on startup. First, navigate to a terminal window. The native terminal program on Raspbian OS works fine. Type the following command:
```console
sudo nano ~/.config/lxsession/LXDE-pi/autostart
```
This will open a text file that you can edit. At the bottom of this file, add the line:
```console
@lxterminal
```
This will make the terminal window open on startup. Close, save, and reboot the Pi. You should see a terminal window open on startup. Once this is done, you will need to edit the .bashrc file in order to make the Pi run code on startup. Do this by running:
```console
sudo nano /home/pi/.bashrc
```
And then adding the line:
```console
sudo python /home/pi/sample.py
```
You will need to edit this line in order to fit your specific naming conventions. /home/pi/sample.py will be replaced by the path of your specific python program. For our case, it was /home/robotgrouppi/13CL_Driving_Project/Final\ Run\ Code\ Version.py. Once you've got this set up, you should see the Pi running your program in a terminal window whenever you plug it in!

The Raspberry Pi's GPIO pins will need to be used in order to provide stepping instructions to the smaller stepper motors. The code for doing this is included in "Final Run Code Version.py" in lines 16-29 and 76-89. These codes corrspond to the specific pinouts that we chose for our Raspberry Pi, shown below.

<img src="Diagrams_and_Images/Raspberry Pi Pinout.png" width=900>

Any of the numbered GPIO pins, despite their secondary function, can be used for the purposes needed for this project. While the Raspberry Pi is very capable of controlling the logic pins on the smaller stepper motors, we also need a Labjack for some other purposes. 
<a name="labjack"></a>
## Labjack
While this project does not require a Labjack, we used one for its analog voltage readings and as a source of 5V DC current to power the smaller stepper motors. Here is the pinout that we used and our code is designed for. However, you can do this in any way you like:
<img src="Diagrams_and_Images/Labjack Pinout.png" width=400>

* VS – First IR sensor power
* VS – Second IR sensor power
* VS – First stepper motor power
* VS – Second stepper motor power
* GND – IR grounds, stepper motor grounds
* AIN0 – Fixed IR sensor data cable
* AIN1 – Scope IR Sensor data cable
* FIO4 – Connected to STEP of DRV 8825 Driver Board to drive main stepper motor
* FIO5 – Connected to DIR of DRV 8825 Driver Board


<a name="challenges"></a>
# Real World Challenges
For this project, the main challenge was that it takes place in the real world. If we lived in an idealized world with perfect no slip conditions and no friction when we don't want there to be friction, everything would be easy. That is obviously not the case. We ran into several key issues when experimenting with our car that needed to be solved. These solutions ranged from coding fixes to rubber bands. as we will detail in this section.

## IR Sensors
We ran into an issue with our scanners where they would often record data incorrectly. We believe this is due to how small voltage changes lead to large distance changes in the range of 40 to 50cm.
<img src="Diagrams_and_Images/IR Error.png" width=600>

Unfortunately, most of the time, that is the distance we are looking for obstacles in. We need about 40cm for the car to have enough space to turn. These small voltage fluctuations would lead to the car often seeing things that were not actually there. This was extremely problematic, as when the avoid loop starts the scope often won't pick anything up, leading to the car venturing off course. We solved this problem by having the car first stop and scan again. This is a more intensive scan, as it takes 30 readings. From there it finds the mode of the data. Then it takes all of the readings within 0.04cm of the mode and averages them. This is the distance that the sensor returns from this scan. This greatly improved the consistency of navigation, but highlights some of the problems with the IR sensors. 

## Motor Mounting
We consistently struggled with mounting the NEMA 14 Stepper motor, which resulted in the stepper motor often “skipping” over the LEGO piece. As all of our pathfinding is internal, this would lead to the car thinking it traveled further than it had. We used a short term solution for the duration of this project. We were able to mount the motor securely using a combination of LEGO pieces and rubber bands, as shown.
<img src="Diagrams_and_Images/Rubber Bands.png" width=300>


However, when we had all of the electronics onboard the motor would still skip. Our short term solution to this is just to carry the electronics alongside the car. Of course, neither of these solutions are ideal, and now that the design of the car is finalized our immediate next step would be to 3D print a better chassis and motor mount and fully onboard all of the technology. 

## Friction
One major issue we had was the friction of the car while it turned. Because there is more friction when the front wheels are turned than when they are not, the distance the car travels per step changes. This made it extremely hard to consider how the location is changing as the car is turning, as all of our location data was onboarded and dependent on a discrete correlation between steps and centimeters. Anything that adjusts this relationship, such as turning, introduced mistakes in getting to the final destination. Our solution for this was to have the car undergo a two point turn, traveling half the desired turn angle forwards, then the rest backwards. This maneuver would leave the car in the exact place it started, facing the desired angle. After doing this, it will start updating distance data using simple trigonometry to describe how the car moves relative to the destination. 

## Heading and Destination
As we were unable to create an external way to tell the car where it is, like a wireless beacon, due to time constraints, we had to handle geographic positioning through a discreet relationship between steps and centimeters. Every time we drive the car, always in 1cm steps, the car considers its heading relative to the positive x axis which is measured from the origin facing the obstacle. Positive angles always correspond to left turns. The car considers this heading as it drives to calculate where it is relative to the grid through trigonometry. This allows it to always know where it is, as well as the angle and distance to the destination. One technique we developed to reduce the chances for mistakes was to redefine the coordinate system every time the robot had a straight shot at the destination. Essentially, after every avoid loop, the car looks directly at the destination. If this path is clear, the car will turn to face it, and then update the coordinate system so that the destination is again on the positive x axis measured from the car. The car calculates the distance this point is from the original x and y values. This allows the car to dodge obstacles one at a time, reducing the chances of a breakdown in pathfinding.  


<a name="code"></a>
# Pathfinding Code
For our pathfinding code, we have three different loops that would allow our car to drive from a starting point to an ending point while dodging barriers along the way. Further thoughts need to be put in to find ways to dodge barriers while we are turning and also while we are driving backwards.

A general introduction to our pathfinding algorithms: we have three main loops called `sensor_loop`, `avoid_loop`, and `escape_loop`, each serving a separate role for driving our car, `sensor_loop` always drives us while we are facing towards the destination; `avoid_loop` kicks in when we are running in a path that is deviated from a straight path that runs towards the destination; `escape_loop` is our backup loop when we are running into a cave like barriers. One thing that is worth noticing is that we compute our distance to the destination using a coordinate system where we always update the direction we are facing and use the trigonometry to calculate the distance towards the final destination.

<img src="Diagrams_and_Images/Sensor Loop.jpg" width=800>

I will start with the main loop, which is our `sensor loop`. This is the main loop because we start our pathfinding process from here and always arrive at the destination in this loop. As mentioned before, we are only in this loop if we are facing the destination, which we can theoretically reach using only this loop if we are running in a course with no barriers. While in this loop, we are constantly driving forward with 1 cm steps (each time we step forward, we use coordinate system mentioned to record the distance the destination) and using our IR sensor(infrared sensor) to determine the distance to the nearest barriers (as mentioned in the electronics, our IR sensor works the best in the range of 10-50cm, so we ignore the barriers further than 50 cm away from us). We have tested that for a path that has no barriers in between our starting point and ending point the car can run using this loop independently. 

However, our project is to use the self-driving car to dodge obstacles, so we will possibly scan(`VoltagetoDistance()` converts the electric signals to the distance relative to our car) and find a barrier that is within 50 cm away from us. In this case we will stop our car, and since either our IR sensor or signal receiving labjack is not consistent, we made a proof check by scanning again using `IRValueMode()`(in this scanning, we used a specific method where we take 30 separate data of the voltage where each takes 0.01s, so a total of 0.3s; after that we will find the mode of those voltages, and average the voltages with some deviation from the mode). If it is our IR sensor glitching, then we will keep marching forward, otherwise we will need to deviate from the straight path towards the destination, which we will pass the information of the distance to the barrier to our `avoid_loop`.

<img src="Diagrams_and_Images/Avoid Loop.jpg" width=1100>

For our avoid loop, we are always given a distance to the barrier in front of us, so our first job is to determine how much we need to turn to avoid the obstacle. Our pathfinding algorithm here is rather naive—we are just trying to find the smallest deviation that we need to take from our straight path to the destination. We did this by our `FindMinimumAngle`, where we divide the entire front semicircle into 18 equal pieces with the same arc angle, so each will take 10 degrees (this is due to the limitation of our IR sensor that it can only tell as precise as 10 degrees), we will use the IR sensor that is connected to a stepper motor to slowly scan each section using `IRValueMode()`, which will give us a more accurate value with a lot less odds due to the inconsistency in our electronics. Since we know that our car is about 20 cm wide, which corresponds to about 15 degrees of angle on the circle of radius of 50 cm, we are looking for a 30 degree cavity which we are safe to say that our car can turn towards the middle of that cavity to dodge the barriers. Turning isn’t always easy because we are running in an obstacle course, where we need to take turning radius into consideration. So inspired by real life driving, we use `TurnInPlace` to do a three point turn (in order to turn most efficiently, we always rotate the front axle to the maximum angle possible; and then by experiment, we measured out the turning radius; and since we know that for the three point turning, we will shift the same amount of angle on the way forward and on the way back, so we take in the angle that we want to turn, divide it by half and times the radius to find the arc length, and finally, we drive the distance of the arc length intended; as a result, we turn our car facing the direction without shifting the actual position of our car too much). After that we will update our current angle that we are facing, because by trigonometry each centimeter we are moving does not represent the same displacement in x and y as before turning. Next step is to drive forward until our car is side by side along with the obstacle and while scanning to see if there are any more barriers in front at the same time. This is similar to sensor_loop, but in this case, we are only stopping if there is a barrier that is less than 20 cm away from our car because that is the enough space we need to turn our car, and we will do the same process where we will take a more accurate data for the distance of the obstacle. If we indeed have an obstacle in front, we will return the `avoid_loop` while passing the new parameter for the new barrier. This process will go on and on for as many obstacles that are in the way as the course is. Whenever we manage to actually get side by side with the obstacle we will make another 180 degree scan, where we will first find out the direction of the destination by our coordinate system, and we will then make the scan centered at the destination. We will figure out what’s the smallest deviation from a straight path towards the destination using `FindMinimumAngle`(what’s worth noticing here is that as we move deviate from the original straight path towards the destination, the path facing towards the destination will be constantly changing, but all we care about in this path finding algorithms is that we want to always find the minimum angle that deviates from the straight path to the destination at the local point).

<img src="Diagrams_and_Images/Destination Diagram.png" width=800>

As we turned to the angle that has the smallest deviation from the center use `TurnInPlace`. Then we will need to determine if we are facing towards the destination, if the answer is yes, then we will be happily returning to the sensor loop to keep driving towards the destination while checking for potential obstacles. However, if we are unable to face the destination, we will return to the avoid loop as if there is a barrier in front that is 50cm(this is the maximum distance our IR sensor can scan with accuracy), so we will either move forward for 50cm and then check if we can turn towards the destination or if there is a new barrier we will try to dodge it using avoid loop again, through this process we will eventually turn back to the destination and return to out sensor loop.

Lastly, there is a problem that we haven’t taken into consideration, where all the previous algorithms are about dodging general barriers, however, there is a problem if we are facing a barrier that has a cave like shape, where as we are driving towards the end of the cave, we will notice that we can’t face towards any possible forward direction. So in this case the `escape_loop` (shown below) is going to help us to dodge this specific barrier. The idea is straightforward where we will drive backwards. While we are driving backwards, we are scanning towards both left and right to seek an opening for 30cm. If we do find an opening, we will then turn towards that direction and drive for 50cm, and in the end we will return the job back to our avoid loop to finally move our direction back to facing the destination. One thing worth noting is that this escape loop can only help us to dodge really shallow cave-like obstacles, for the obstacles that are deep or really large, it is not really efficient.

<img src="Diagrams_and_Images/Escape Loop.jpg" width=800>

In general, the pathfinding algorithm so far is effective for really general block shape obstacles, for more complex-shaped obstacles we may need to incorporate moving with scanning, where we constantly update the entire view of the surrounding and it will require more complex logic and more sophisticated algorithms. 


<a name="next"></a>
# Next Steps
While this car is completely functional, it is important to note that everything in this project is still a work in progress. There are several plans for how we would change the car given more time and resources. These updates would combat some of the challenges that we are currently seeing, and they would also allow the car's applications to expand greatly. 
## Packaging the Car
In the car's current state, it must be tethered to the Raspberry Pi, Labjack, and Power Sources. This is due to the stepper motor “skipping” when there is a lot of weight on the car, as instead of turning the driveshaft, instead the motor will move within its mounting, causing the gears to skip over each other. The motor should have plenty of torque to move the car with all of the weight on it, and therefore a solution to this would enable us to mount all of the electronics directly to the vehicle. The easiest way to address this is with a strong glue or epoxy. We opted against this as our design was often changing and we did not want to permanently mount something only to realize it needs to change. A more aesthetic solution would be 3D printing a chassis for the car, leaving space in that chassis to also mount the electronic components. This is the immediate next step in our project, as the design of our car is finalized. 
## Better Sensor Technology
While our IR sensors were effective for a small, controlled course, in any other environments they would be extremely limiting, due to their 2D scans and inconsistent behavior. With a greater budget, using LIDAR (laser imaging, detection, and ranging) instead of an IR sensor would vastly improve functionality. Relying on very similar physics, LIDAR measures the time that it takes for a laser to be sent from the device, bounce off an object, and return. Therefore, implementing it into the current system would not require many modifications to achieve the same functionality as we have now, and would unlock much greater functionality down the line. Of course, we would need to modify our scripts to handle the 3D scans LIDAR technology would allow us to take, but this is one of the first steps in making this device effective in real world situations. This technology is what most autonomous vehicles in today's market use, and its implementation would allow us to handle 3D, uncontrolled environments. 
## Improved Destination Tracking
The destination tracking within the code can be described as rusty at best. While the coding solution provides a rough estimate for where the car is trying to get, it is only as accurate as we can get our distance tracking to be. Over longer distances, the error will get larger and larger because our coding solution can never be exact. There are issues due to step sizes, and the much larger issue of skipping steps and bouncebacks of the drive motor. The inaccuracy in the turn in place command due to hardware limitations also is a source of error.

Our solution to this issue is to completely remove the coding aspect of distance tracking. The car, ideally, will not have to calculate how far it is traveling. This can be done by having some sort of signal beacon at the destination and a receiver on the car. By using multiple receivers at different positions on the car, we can triangulate the location of the beacon and know where the destination is at all times. However, on small scales, this is incredibly difficult. GPS does not have the required accuracy at this range. Radio signals are similarly long range, and a change in their signal strength is not noticable over this distance and can be attenuated significantly by obstacles. Bluetooth/Wifi signals could be a possibility with strength of signal measurements, but the Raspberry Pi is not capable of receiving signals from multiple bluetooth receivers at the same time. This could be solved with more components, such as an arduino, but there is a better solution. 

The best method for this task is using ultra-wideband transceivers, specifically DWM1001C modules from Qorvo. These have numerous advantages over other methods of distance measurement. First of all, they can use time of flight measurements instead of signal strength. This means that they have no issue with attenuation by obstacles, which we expect in our project. They also are able to be used at our ranges because of this. Hypothetically, these could be calibrated to our environment to find very accurate distance measurements, but there are physical limitations due to the speed of light and circuit speeds. They can be used, though, for triangulation, since the Pi can receive data from multiple at once. To use these UWB modules, we will need several things. We will need a J-Link debugger, which can come at a significant cost. We will also need PCB boards for the modules and microsoldering equipment to attach them. We will also need to write firmware for the devices so they can perform the tasks needed. While this would take significant time, we expect it would be the ideal way to solve many of our issues. For more information about UWB, check https://www.qorvo.com/products/p/DWM1001C and https://github.com/Decawave/dwm1001-examples.

<a name="acks"></a>
# Acknowledgements
We would like to thank Professor Andrew Jayich, Eric Deck, and Robert Kwapisz for their help and advice with this project. We would also like to thank Raffi Shirinian, Max Kolevski, and the Instructional Lab Group at UCSB for their help with 3d printing and acquiring parts and Karsten Lansing and Tarun Kumar for allowing us to use their Raspberry Pis for this project. 
