# Project Description
The main goal of this project was to build a small scale self-driving car. This car would have several key capabilities. First, we wanted it to be able to drive to a specific set destination. Second, we needed the car to be able to avoid obstacles along the way to this destination. We expected the car to be able to perform all tasks on board. That is, as soon as it was plugged in/turned on, it would travel to its destination without any user input. To do this, we built the car, created a steering column and drive motor, and added several electronic components to perform the needed tasks. This README serves as an introduction into working with the car and as a guide to recreating the project.
So, why a self driving car? As evidenced by the popularity of Tesla, a car that can drive itself is a huge draw for consumers. Not only that, but it also has many possible applications in small scale robotics. However, we decided to focus more on the human aspect of this technology. Our car has a steering column that allows it to drive like an actual car on the road. Although our car is still a work in progress, with more testing we believe that it can overcome a large variety of obstacles it may encounter. We believe that the greatest advantage for our car is its destination tracking capabilities. Small scale distance measurement is a complicated field of active research which will be detailed in section 4, along with our solution.

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

<a name="code"></a>
# Pathfinding Code

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
