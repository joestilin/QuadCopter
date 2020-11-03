# QuadCopter
Quadcopter control on Arduino

This code runs on the popular Arduino microcontroller series. While the ```mordecai.ino``` sketch is written for the Arduino Uno
board, the code can easily be expanded to work with boards having more pins, features, and memory.

The easiest way to get up and running is by compiling the code and loading it onto your Arduino board with Arduino's IDE.

## Why "Mordecai"?
Mordecai is the name of the hawk in the movie _Royal Tannenbaums_. It is also the name for Joe Stilin and Peter Maag's senior thesis at
Princeton, an RC plane controlled by an Arduino Uno board and Android phone. The spare Arduino survived the project and now lives on in
this next reincarntion of the noble Mordecai, aboard a quadrotor.

## Installation
1. Download and install the Arduino IDE for your system from https://www.arduino.cc/en/main/software

2. Your system will have created an ```Documents/Arduino``` folder. Clone the repository into this folder.

```bash
$ git clone https://www.github.com/joestilin/QuadCopter

```

3. Move the ```/Mordecai_Library``` folder into ```/Documents/Arduino/libraries```

4. Open the Arduino IDE and open the ```/Documents/Arduino/Mordecai/Mordecai.ino``` sketch. ```verify``` to make sure the code 
compiles cleanly.

For more information on installing Arduino libraries, refer to the section on Manual Installation in: https://www.arduino.cc/en/Guide/Libraries

## Usage 
With your board connected, click ```upload``` in the Arduino IDE to flash the sketch to the board.

```mordecai.ino``` is the main Arduino sketch file. When the code has been compiled and loaded onto the board, it monitors
the serial line for incoming bytes (pilot commands), reads sensor input from an inertial measurement unit (IMU), does an AHRS state estimation 
on the sensor data, and calculates the required motor command inputs to follow the pilot's desired commands with a PID controller. The proportional, 
integral, and derivative gains are part of the incoming message data frame and can be tuned in real time from the ground station.

## Hardware
The basic equiment needed to drop this code into a quadcopter are:
  1. A way to communicate send data to the Arduino serial line wirelessly. This project used an xbee radio and xbee shield for 
     Arduino like the one here: https://www.sparkfun.com/products/12847
  2. A groundstation transmitter capable of sending data on a TX xbee. This project used a Macbook Air with a simple GUI written in Java
     that sends data over a serial out line.
  3. A quadrotor! With motors, speed controllers, and power installed. The four motor commands come from the Arduino pins defined as 
     ```motorPin1``` through ```motorPin4``` in ```Mordecai.ino```. 
  4. An inertial measurement unit (IMU). This project used the MPU9250. Its pin connections are shown at the top of the sketch.
  
  ## Improvements
This project is a work in progress. There is infinite room for improvement. For one, Arduino is not the most powerful platform to run 
quadrotor control on. In the future, this code should be modified to work on other controllers like Raspberry Pi, etc. 

- GPS sensor connection and integration into the navigation scheme.
- More precise quadrotor dynamic model.
- Making the system more robust to variation in physical parameters of the vehicle: mass, inertial properties, motors specs, and arm lengths.
- Higher level navigation schemes, for example calculation of minimum snap trajectories for waypoint missions.

  
