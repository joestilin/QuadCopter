# QuadCopter
Quadcopter control on Arduino

This code runs on the popular Arduino microcontroller series. While the ```mordecai.ino``` sketch is written for the Arduino Uno
board, the code can easily be expanded to work with boards having more pins and features.

The easiest way to get up and running is by compiling the code and loading onto your Arduino board with Arduino's IDE.

## Installation
1. Download and install the Arduino IDE for your system from https://www.arduino.cc/en/main/software

2. Clone the repository into your local workspace

```bash
$ git clone https://www.github.com/joestilin/QuadCopter

```

3. Set the working directory in the IDE to include the src, inc, and Mordecai folders.

## Usage 

```mordecai.ino``` is the main Arduino sketch file. When the code has been compiled and loaded onto the board, it monitors
the serial line for incoming bytes (pilot commands), reads the input from an inertial measurement unit (IMU), and calculates the required
motor command imputs to hold a quadcopter at stable hover.
