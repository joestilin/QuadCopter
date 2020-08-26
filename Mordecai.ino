/* Main Mordecai Flight Controller Sketch for Arduino Uno
   By: Joe Stilin
   October 2017
 */

#include "Mordecai.h"
//#include "RCstream.h"
#include "Control.h"
#include "Motors.h"
#include "quaternionFilters.h"
#include "MPU9250.h"
//#include <Adafruit_BMP085.h>


// Arduino Pin Connections for reference
// MPU9250 Breakout --------- Arduino
// VDD ---------------------- 3.3V
// VDDI --------------------- 3.3V
// SDA ----------------------- A4
// SCL ----------------------- A5
// GND ---------------------- GND


// motor pins
int motorPin1 = 5;
int motorPin2 = 10;
int motorPin3 = 9;
int motorPin4 = 3;

int HIGHPIN = 8;

// interrupt pin
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed  = 13;  // Set up pin 13 led for toggling

bool initialized = false;

int startbyte = 255;

MPU9250 IMU;
//Adafruit_BMP085 bmp;
RCstream rcin;
Control controller;
Motors motors;

void setup()
{
	// set motor pins
	pinMode(motorPin1, OUTPUT);
	pinMode(motorPin2, OUTPUT);
	pinMode(motorPin3, OUTPUT);
	pinMode(motorPin4, OUTPUT);
	pinMode(HIGHPIN, OUTPUT);
	motors.motorsAttached = true;

	Wire.begin();
	Serial.begin(115200);
	Serial.flush();

	// wait for arming command from radio
	arm();

	/*
	// start pressure sensor
	if (!bmp.begin()) {
		Serial.println("***Pressure Sensor Not Online***");
		//while (1) {}
	}
	*/
	// get pressure sensor noise bias


	// Set up the interrupt pin, its set as active high, push-pull
	pinMode(intPin, INPUT);
	digitalWrite(intPin, LOW);
	pinMode(myLed, OUTPUT);
	digitalWrite(myLed, HIGH);

	digitalWrite(HIGHPIN, HIGH);

	// start IMU
	byte c = IMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
	if (c == 0x71) // WHO_AM_I should always be 0x68
	{
		Serial.println("MPU9250 is online...");

		// Start by performing self test and reporting values
		IMU.MPU9250SelfTest(IMU.SelfTest);
		Serial.print("x-axis self test: acceleration trim within : ");
		Serial.print(IMU.SelfTest[0],1); Serial.println("% of factory value");
		Serial.print("y-axis self test: acceleration trim within : ");
		Serial.print(IMU.SelfTest[1],1); Serial.println("% of factory value");
		Serial.print("z-axis self test: acceleration trim within : ");
		Serial.print(IMU.SelfTest[2],1); Serial.println("% of factory value");
		Serial.print("x-axis self test: gyration trim within : ");
		Serial.print(IMU.SelfTest[3],1); Serial.println("% of factory value");
		Serial.print("y-axis self test: gyration trim within : ");
		Serial.print(IMU.SelfTest[4],1); Serial.println("% of factory value");
		Serial.print("z-axis self test: gyration trim within : ");
		Serial.print(IMU.SelfTest[5],1); Serial.println("% of factory value");

		// Calibrate gyro and accelerometers, load biases in bias registers
		IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);

		IMU.initMPU9250();
		// Initialize device for active mode read of acclerometer, gyroscope, and
		// temperature
		Serial.println("MPU9250 initialized for active data mode....");

		// Get magnetometer calibration from AK8963 ROM
		IMU.initAK8963(IMU.magCalibration);
		// Initialize device for active mode read of magnetometer
		Serial.println("AK8963 initialized for active data mode....");
		if (DEBUG)
		{
			//  Serial.println("Calibration values: ");
			Serial.print("X-Axis sensitivity adjustment value ");
			Serial.println(IMU.magCalibration[0], 2);
			Serial.print("Y-Axis sensitivity adjustment value ");
			Serial.println(IMU.magCalibration[1], 2);
			Serial.print("Z-Axis sensitivity adjustment value ");
			Serial.println(IMU.magCalibration[2], 2);
		}
	} // if (c == 0x71)
	else
	{
		Serial.print("Could not connect to MPU9250: 0x");
		Serial.println(c, HEX);
		while(1) ; // Loop forever if communication doesn't happen
	}

}

void loop() {

	/****** MEASUREMENT BLOCK ********/

	// get an INS sample (fast loop)
	IMU.updateINS();

	// perform AHRS (fast loop) including inertial frame z acceleration
	IMU.AHRS();

	// feed angles and rates through moving average filter
	//controller.movingAverageFilter(IMU.roll, IMU.pitch, IMU.yaw, IMU.rolldot, IMU.pitchdot, IMU.yawdot);

	//controller.lowPassFilter(IMU.roll, IMU.pitch, IMU.yaw, IMU.rolldot, IMU.pitchdot, IMU.yawdot);

	// resolve altitude and inertial climb rate with pressure sensor altitude and z_i acceleration
	//IMU.InertialAltitude(bmp.altitude);

	/****** END MEASUREMENT BLOCK *****/

	/****** BEGIN NAVIGATION BLOCK ****/
	// combine pilot RC commands + current flight phase into desired vehicle thrust, yaw, pitch, roll commands
	// Flight phases:
	// Resting
	// Motors On
	// Takeoff
	// In Flight
	// Landing
	// Motors off

	// notify controller of pilot commands and gain values
	controller.readRC(rcin);

	// Motors on:
	if (!controller.inFlight && !motors.spinUpMotors) {
		// set desired orientation to current yaw
		controller.yawd = IMU.yaw;
		// motors spinup command: sticks in and down
		if (rcin.motorsStartCommand()) {
			motors.spinUpMotors = true;
			// reset emergency off to false if it was triggered on
			motors.emergencyoff = false;
			// set desired yaw to current yaw
			controller.yawd = IMU.yaw;
		}
	}

	// reving up motors to takeoff
	if (!controller.inFlight && motors.spinUpMotors) {
		// pilot throttled back down: turn motors off
		// pilot throttled up to takeoff thrust
		if (motors.motorrampfactor == 1.0) {
			// switch to inFlight mode
			controller.inFlight = true;
			// take motors out of spinup mode
			motors.spinUpMotors = false;
			// set initial yaw to current yaw orientation
			controller.yawd = IMU.yaw;
			//controller.takeoffAltitude = bmp.filteredAltitude;
		}
	}

	// in flight:
	// throttle, yaw rate, pitch, roll in [-max, max] scaled by RC commands
	if (controller.inFlight) {

		// handle yawing:
		// while yawing, yawd = yaw
		// when not yawing, yawd = yaw at end of last yaw command

		// currently yawing
		if (controller.yawing) {
			controller.yawd = IMU.yaw;		// yaw error effectively zero while yawing
			// stopped yawing
			if (rcin.yawRateZero()) {
				controller.yawing = false;
				controller.yawd = IMU.yaw;	// save yaw value at end of yawing
			}
		}

		// not currently yawing
		if (!controller.yawing) {
			// started yawing
			if (!rcin.yawRateZero()) {
				controller.yawing = true;
			}
		}

		// Landing
		// If in flight, throttle all the way down and altitude not changing (we're on the ground)
		if (rcin.landingCommand() && IMU.az_i >= 9.75) {
			controller.inFlight = false;
			// motors back down to spinup mode
			motors.spinUpMotors = true;
		}
	}

	// Emergency off
	if (rcin.emergencyOffCommand()) {
		// turn off all flight phases
		motors.emergencyoff = true;
		motors.spinUpMotors = false;
		controller.inFlight = false;
	}

	/***** END NAVIGATION BLOCK *******/


	/***** BEGIN CONTROL BLOCK ********/

	controller.stabilize(IMU.roll, IMU.pitch, IMU.yaw, IMU.rolldot, IMU.pitchdot, IMU.yawdot, IMU.ax_i, IMU.ay_i, IMU.az_i);

	/***** END CONTROL BLOCK **********/

	/***** BEGIN MOTOR CONTROL BLOCK **/

	// on ground: scale all motor commands from min_motor_scale to thrust command;  x 1 when in air
	if (motors.spinUpMotors) {
		motors.onGroundMotorsOn(rcin.throttle);
	}

	motors.convertToPwm(controller.omega1, controller.omega2, controller.omega3, controller.omega4);


	// Emergency Shutoff set PWM values to 0
	if (motors.emergencyoff) {
		motors.motorsOff();
	}

	// write final PWM values to motors
	if (motors.motorsAttached) {
		analogWrite(motorPin1, motors.pwm1);
		analogWrite(motorPin2, motors.pwm2);
		analogWrite(motorPin3, motors.pwm3);
		analogWrite(motorPin4, motors.pwm4);
	}

	/***** END MOTOR CONTROL BLOCK ****/

	/***** BEGIN LOGGING BLOCK ********/

	// Serial print and/or display at 0.5 s rate independent of data rates
	IMU.delt_t = millis() - IMU.count;

	if (IMU.delt_t > 50)
	{

		// Data dump to ground station
		Serial.write(startbyte);
		Serial.write((int)(IMU.roll*RAD_TO_DEG + 127));
		Serial.write((int)(IMU.pitch*RAD_TO_DEG + 127));
		Serial.write((int)(IMU.yaw*RAD_TO_DEG + 127));
		Serial.write((int)(IMU.ax_i + 127));
		Serial.write((int)(IMU.ay_i + 127));
		Serial.write((int)(IMU.az_i + 127));
		Serial.write(motors.pwm1);
		Serial.write(motors.pwm2);
		Serial.write(motors.pwm3);
		Serial.write(motors.pwm4);
		Serial.write((int)(IMU.sumCount/IMU.sum));


		/*
		Serial.print("throttle: ");
		Serial.println(rcin.throttle);
		Serial.print("yawRate: ");
		Serial.println(rcin.yawRate);
		Serial.print("pitch: ");
		Serial.println(rcin.pitch);
		Serial.print("roll: ");
		Serial.println(rcin.roll);


		Serial.print("Yaw: ");
		Serial.print(IMU.yaw, 2);
		Serial.print(" Pitch : ");
		Serial.print(IMU.pitch, 2);
		Serial.print(" Roll: ");
		Serial.print(IMU.roll, 2);
		Serial.print(" YawDot: ");
		Serial.print(IMU.yawdot, 2);
		Serial.print(" PitchDot : ");
		Serial.print(IMU.pitchdot, 2);
		Serial.print(" RollDot: ");
		Serial.print(IMU.rolldot, 2);

		/*
		Serial.print("Inflight: ");
		Serial.print(controller.inFlight);
		Serial.print(" SpinUp: ");
		Serial.print(motors.spinUpMotors);
		Serial.print(" Yawing: ");
		Serial.print(controller.yawing);

		Serial.print(" hdotd ");
		Serial.print(controller.hdotd);
		Serial.print(" yawd ");
		Serial.print(controller.yawd);
		Serial.print(" yawdotd ");
		Serial.print(controller.yawdotd);
		Serial.print(" pitchd ");
		Serial.print(controller.pitchd);
		Serial.print(" rolld ");
		Serial.print(controller.rolld);
		Serial.print(" az_i: ");
		Serial.println(IMU.az_i);

		Serial.print("Thrust: ");
		Serial.print(controller.T, 2);
		Serial.print(" tauroll= ");
		Serial.print(controller.tauroll, 2);
		Serial.print(" taupitch= ");
		Serial.print(controller.taupitch, 2);
		Serial.print(" tauyaw= ");
		Serial.print(controller.tauyaw, 2);

		Serial.print(" PWM1= ");
		Serial.print(motors.pwm1);
		Serial.print(" PWM2= ");
		Serial.print(motors.pwm2);
		Serial.print(" PWM3= ");
		Serial.print(motors.pwm3);
		Serial.print(" PWM4= ");
		Serial.println(motors.pwm4);



		/*
		Serial.print("rate = ");
		Serial.print((float)IMU.sumCount/IMU.sum, 2);
		Serial.println(" Hz");

		Serial.print("RC freq: ");
		Serial.println(rcin.frequency, 2);
		 */


		IMU.count = millis();
		IMU.sumCount = 0;
		IMU.sum = 0;
	}



	/* convert RC values to scaled thrust, roll, pitch, yaw commands
	 * commands come in as 0-255.
	 * 128 = "center stick"
	 * Thrust 128 = hold altitude
	 * Thrust 0 - 127 = descend, 0 = max rate descend
	 * Thrust 129 - 255 = ascend, 255 = max rate ascend
	 *
	 * Yaw 128 = hold yaw angle
	 * Yaw 0 - 127 = yaw negative (cw looking from above), 0 = max yaw negative
	 * Yaw 129 - 255 yaw positive (ccw) 255 = max yaw positive
	 *
	 * roll & pitch 128 hold roll and pitch angles
	 * roll & pitch 0 - 127 roll/pitch negative 0 = max negative angle
	 * roll & pitch 129 - 255 max positive angle
	 */

	// Convert these ints to floats for actual thrust, yaw, roll, pitch
	// they feed into the controller control.. so a class controller that has these as
	// desired trajectory, and takes rates and angles from MPU9250 class and
	// quaternion calculations


	// Read in raw sensor values
	// Convert to physical values

	// Attitude control:
	// rotate to inertial frame for roll, pitch, yaw rates
	// quaternion update for roll, pitch, yaw
	// derivative error: difference of rates
	// proportional error: difference in angles
	// integral error: sum of all past error (handle windup)
	// feed to controller controller
	// send to motors


}

void serialEvent() {
	// RC data has arrived. Load Arduino serial buffer into our own buffer
	if (Serial.available() >= (NCHANNELS + 1)) {
		rcin.newBytesRead = Serial.readBytes(rcin.serialBuffer, Serial.available());
		if (rcin.getRCframe()){
			rcin.mapRC();
			rcin.deltat = millis() - rcin.count;
			rcin.frequency = 1000 / rcin.deltat;
			rcin.count = millis();
		}
	}
}



