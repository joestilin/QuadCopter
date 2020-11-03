
#include <stdint.h>
#include <arduino.h>
#include "RCstream.h"

#define DEBUG		1
#define GRAVITY 		9.81 // m/s^2
#define L			0.25	// Quadcopter frame length
#define K			0.003	// constant: ratio rotor force to ang. velocity squared
#define B			0.0001 // constant
#define M			0.18 // mass constant kg
#define IXX			0.005 // inertia constants
#define IYY			0.005
#define IZZ			0.008


class Control {

public:

	bool inFlight = false;
	bool yawing = false;
	bool holdAltitude = false;

	float takeoffAltitude = 0.0f;

	// moving average filter
	/*
	static const int movingAverageLength = 5;
	float rollMovingAverageSamples[movingAverageLength];
	float pitchMovingAverageSamples[movingAverageLength];
	float yawMovingAverageSamples[movingAverageLength];
	float rolldotMovingAverageSamples[movingAverageLength];
	float pitchdotMovingAverageSamples[movingAverageLength];
	float yawdotMovingAverageSamples[movingAverageLength];

	float MAroll = 0.0f;
	float MApitch = 0.0f;
	float MAyaw = 0.0f;
	float MArolldot = 0.0f;
	float MApitchdot = 0.0f;
	float MAyawdot = 0.0f;
	*/

	// Low pass filter
	/*
	int lowpasslength = 1;
	int lowpasscount = 0;
	float rollLPsum = 0.0f;
	float pitchLPsum = 0.0f;
	float yawLPsum = 0.0f;
	float rolldotLPsum = 0.0f;
	float pitchdotLPsum = 0.0f;
	float yawdotLPsum = 0.0f;

	float LProll = 0.0f;
	float LPpitch = 0.0f;
	float LPyaw = 0.0f;
	float LProlldot = 0.0f;
	float LPpitchdot = 0.0f;
	float LPyawdot = 0.0f;
	*/

	// controller gains
	int Kph = 0, KpRoll = 0, KpPitch = 0, KpYaw = 0;
	int Kdh = 0, KdRoll = 0, KdPitch = 0, KdYaw = 0;
	int Kih = 0, KiRoll = 0, KiPitch = 0, KiYaw = 0;
	int Kddx = 0.0, Kddy = 0, Kddz = 0;

	// integral gain filter
	float roll_integral = 0.0f;
	float pitch_integral = 0.0f;
	float yaw_integral = 0.0f;

	// integral windup thresholds
	float rollwindupthreshold = 0.0f;
	float pitchwindupthreshold = 0.0f;
	float yawwindupthreshold = 0.0f;

	// desired climb rate, roll, pitch, yaw commands
	float hd = 0.0f, hdotd = 0.0f;
	float rolld = 0.0f, rolldotd = 0.0f;
	float pitchd = 0.0f, pitchdotd = 0.0f;
	float yawd = 0.0f, yawdotd = 0.0f;

	// desired translational
	float ax_d = 0.0f, ay_d = 0.0f, az_d = 0.0f;

	// translational acceleration error
	float ax_e = 0.0f, ay_e = 0.0f, az_e = 0.0f;

	// maximum control values
	float maxClimbRate = 0.25f;		// meters/s
	float maxPitch = 20.0f*DEG_TO_RAD;
	float maxRoll = 20.0f*DEG_TO_RAD;
	float maxYawRate = 45.0f*DEG_TO_RAD;		// deg/s
	float maxPitchRate = 30.0f*DEG_TO_RAD;
	float maxRollRate = 30.0f*DEG_TO_RAD;

	// loop timing
	float deltat = 0.0f;
	uint32_t now = 0;
	uint32_t lastUpdate = 0;

	// controller total thrust command
	float T = 0.0f;

	// torque commmands
	float tauroll = 0.0f, taupitch = 0.0f, tauyaw = 0.0f;

	// rotor angular velocity commands
	float omega1 = 0.0f, omega2 = 0.0f, omega3 = 0.0f, omega4 = 0.0f;


public:
	void altitudeFilter(float bmpaltitude);
	// void movingAverageFilter(float roll, float pitch, float yaw, float rolldot, float pitchdot, float yawdot);
	// void lowPassFilter(float roll, float pitch, float yaw, float rolldot, float pitchdot, float yawdot);
	void updateTime();
	void stabilize(float roll, float pitch, float yaw, float rolldot, float pitchdot, float yawdot, float ax_i, float ay_i, float az_i);
	void integralFilter(float roll, float pitch, float yaw);
	void readRC(RCstream rcin);


};
