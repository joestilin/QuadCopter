
#include <arduino.h>


# define MOTOR_SCALE	0.8   // ratio PWM [0 255] to angular velocity rotor

class Motors {

public:

	bool spinUpMotors = false;
	bool motorsAttached = false;
	bool emergencyoff = false;

	// in [0, 1] scales output control to motors
	double motorrampfactor = 0.0;
	// motors on start value of ramp factor
	double motorstartfactor = 0.2;

	uint8_t pwm1 = 0;
	uint8_t pwm2 = 0;
	uint8_t pwm3 = 0;
	uint8_t pwm4 = 0;

public:

void motorsOff();
void onGroundMotorsOn(float throttle);
void convertToPwm(double omega1, double omega2, double omega3, double omega4);

};
