
#include "Motors.h"
#include <Arduino.h>

void Motors::motorsOff() {
	pwm1 = 0;
	pwm2 = 0;
	pwm3 = 0;
	pwm4 = 0;
	motorrampfactor = 0;
}
void Motors::onGroundMotorsOn(float throttle) {
	motorrampfactor = constrain(motorstartfactor + 2*throttle*(1.0 - motorstartfactor) , 0.0, 1.0);
}

void Motors::convertToPwm(double omega1, double omega2, double omega3, double omega4) {
	// failsafe: pwm values must be integers in [0, 255]
	pwm1 = constrain((uint8_t)omega1*MOTOR_SCALE*motorrampfactor, 0, 255);
	pwm2 = constrain((uint8_t)omega2*MOTOR_SCALE*motorrampfactor, 0, 255);
	pwm3 = constrain((uint8_t)omega3*MOTOR_SCALE*motorrampfactor, 0, 255);
	pwm4 = constrain((uint8_t)omega4*MOTOR_SCALE*motorrampfactor, 0, 255);
}
