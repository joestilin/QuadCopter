
#include "Control.h"
#include <HardwareSerial.h>
#include <Arduino.h>


void Control::readRC(RCstream rcin) {
	hdotd = rcin.throttle*maxClimbRate;
	yawdotd = rcin.yawRate*maxYawRate;
	pitchd = rcin.pitch*maxPitch;
	rolld = rcin.roll*maxRoll;
	Kph = (float)rcin.kph;
	Kdh = (float)rcin.kdh;
	Kih = (float)rcin.kih;
	KpRoll = (float)rcin.kproll;
	KdRoll = (float)rcin.kdroll;
	KiRoll = (float)rcin.kiroll;
	KpPitch = (float)rcin.kppitch;
	KdPitch = (float)rcin.kdpitch;
	KiPitch = (float)rcin.kipitch;
	KpYaw = (float)rcin.kpyaw;
	KdYaw = (float)rcin.kdyaw;
	KiYaw = (float)rcin.kiyaw;
	rollwindupthreshold = ((float)rcin.filterthreshold / 100);
	pitchwindupthreshold = ((float)rcin.filterthreshold / 100);
}



void Control::stabilize(float roll, float pitch, float yaw, float rolldot, float pitchdot, float yawdot, float ax_i, float ay_i, float az_i) {


	// collect integral term
	// update integral deltat
	updateTime();
	integralFilter(roll, pitch, yaw);

	// translational control
	az_d = hdotd;	// bogus assignment of vertical acceleration to desired climb rate

	ax_e = (ax_d - ax_i);
	ay_e = (ay_d - ay_i);
	az_e = (az_d - az_i + GRAVITY);

	// multiply errors by translational gains
	Kddz = Kdh;
	ax_e *= Kddx;
	ay_e *= Kddy;
	az_e *= Kddz;

	//rolld += asin((ax_e*sin(yaw) - ay_e*cos(yaw)) / (ax_e*ax_e + ay_e*ay_e + pow(az_e +  GRAVITY, 2)));
	//pitchd += atan((ax_e*cos(yaw) + ay_e*sin(yaw)) / (az_e + GRAVITY));

	//T = M*(ax_e*(sin(pitch)*cos(yaw)*cos(roll) + sin(yaw)*sin(roll)) + ay_e*(sin(pitch)*sin(yaw)*cos(roll) - cos(yaw)*sin(roll)) + (az_e + GRAVITY)*cos(pitch)*cos(roll));

	T = (GRAVITY + Kdh*(hdotd))*M / (cos(roll)*cos(pitch));
	tauroll = (KdRoll*(rolldotd - rolldot) + KpRoll*(rolld - roll) + KiRoll*roll_integral)*IXX;
	taupitch = (KdPitch*(pitchdotd - pitchdot) + KpPitch*(pitchd - pitch) + KiPitch*pitch_integral)*IYY;
	tauyaw = (KdYaw*(yawdotd - yawdot) + KpYaw*(yawd - yaw) + KiYaw*yaw_integral)*IZZ;


	// control input to angular velocities of rotors
	omega1 = 0.25*T/K - 0.25*taupitch/(K*L) - 0.25*tauroll/(K*L) - 0.25*tauyaw/B;
	omega2 = 0.25*T/K + 0.25*taupitch/(K*L) - 0.25*tauroll/(K*L) + 0.25*tauyaw/B;
	omega3 = 0.25*T/K + 0.25*taupitch/(K*L) + 0.25*tauroll/(K*L) - 0.25*tauyaw/B;
	omega4 = 0.25*T/K - 0.25*taupitch/(K*L) + 0.25*tauroll/(K*L) + 0.25*tauyaw/B;

}

void Control::integralFilter(float roll, float pitch, float yaw) {

	// prevent integral windup
	if (abs(roll_integral) < rollwindupthreshold) {
		roll_integral += (rolld - roll)*deltat;
	}
	else {
		roll_integral = 0.0;
	}

	if (abs(pitch_integral) < pitchwindupthreshold) {
		pitch_integral += (pitchd - pitch)*deltat;
	}
	else {
		pitch_integral = 0.0;
	}

	if (abs(yaw_integral) < yawwindupthreshold) {
		yaw_integral += (yawd - yaw)*deltat;
	}
	else {
		yaw_integral = 0.0;
	}

}

/*
void Control::movingAverageFilter(float roll, float pitch, float yaw, float rolldot, float pitchdot, float yawdot) {
	float sum = 0.0f;
	// roll
	for (int i = 1; i < movingAverageLength; i++) {
		sum += rollMovingAverageSamples[i];
		rollMovingAverageSamples[i-1] = rollMovingAverageSamples[i];
	}
	sum += roll;
	rollMovingAverageSamples[movingAverageLength - 1] = roll;
	MAroll = sum / movingAverageLength;
	sum = 0.0;

	// pitch
	for (int i = 1; i < movingAverageLength; i++) {
		sum += pitchMovingAverageSamples[i];
		pitchMovingAverageSamples[i-1] = pitchMovingAverageSamples[i];
	}
	sum += pitch;
	pitchMovingAverageSamples[movingAverageLength - 1] = pitch;
	MApitch = sum / movingAverageLength;
	sum = 0.0;

	// yaw
	for (int i = 1; i < movingAverageLength; i++) {
		sum += yawMovingAverageSamples[i];
		yawMovingAverageSamples[i-1] = yawMovingAverageSamples[i];
	}
	sum += yaw;
	yawMovingAverageSamples[movingAverageLength - 1] = yaw;
	MAyaw = sum / movingAverageLength;
	sum = 0.0;

	// rolldot
	for (int i = 1; i < movingAverageLength; i++) {
		sum += rolldotMovingAverageSamples[i];
		rolldotMovingAverageSamples[i-1] = rolldotMovingAverageSamples[i];
	}
	sum += rolldot;
	rolldotMovingAverageSamples[movingAverageLength - 1] = rolldot;
	MArolldot = sum / movingAverageLength;
	sum = 0.0;

	// pitchdot
	for (int i = 1; i < movingAverageLength; i++) {
		sum += pitchdotMovingAverageSamples[i];
		pitchdotMovingAverageSamples[i-1] = pitchdotMovingAverageSamples[i];
	}
	sum += pitchdot;
	pitchdotMovingAverageSamples[movingAverageLength - 1] = pitchdot;
	MApitchdot = sum / movingAverageLength;
	sum = 0.0;

	// yawdot
	for (int i = 1; i < movingAverageLength; i++) {
		sum += yawdotMovingAverageSamples[i];
		yawdotMovingAverageSamples[i-1] = yawdotMovingAverageSamples[i];
	}
	sum += yawdot;
	yawdotMovingAverageSamples[movingAverageLength - 1] = yawdot;
	MAyawdot = sum / movingAverageLength;
}
*/
/*
void Control::lowPassFilter(float roll, float pitch, float yaw, float rolldot, float pitchdot, float yawdot) {
	// simple average a sample of lowpasslength measurements
	if (lowpasscount < lowpasslength) {
		lowpasscount += 1;
		rollLPsum += roll;
		pitchLPsum += pitch;
		yawLPsum += yaw;
		rolldotLPsum += rolldot;
		pitchdotLPsum += pitchdot;
		yawdotLPsum += yawdot;
	}
	if (lowpasscount == lowpasslength) {
		LProll = rollLPsum / lowpasslength;
		LPpitch = pitchLPsum / lowpasslength;
		LPyaw = yawLPsum / lowpasslength;
		LProlldot = rolldotLPsum / lowpasslength;
		LPpitchdot = pitchdotLPsum / lowpasslength;
		LPyawdot = yawdotLPsum / lowpasslength;

		// reset to zero
		lowpasscount = 0;
		rollLPsum = 0;
		pitchLPsum = 0;
		yawLPsum = 0;
		rolldotLPsum = 0;
		pitchdotLPsum = 0;
		yawdotLPsum = 0;
	}
}
*/

void Control::updateTime()
{
	now = micros();

	deltat = ((now - lastUpdate) / 1000000.0f);
	lastUpdate = now;
}





