// RCstream.cpp

#include "RCstream.h"
#include <HardwareSerial.h>
#include <Arduino.h>

bool RCstream::getRCframe() {

	/*
	// find the first match of STARTRCFRAME in serialBuffer
	bool matchChar = false;
	bool match = false;
	int index = 0;

	// iterate by i through serialBuffer
	for (int i = 0; i < (numChars - STARTRCFRAME.length() - NCHANNELS - 1); i++) {
		matchChar = false;
		// iterate by j through STARTRCFRAME
		for (int j = 0; j < STARTRCFRAME.length(); j++) {
			// check for a char match
			Serial.print("i: ");
			Serial.print(i);
			Serial.print(" ");
			Serial.print("j: ");
			Serial.print(j);
			Serial.print(" ");
			Serial.print("buf: ");
			Serial.print(serialBuffer[i + j]);
			Serial.print(" ");
			Serial.print("fra: ");
			Serial.println(STARTRCFRAME.charAt(j));
			if ((char)serialBuffer[i + j] == STARTRCFRAME.charAt(j)) {
				matchChar = true;
				Serial.println("matchchar");
				// if no match break from iterating through STARTRCFRAME
			} else {
				matchChar = false;
				break;
			}
		}
		// if matchChar still true
		if (matchChar) {
			Serial.println("match start");
			match = true;
			index = i;
			break;
		}
	}

	if (match) {
		for (int k = 0; k < NCHANNELS; k++) {
			rcDataFrame[k] = serialBuffer[index + STARTRCFRAME.length() + k];
		}
	}

	// reset serialBuffer to zeros
	for (int i = 0; i < numChars; i++) {
		serialBuffer[i] = 0;
	}

	if (match) {
		Serial.println("true");
		return true;
	}
	else {
		return false;
	}
	 */
	if (serialBuffer[0] == (char)startbyte) {
		for (int i = 0; i < NCHANNELS; i++) {
			rcDataFrame[i] = (int)serialBuffer[i + 1];
		}
		return true;
	}
	else {
		return false;
	}
}

void RCstream::mapRC() {
	// convert serial ints in [-127, 128] to float in [-1.0, 1.0]
	throttle = ((float)rcDataFrame[0]) / 127.0f;
	yawRate = ((float)rcDataFrame[1]) / 127.0f;
	pitch = ((float)rcDataFrame[2]) / 127.0f;
	roll = ((float)rcDataFrame[3]) / 127.0f;
	emergencyoff = rcDataFrame[4];
	kph = rcDataFrame[5];
	kdh = rcDataFrame[6];
	kih = rcDataFrame[7];
	kpyaw = rcDataFrame[8];
	kdyaw = rcDataFrame[9];
	kiyaw = rcDataFrame[10];
	kproll = rcDataFrame[11];
	kdroll = rcDataFrame[12];
	kiroll = rcDataFrame[13];
	kppitch = rcDataFrame[14];
	kdpitch = rcDataFrame[15];
	kipitch = rcDataFrame[16];
	motortrim1 = rcDataFrame[17];
	motortrim2 = rcDataFrame[18];
	motortrim3 = rcDataFrame[19];
	motortrim4 = rcDataFrame[20];
	filterthreshold = rcDataFrame[21];

}

bool RCstream::motorsStartCommand() {
	if (throttle < -0.5 && yawRate > 0.5 && pitch < -0.5 && roll < -0.5) {
		return true;
	}
	else {return false;
	}
}

bool RCstream::throttlePositive() {
	if (throttle > 0.0) {
		return true;
	}
	else {
		return false;
	}
}

bool RCstream::throttleOff() {
	if (throttle == 0.0) {
		return true;
	}
	else {
		return false;
	}
}

bool RCstream::yawRateZero() {
	if (yawRate == 0.0) {
		return true;
	}
	else {
		return false;
	}
}

bool RCstream::landingCommand() {
	if (throttle < -0.98) {
		return true;
	}
	else {
		return false;
	}
}

bool RCstream::emergencyOffCommand() {
	if (emergencyoff == 1) {
		return true;
	}
	else {
		return false;
	}

}




