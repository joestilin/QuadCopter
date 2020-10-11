/* RCstream.h
 *
 */
#include <stdint.h>
#include <Arduino.h>

//#define STARTRCFRAME 	"START"
#define ENDRCFRAME 		"END"

#define NCHANNELS		22			// number of channels in your RC protocol
#define DEBUG			1


class RCstream {

public:

	// RC timing
	float frequency = 0.0f;
	uint16_t count = 0;
	uint16_t deltat = 0;

	// Channels
	float throttle = 0.0f;
	float yawRate = 0.0f;
	float roll = 0.0f;
	float pitch = 0.0f;
	int emergencyoff = 0;
	int kph = 0;
	int kdh = 0;
	int kih = 0;
	int kpyaw = 0;
	int kdyaw = 0;
	int kiyaw = 0;
	int kproll = 0;
	int kdroll = 0;
	int kiroll = 0;
	int kppitch = 0;
	int kdpitch = 0;
	int kipitch = 0;
	int filterthreshold = 0;
	int motortrim1 = 0;
	int motortrim2 = 0;
	int motortrim3 = 0;
	int motortrim4 = 0;

	static const int numChars = 63;
	int indexFrame = 0;
	int newBytesRead = 0;

	byte startbyte = -128;
	char serialBuffer[numChars];
	int rcDataFrame[NCHANNELS];

	bool charMatch = false;


public:
	bool getRCframe();
	void mapRC();
	bool motorsStartCommand();
	bool throttleOff();
	bool throttlePositive();
	bool yawRateZero();
	bool landingCommand();
	bool emergencyOffCommand();

};

