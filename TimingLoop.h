#include <stdint.h>
#include <Arduino.h>

class TimingLoop {

public:
	uint32_t length;			// loop length in milliseconds
	uint32_t count;			// time of last loop start in milliseconds
	uint32_t deltat = 0; 	// current loop time

public:
	void setLength(uint32_t);
	void start();
	bool isDone();
};
