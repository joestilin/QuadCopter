#include "TimingLoop.h"

void TimingLoop::setLength(uint32_t l) {
	length = l;
}
void TimingLoop::start() {
	deltat = 0;
	count = millis();
}

bool TimingLoop::isDone() {
	deltat = millis() - count;
	if (deltat > length) {
		return true;
	}
	else{
		return false;
	}
}
