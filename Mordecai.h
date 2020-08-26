
#include "quaternionFilters.h"
#include "MPU9250.h"

#define ARMSEQUENCE 		"&RM"
#define DEBUG		0
#define ARMTIMEMAX 	60	// seconds

// When sketch is started, wait for ARMSEQUENCE
void arm() {

	int armed = 0;
	long start_time = millis();

	while (!armed) {
		if (Serial.find(ARMSEQUENCE)) {
			armed = 1;
			Serial.println(""); Serial.println("Mordecai armed!");
		}
		// timeout if not armed after ARMTIMEMAX seconds
		if ((millis() - start_time) >= ARMTIMEMAX*1000) {
			Serial.print("Failed to arm Mordecai after ");
			Serial.print(ARMTIMEMAX);  Serial.println(" seconds");
			while(1);
		}
	}
}











