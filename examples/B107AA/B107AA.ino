// Define Module
#define B107AA

// Include Library
#include <Arduino.h>
#include "AVR_Functions.h"

// Define Hardware Class
AVR_Functions Hardware;

// Define Variables
int x = 0;

// Setup
void setup() {

	// Define Pin Out
	Hardware.PinOut();

	// Define Timer
	Hardware.AVR_Timer();

  // Define Serial
  Serial.begin(115200);

}

void loop() {


	

}

// Timer Interrupt
ISR(TIMER5_COMPA_vect) {

	// Watchdog HeartBeat
	Hardware.Heartbeat();

	Serial.println(x);
	x++;

}