// Determines if the emitter to reciever beam is broken
//  while accounting for ambiant light
#include "mbed.h"

class ball_sense {
public:
	ball_sense();

	// Call once per cycle
	void update_ball_sensor();
	
	// Returns true if beam is broken (Ball is contained)
	bool have_ball();
private:
	DigitalOut *emitter_pin;
	AnalogIn   *detector_pin;

	// Holds the light sensed when
	//  emitter is lit (sense_light) and
	//  dark (sense_dark)
	int sense_light = 0, sense_dark = 0;
	
	// Emitter state (Light or dark)
	bool emitter_on = false;

	// Difference between the light and dark
	//  values before the sensor is considered
	//  broken
	const int sense_threshold = 8000;

	// Number of consecative "broken" senses
	//  before we are confident the beam is broken
	const unsigned int consec_num = 2;
	
	// Consecative "broken" senses counter
	unsigned int consec_ctr;
};
