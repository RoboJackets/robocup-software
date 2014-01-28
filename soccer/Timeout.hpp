
#pragma once

#include <Utils.hpp>

/**
 * This is a simple timeout timer.
 *
 * Initialize it with a given duration,
 * then query it repeatedly to see if the duration is over.
 */
class Timeout {
public:
	Timeout(float seconds = 0) {
		setIntervalInSeconds(seconds);
		reset();
	}

	void reset() {
		_startTime = timestamp();
	}


	void setIntervalInSeconds(float seconds) {
		_interval = (uint64_t)(seconds * 1000.0f);
	}

	void setIntervalInMilliseconds(uint64_t ms) {
		_interval = ms;
	}


	bool isTimedOut() {
		return timestamp() - _startTime > _interval;
	}

private:
	uint64_t _interval;
	uint64_t _startTime;
};
