
#pragma once

#include <Utils.hpp>


class Timeout {
public:
	Timeout(float seconds = 0) {
		setIntervalInSeconds(seconds);
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
