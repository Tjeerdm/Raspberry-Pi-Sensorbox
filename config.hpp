#include "Fixed.hpp"

extern struct config_t {
	unsigned baudrate;
	int ecomp;
	bool noahrs;
	fixed VarioAccel;
	struct {
		unsigned volume;
		bool speed2fly;
		fixed deadbandlow;
		fixed deadbandhigh;
		unsigned freqlow;
		unsigned freqmid;
		unsigned freqhigh;
	} audio;
} config;

void read_config();

