#include "Fixed.hpp"

extern int flags;
// flags
// Protocol
#define VW1150   1 // default
#define EYE	 2 // readable output / debug

// Hardware
#define	MCP3201	0x100
#define	MCPxxxx	0x200

// Connections
#define HAVE_AIRDATA	0x400
#define HAVE_MPU9150	0x800
#define HAVE_SERIAL	0x1000

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

