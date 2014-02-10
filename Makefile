PROGRAMS := raspisb
OBJECTS = raspisb.o config.o ms5611.o mcp3201.o 24lc16.o serial.o audio_vario.o settings.o connect2xcsoar.o \
          KalmanFilter1d.o Fixed.o ToneSynthesiser.o VarioSynthesiser.o VarioSettings.o AirDensity.o \
          bcm2835.o
#CFLAGS = -DFIXED_MATH -DALSA_ASYNC=0 -Wall -std=gnu++0x -pthread -Impu9150 -O
CFLAGS = -DALSA_ASYNC=0 -Wall -std=gnu++0x -pthread -Impu9150 -O
LIBS = mpu9150/mpu9150.a -lasound

all: $(PROGRAMS)

raspisb: $(OBJECTS) $(LIBS)
	g++ $(CFLAGS) $(OBJECTS) -o $@ $(LIBS)

%.o: %.cpp
	g++ -c $(CFLAGS) $< -o $@

clean:
	rm *.o raspisb *~

depend:
	g++ -MM $(CFLAGS) *.cpp > .depend

include .depend

