OBJECTS = CalLib.o inv_mpu.o inv_mpu_dmp_motion_driver.o MPU9150Lib.o MPUQuaternion.o MPUVector3.o
CFLAGS = -Wall -std=gnu++0x -pthread -I.. -O

all: mpu9150.a

mpu9150.a: $(OBJECTS)
	ar r mpu9150.a $(OBJECTS)
	ar s mpu9150.a

%.o: %.cpp
	g++ -c $(CFLAGS) $< -o $@

clean:
	rm *.o mpu9150.a

depend:
	g++ -MM $(CFLAGS) *.cpp > .depend

include .depend

