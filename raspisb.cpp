/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2013 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

#define PI_USART_SELECT RPI_V2_GPIO_P1_22

#include <thread>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <alsa/asoundlib.h>

#include "raspisb.hpp"
#include "bcm2835.hpp"
#include "Fixed.hpp"
#include "Clamp.hpp"
#include "config.hpp"
#include "ms5611.hpp"
#include "mcp3201.hpp"
#include "24lc16.hpp"
#include "settings.hpp"
#include "connect2xcsoar.hpp"
#include "KalmanFilter1d.hpp"
#include "NMEAChecksum.hpp"
#include "AirDensity.hpp"

#include "MPU9150Lib.h"
#include "CalLib.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#include "VarioSynthesiser.hpp"
#include "audio_vario.hpp"

bool circling = true;           // Both received from XCSoar
fixed speed2fly;                // in km/h

unsigned board_revision = 0;
int flags;

/* Pressure sensor related */
struct ms5611_t ms5611[3];
KalmanFilter1d vkf;             // vario kalman filter

/* Kalman filter for magnetic heading from mpu9150 */
KalmanFilter1d hkf;

/* MPU9150 related */
#define  MPU9150_DEVICE    1                        
MPU9150Lib MPU;                 // the MPU9150Lib object
CALLIB_DATA calData;            // the calibration data

#define PRESSURE_SAMPLE_RATE 25		// rate (in Hz) in which to sample air pressure
#define TEMP_SAMPLE_RATE  (PRESSURE_SAMPLE_RATE/5)  	// rate (in Hz) in which to sample oressure sensors temperature
#define AIRDATA_SEND_RATE     5		// rate (in Hz) in which to send air data
#define AIRDATA_SEND_RATE     5		// rate (in Hz) in which to send air data

#define MPU_UPDATE_RATE       5		// rate (in Hz) at which the MPU updates the sensor data and DMP output
#define MAG_UPDATE_RATE       5		// rate (in Hz) at which the Mag date is updated
#define MAG_MIX               5		// 0 = Gyro only, 1 = Mag only, 2..n decreasing mag influence
#define MPU_LPF_RATE          5		// low pas filter rate and can be between 5 and 188Hz


static void fail(const char *s, const int ec)
{
  fprintf(stderr, s);
  exit(ec);
}

/*
 * Sensor offset calibration
 */
static int airspeed_auto_zero(int offset)
{
  printf("Auto zeroing airspeed_...\n"); fflush(stdout);
  int new_offset;
  int staticp = 0;
  int pitotp = 0;
  const int samples = 100;
  for (int i=0; i<samples;i++) {
    if (i%10 == 0) {
      ms5611_start_temp(2);
      bcm2835_delay(10);
      ms5611_read_temp(2);
      bcm2835_delay(10);
      printf("%d%%\r", i); fflush(stdout);
    }
    ms5611_start_press(2);
    bcm2835_delay(10);
    ms5611_read_press(2);
    bcm2835_delay(10);
    staticp += ms5611[0].P;
    pitotp  += ms5611[1].P;
  }
  new_offset = (pitotp - staticp)/samples;
  if (abs(new_offset) < 300) {
    eeprom_write((char*)&new_offset, EEP_OFFSET, sizeof(new_offset));
    offset = new_offset;
  }
  else
    printf("Invalid new offset %d\n", new_offset);
  printf("Offset = %d Pascal\n", offset);
  return offset;
}

static fixed ComputeVario(const fixed p, const fixed d_p)
{
  static const fixed FACTOR = fixed(-2260.389548275485);
  static const fixed EXP = fixed(-0.8097374740609689);
  return FACTOR*pow(p, EXP) * d_p;
}

/* rad to degrees */
static float r2d(const float r)
{
  return r * 180 / M_PI;
}

int main(int argc, char **argv)
{
  int sock, serial_sock, ahrs_sock=-1;

  snd_pcm_t *alsa_handle = NULL;
  VarioSynthesiser synth;

  /* from NVRAM */
  int offset = 0;
  unsigned uv = 0;
  
  if (!bcm2835_init())
    fail("bcm2835_init() failed\n", 1);

  /*
   * Configure IO pins
   */
  bcm2835_gpio_fsel(PI_USART_SELECT, BCM2835_GPIO_FSEL_INPT);	// not used
  bcm2835_gpio_set_pud(PI_USART_SELECT, BCM2835_GPIO_PUD_UP);

  eeprom_write_disable();

  ms5611_init();
  mcp3201_init();

  if (argc == 3) {
    /*
     * Write board revision to EEPROM
     */
    if (!strcmp(argv[1], "board")) {
      unsigned brd = strtol(argv[2], NULL, 10);
      eeprom_write((char*)&brd, EEP_BOARD, sizeof(brd));
      exit(1);
    }

    /*
     * Write ADC calibration to EEPROM
     */
    if (!strcmp(argv[1], "adcuv")) {
      // Write adc calibration constant
      uv = strtol(argv[2], NULL, 10);
      eeprom_write((char*)&uv, EEP_ADCCAL, sizeof(uv));
      // show measurement result
      int v = 0;
      for (unsigned i=0; i<100; i++) { v += mcp3201_read(); bcm2835_delay(25); }
      printf("mcp3201: %f Volt\n", v * uv * 1e-8);
      exit(1);
    }
  }

  if (argc != 2)
    fail("usage: raspisb inet_addr || raspisb adcuv uv_per_bit || raspisb board revision#\n", 1);

  /*
   * Read calibration and hw-configuration from EEPROM
   */
  eeprom_read((char*)&board_revision, EEP_BOARD, sizeof(board_revision));
  if (uv == 0xffff) board_revision = 0;
  printf("board_revision = %d\n", board_revision);

  eeprom_read((char*)&offset, EEP_OFFSET, sizeof(offset));
  if (abs(offset) >= 300) offset = airspeed_auto_zero(offset);
  else printf("Offset = %d Pascal\n", offset);

  eeprom_read((char*)&uv, EEP_ADCCAL, sizeof(uv));
  if (uv == 0xffff) uv = 0;
  printf("adcuv = %d\n", uv);

  /*
   * Read configuration file
   */
  read_config();

  if (config.audio.volume == 0) {
    // Set the output pin to Alt Fun 5, to allow PWM channel 0 to be output there
    bcm2835_gpio_fsel(RPI_GPIO_P1_12, BCM2835_GPIO_FSEL_ALT5);
    // Clock divider is set to 16.
    // With a divider of 2 and a RANGE of 1024, in MARKSPACE mode,
    // the pulse repetition frequency will be ~8kHz
    bcm2835_pwm_set_clock(BCM2835_PWM_CLOCK_DIVIDER_2);
    bcm2835_pwm_set_mode(0, 1, 1);
    bcm2835_pwm_set_range(0, 1024);
  } else {
    /* Configure audio vario */
    synth.SetFrequencies(config.audio.freqlow, config.audio.freqmid, config.audio.freqhigh); 
    synth.SetDeadBandRange(config.audio.deadbandlow, config.audio.deadbandhigh);
    synth.SetVolume(config.audio.volume);
    alsa_handle = alsa_setup(&synth);
  }

  /*
   * Prepare network connections for air data and serial data using TCP.
   */ 
  sock = socket(AF_INET, SOCK_STREAM, 0);
  serial_sock = socket(AF_INET, SOCK_STREAM, 0);
  
  /*
   * Try to start the mpu9150
   */
  MPU.selectDevice(MPU9150_DEVICE);                        // select the correct device address
  if (MPU.init(MPU_UPDATE_RATE, MAG_MIX, MPU_UPDATE_RATE, MPU_LPF_RATE)) { // start the MPU
    /* Prepare network connection for ahrs using UDP */
    ahrs_sock = socket(AF_INET, SOCK_DGRAM, 0);
    /* Set heading kalman filter parameter. */
    hkf.SetAccelerationVariance(fixed(0.03));
  }

  /*
   * Read static pressure and init all pressures to this value.
   */
  ms5611_start_temp(1);
  bcm2835_delay(10);
  ms5611_read_temp(1);
  ms5611_start_press(1);
  bcm2835_delay(10);
  ms5611_read_press(1);

  int staticp = ms5611[0].P;
  int totalp = staticp;
  fixed tep = fixed(staticp)/10;

  vkf.SetAccelerationVariance(fixed(config.VarioAccel));

  /*
   * Init kalman filter with sensible value
   */
  for (unsigned i=0; i<1000; i++)
    vkf.Update(tep, fixed(0.25), 1);

  /*
   * Start connection thread.
   */
  in_addr_t xcsoar = inet_addr(argv[1]);
  new std::thread(connect2xcsoar, sock, serial_sock, ahrs_sock,
                                  &flags, xcsoar);

  printf("\n"); fflush(stdout);

  /*
   * Main loop, read sensors and send results over wlan as NMEA string(s).
   * First some more variables.
   */
  uint64_t last_send = 0;
  uint64_t last_ahrs_send = 0;
  uint64_t last_temp = 0;
  uint64_t last_press = 0;

  fixed Heading = fixed(0), last_heading = fixed(0), G = fixed(1), Yawps = fixed(0);
  int Slip = 0;

  char s[1024];     // string to send
  ssize_t l = 0;    // and its length

  const int nr_of_sensors = (config.ecomp ? 2 : 3);

  enum state_T { IDLE, TEMP, PRESS } state = IDLE;
  uint64_t state_timer = 0;

  /*
   * The main loop
   */
  while (1) {
    if (bcm2835_gpio_lev(CALIBRATE) == LOW) offset = airspeed_auto_zero(offset);

#if !ALSA_ASYNC
    if (alsa_handle) alsa_fill_buffer(&synth, alsa_handle);
#endif

    uint64_t t = bcm2835_st_read();

    /*
     * Pressure sensors
     */
    switch (state) {
      default:
      //case IDLE:
        // measure temperatures every 200ms 
        if ((t - last_temp) > 1000000/TEMP_SAMPLE_RATE) {
          ms5611_start_temp(nr_of_sensors);
          state = TEMP;
          state_timer = t + 10000;
        } else if ((t - last_press) > 1000000/PRESSURE_SAMPLE_RATE) {
          ms5611_start_press(nr_of_sensors);
          state = PRESS;
          state_timer = t + 10000;
        }
        break;
      case TEMP:
        if (t < state_timer) break;
        ms5611_read_temp(nr_of_sensors);
        last_temp = t;
        state = IDLE;
        break;
      case PRESS:
        if (t < state_timer) break;
        ms5611_read_press(nr_of_sensors);
        if (config.ecomp) {
          staticp = ms5611[0].P;
          totalp  = ms5611[1].P - offset;
          tep = (fixed(staticp) * (fixed(1) + fixed(config.ecomp)/100) - fixed(totalp))/100;
          fixed dt = fixed(t - last_press) * fixed(1e-6); 
          vkf.Update(tep, fixed(0.25), dt);
          last_press = t;
        } else {
          /*
           * Lowpass filter.
           */
          staticp = (staticp*3 + ms5611[0].P) / 4;
          totalp  = (totalp*3 + ms5611[1].P - offset) / 4;
          /*
           * measure TE pressure and feed to kalman filter
           */
          tep = fixed(ms5611[2].P)/100;
          fixed dt = fixed(t - last_press) * fixed(1e-6); 
          vkf.Update(tep, fixed(0.25), dt);
          last_press = t;
        }
        state = IDLE;
	break;
    }

    if (state != IDLE)
      /* Lower noise on MS5611 when processor ist idle during conversion */
      bcm2835_delay(10);
    else
      /* Lower power consumption when processor idle */
      bcm2835_delay(5);

    /*
     * MPU9150, works but not (yet?) usefull.
     */
    if (ahrs_sock && MPU.read()) {                       // get the latest data

      /* Compute yaw rate */
      Heading = r2d(MPU.m_fusedEulerPose[2]);
      fixed yaw = (Heading - last_heading) * MPU_UPDATE_RATE;		// heading change per second
      if (abs(yaw) < 100) hkf.Update(yaw, fixed(0.25), fixed(1)/MPU_UPDATE_RATE); // remove singularities around 360 degrees
      Yawps = hkf.GetXAbs(); 

      /* Compute G load */
      G = sqrt(sqr((fixed)MPU.m_calAccel[0]/16536) +
                     sqr((fixed)MPU.m_calAccel[1]/16536) +
                     sqr((fixed)MPU.m_calAccel[2]/16536));
    }

    /*
     * Send air data.
     */
    if ((t - last_send) > (1000000 / AIRDATA_SEND_RATE)) {

      fixed uvolt = fixed(mcp3201_read() * uv);

      fixed XAbs = vkf.GetXAbs();
      fixed vario = ComputeVario(XAbs, vkf.GetXVel()); 

      int dynp  = abs(totalp - staticp);
      int altitude = (int)(fixed(44330.8) - 
                  fixed(4946.54)*pow(fixed(staticp), fixed(0.1902632)));
      fixed ias = sqrt(fixed(21.15918367) * dynp);	// km/h
      fixed tas = ias * AirDensityRatio(altitude);

      fixed vario_sollfahrt = vario;
      if (!circling)
        vario_sollfahrt = (ias - speed2fly) / 10; /* range +- 50 km/h */

      if (alsa_handle)
        synth.SetVario(11025, vario_sollfahrt);
      else {
        int pwm = Clamp(512 + (int)(vario_sollfahrt * 102), 0, 1023);
        bcm2835_pwm_set_data(0, pwm);
      }

      // Emulate Westerboer VW1150
      // $PWES0,,TEvario,,,,,altitude,,ias,tas,voltage,(Temperature),ChkSum
      /* The abs() allows checking of correct offset calibration while on the ground.
       * The displayed speed is always positiv and should be below ~25km/h (30 Pascal) */
      int v = (int)(uvolt * fixed(1e-5));
      if (v)
        l = sprintf(s, "$PWES0,,%d,,,,,%d,,%d,%d,%d,,",
                       (int)(vario*10), altitude,
                       (int)(ias*10), (int)(tas*10), v);
      else
        l = sprintf(s, "$PWES0,,%d,,,,,%d,,%d,%d,,,",
                       (int)(vario*10), altitude,
                       (int)(ias*10), (int)(tas*10));
      l += sprintf(s + l, "*%02X\n", NMEAChecksum(s));
 
      if (flags & HAVE_AIRDATA) {
        ssize_t r = send(sock, s, l, 0);
        if (r != l)
          fail("Air data send failed\n", 1);
      }
      last_send = bcm2835_st_read();
    }

    /*
     * Send AHRS data.
     */
    if (ahrs_sock && (t - last_ahrs_send) > (1000000 / AIRDATA_SEND_RATE)) {
      // Emulate Levil AHRS
      /* $RPYL,roll,pitch,heading,slip,yaw,G,error,checksum, all angles in 1/10 of a degree, G in 1000 */
      l = sprintf(s, "$RPYL,%d,%d,%d,%d,%d,%d,0,", (int)(r2d(MPU.m_fusedEulerPose[0]) * 10),
                                                   (int)(r2d(MPU.m_fusedEulerPose[1]) * 10),
                                                   (int)(Heading * 10),
                                                   Slip, (int)(Yawps * 10),
                                                   (int)(G*1000));
      l += sprintf(s + l, "*%02X\n", NMEAChecksum(s));
      if (flags & HAVE_MPU9150) {
        ssize_t r = send(ahrs_sock, s, l, 0);
        if (r != l)
          fail("AHRS send failed\n", 1);
      }

      //printf("ahrs %d ms  yaw = %.1f        \r", (ahrst-last_ahrs)/10, Yawps); fflush(stdout);

      if (last_ahrs_send == 0) last_ahrs_send = bcm2835_st_read() + 5000;
      else last_ahrs_send = bcm2835_st_read();
      last_heading = Heading;
    }
  }

  // we never come here ....
  ms5611_exit();
  mcp3201_exit();
  return 0;
}
