#include <stdio.h>
#include <string.h>
#include "Fixed.hpp"
#include "config.hpp"

struct config_t config;

static bool read_flag(const char *f, const char *s)
{
  return (strstr(f, s) != 0);
}

static bool read_value(const char *f, const char *s, fixed *v)
{
  const char *p = strstr(f, s);
  if (p) *v = (unsigned)atof(p + strlen(s));
  return p != 0;
}

static bool read_value(const char *f, const char *s, unsigned *v)
{
  const char *p = strstr(f, s);
  if (p) *v = (unsigned)atoi(p + strlen(s));
  return p != 0;
}

void read_config()
{
  config.baudrate = 19200;
  config.ecomp = 0;
  config.noahrs = false;
  config.VarioAccel = fixed(.3);
  config.audio.volume = 0; /* saved power */
  config.audio.speed2fly = true;
  config.audio.deadbandlow = -0.1;
  config.audio.deadbandhigh = 0.1;
  config.audio.freqlow = 200;
  config.audio.freqmid = 500;
  config.audio.freqhigh = 1500;

  FILE *fp = fopen("/boot/raspisb.txt", "r");
  if (fp) {
    char s[2048]; memset(s, 0, sizeof s);
    fread(s, sizeof s, 1, fp);
    read_value(s, "BAUDRATE=", &config.baudrate);
    read_value(s, "VOLUME=", &config.audio.volume);
    read_value(s, "FREQLOW=", &config.audio.freqlow);
    read_value(s, "FREQMID=", &config.audio.freqmid);
    read_value(s, "FREQHIGH=", &config.audio.freqhigh);
    if (read_flag(s, "VARIOSLOW"))   config.VarioAccel = fixed(.03);
    if (read_flag(s, "VARIOMEDIUM")) config.VarioAccel = fixed(.3);
    if (read_flag(s, "VARIOFAST"))   config.VarioAccel = fixed(2);
  }
}

