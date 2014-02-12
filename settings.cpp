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

#include <exception>
#include <thread>
#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include "Fixed.hpp"
#include "serial.hpp"

extern int sock;

extern bool circling;
extern fixed speed2fly;

int beep_file = -1;

static void PRSB1(char *b)
{
  int stf=-1;
  if (*b != ',') circling = strtol(b, &b, 10) ? true:false;
  b++;
  if (*b != ',') speed2fly = fixed(strtol(b, &b, 10));
  b++;
}

static void PRSB2(char *b)
{
  char riff_header[44];
  if (beep_file <0) {
    if (strstr(b, "IRD_WAV_BEEPBWEEP")) beep_file = open("/boot/beep_bweep.wav", O_RDONLY);
    else if (strstr(b, "IRD_WAV_CLEAR")) beep_file = open("/boot/beep_clear.wav", O_RDONLY);
    else if (strstr(b, "IRD_WAV_DRIP")) beep_file = open("/boot/beep_drip.wav", O_RDONLY);
    if (beep_file >= 0) read(beep_file, riff_header, sizeof riff_header);
  }
}

/*
 * Unfortunately the lines are not send in one packet.
 * So put them together again.
 */
static int readNMEAline(const int sock, char *s)
{
  char b[256];
  ssize_t size = 1;

  s[0] = 0;
  while (size) {
    memset(b, 0, sizeof b);
    size = recv(sock, b, sizeof b, 0);
    char *start = strchr(b, '$');
    if (start) {
      strcpy(s, start);
      break;
    }
  }
  while (!strchr(b, '\n') && size) {
    memset(b, 0, sizeof b);
    size = recv(sock, b, sizeof b, 0);
    strcat(s, b);
  }
  return strlen(s);
}

void receive_settings(const int sock)
{
  char s[1024];
  memset(s, sizeof s, 0);

  try {
    while (1) {
      if (!readNMEAline(sock, s)) throw(0);
      if (!memcmp(s, "$PRSB1,", 7))
        PRSB1(s+7);
      else if (!memcmp(s, "$PRSB2,", 7))
        PRSB2(s+7);
    }
  }
  catch ( ... )
  {
    printf("Settings thread died\n"); fflush(stdout);
    exit(1);
  }
}

