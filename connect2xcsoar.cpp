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

#include <thread>
#include <sys/types.h>
#include <sys/socket.h>
#include <string.h>

#include "raspisb.hpp"
#include "bcm2835.hpp"
#include "connect2xcsoar.hpp"
#include "settings.hpp"
#include "serial.hpp"
#include "config.hpp"

static bool connectioncheck(int sock)
{
  const char *b = "Connection check\n";
  for (int i=0; i<1000; i++) {
    ssize_t r = send(sock, b, strlen(b), 0);
    if (r != (ssize_t)strlen(b)) return true;
  }
  return false;
}

void connect2xcsoar(int sock, int serial_sock, int ahrs_sock,
                    int *flags, in_addr_t xcsoar)
{
  while (1) {
    int waittime = 10000;
    /*
     * Try to open a connection for airdata.
     */
    if (!(*flags & HAVE_AIRDATA) && sock>0) {
      struct sockaddr_in server4353;
      server4353.sin_addr.s_addr = xcsoar;
      server4353.sin_family = AF_INET;
      server4353.sin_port = htons(4353);

      if (connect(sock, (struct sockaddr *)&server4353, sizeof(server4353)) >= 0) {
        *flags |= HAVE_AIRDATA;
        printf("Sending air data\n"); fflush(stdout);
        /*
         * Start thread to receive settings from XCSoar
         */
        new std::thread(receive_settings, sock);
      } else {
        if (errno == ECONNREFUSED)
          /* someone is there but not (yet) listening */
          waittime = 2000;

        printf("connect returned with errno = %d\n", errno);
      }
    }

    /*
     * Try to open a connection for serial data.
     */
    if (!(*flags & HAVE_SERIAL) && serial_sock>0) {
      struct sockaddr_in server4352;
      server4352.sin_addr.s_addr = xcsoar;
      server4352.sin_family = AF_INET;
      server4352.sin_port = htons(4352);

      if (connect(serial_sock, (struct sockaddr *)&server4352, sizeof(server4352)) >= 0) {
        *flags |= HAVE_SERIAL;
        /*
         * Start thread to send and receive serial data
         */
        new std::thread(start_serial, serial_sock);
      }
    }

    /*
     * Try to open a connection for AHRS.
     */
    if ((*flags & HAVE_AIRDATA) && !(*flags & HAVE_MPU9150) && (ahrs_sock>0)) {
      struct sockaddr_in ahrs_server;
      ahrs_server.sin_addr.s_addr = xcsoar;
      ahrs_server.sin_family = AF_INET;
      ahrs_server.sin_port = htons(4353);

      if (connect(ahrs_sock, (struct sockaddr *)&ahrs_server, sizeof(ahrs_server)) >= 0) {
        /*
         * For some reason it fails only after many packets.
         */
        if (connectioncheck(ahrs_sock)) {
          close(ahrs_sock); ahrs_sock = -1;
        } else {
          printf("Sending AHRS data\n"); fflush(stdout);
          *flags |= HAVE_MPU9150;
        }
      }
    }

    /* all connections established ? */
    if ((*flags & HAVE_AIRDATA) &&
        (*flags & HAVE_SERIAL) &&
        ((*flags & HAVE_MPU9150) ||
         (ahrs_sock < 0)))
      return;

    bcm2835_delay(waittime);
  }
}
