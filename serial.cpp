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
#include <netinet/in.h>
#include <arpa/inet.h>
#include <arpa/inet.h>
#include "serial.hpp"
#include "config.hpp"
#include "bcm2835.hpp"

static void serial_send(int ttyAMA0, int serial_socket)
{
  char b[1024];

  for (;;) {
    ssize_t l = recv(serial_socket, b, sizeof(b), 0);
    if ((l <= 0) || (write(ttyAMA0, &b, l) != l)) exit(1);
  }
}

void start_serial(int serial_sock)
{
  int ttyAMA0 = open("/dev/ttyAMA0", O_RDWR);
  if (ttyAMA0 <= 0) return;

  int speed;
  switch (config.baudrate ) {
    case 4800:  speed = B4800; break;
    case 9600:  speed = B9600; break;
    case 38400: speed = B38400; break;
    default:    speed = B19200; break;  //Flarm default
  }
  struct termios t;
  tcgetattr(ttyAMA0, &t);
  cfmakeraw(&t);
  t.c_cc[VMIN] = ~0; // Get as many characters as possible untill transmitter pauses more then 100ms
  t.c_cc[VTIME] = 1; // 100ms
  cfsetispeed(&t, speed);
  cfsetospeed(&t, speed);
  tcsetattr(ttyAMA0, TCSANOW, &t);

  char b[1024];

  new std::thread(serial_send, ttyAMA0, serial_sock);

  for (;;) {
    ssize_t s = read(ttyAMA0, &b, sizeof b);
    if (send(serial_sock, b, s, 0) != s) exit(1);
  }
}
