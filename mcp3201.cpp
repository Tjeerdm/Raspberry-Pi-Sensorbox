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

#include <stdio.h>
#include <string.h>
#include "bcm2835.hpp"
#include "mcp3201.hpp"

void mcp3201_init()
{
  bcm2835_spi_begin();
  bcm2835_spi_chipSelect(BCM2835_SPI_CS0);
  bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);
  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
  bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_512);	// ~500kHz
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);
}

void mcp3201_exit()
{
  bcm2835_i2c_end();
}

unsigned mcp3201_read(void)
{
  union {
    char b[2];
    unsigned short s;
  } d;
  bcm2835_spi_transfern((char*)&(d.b[0]), sizeof d);
  // byte swap
  unsigned char temp = d.b[0]; d.b[0] = d.b[1]; d.b[1] = temp;
  // 3 leading rubbish bits, so shift one more and mask the rubbish.
  return (d.s >> 2) & 0xfff ;
}
