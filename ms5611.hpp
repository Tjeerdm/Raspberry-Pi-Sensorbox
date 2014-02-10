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

#include "Fixed.hpp"

#define CALIBRATE RPI_V2_GPIO_P1_11
#define I2C_SEL RPI_V2_GPIO_P1_13
#define I2C_ENN RPI_V2_GPIO_P1_15

struct ms5611_t {
  unsigned select;
  unsigned address;
  uint32_t C1s;
  uint32_t C2s;
  uint32_t C3;
  uint32_t C4;
  uint32_t C5s;
  uint32_t C6;
  int64_t OFFSET;
  int64_t SENS;
  int TEMP;
  unsigned P;
};

extern struct ms5611_t ms5611[3];

void ms5611_init();
void ms5611_exit();

void ms5611_start_temp(unsigned nr);
void ms5611_start_press(unsigned nr);
void ms5611_read_temp(unsigned nr);
void ms5611_read_press(unsigned nr);

