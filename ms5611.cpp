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
#include "ms5611.hpp"

#define DELAY 10

static inline uint8_t bcm2835_i2c_write_byte(const char b)
{
  return bcm2835_i2c_write(&b, 1);
}

static unsigned short read_prom(const unsigned i)
{
  unsigned short rv;
  unsigned char d[2];

  bcm2835_i2c_write_byte(0xa0 + 2*i); /* PROM location to read */
  bcm2835_i2c_read((char *)&d, sizeof(d));
  rv = (d[0]<<8) + d[1];
  //printf("PROM[%d] = %d\n", i, rv);
  return rv;
}

void ms5611_init()
{
  unsigned i;

  bcm2835_gpio_fsel(I2C_SEL, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_write(I2C_SEL, HIGH);

  bcm2835_i2c_begin();
  bcm2835_i2c_set_baudrate(100000);
  bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_2500);

  ms5611[0].address = 0x77; ms5611[0].select = HIGH; // U1
  ms5611[1].address = 0x76; ms5611[1].select = HIGH; // U2
  ms5611[2].address = 0x77; ms5611[2].select = LOW;  // U4
  //ms5611[3].address = 0x76; ms5611[2].select = LOW;  // U3, reserved for AOA

  for (i=0; i<sizeof ms5611/sizeof ms5611[0]; i++) {
    bcm2835_gpio_write(I2C_SEL, ms5611[i].select);
    bcm2835_i2c_setSlaveAddress(ms5611[i].address);

    bcm2835_i2c_write_byte(0b00011110); bcm2835_delay(DELAY); /* send reset command 0x1e */
    bcm2835_i2c_write_byte(0b00011110); bcm2835_delay(DELAY); /* send reset command 0x1e */
    bcm2835_i2c_write_byte(0b00011110); bcm2835_delay(DELAY); /* send reset command 0x1e */

    unsigned short n_prom[8];
    for (unsigned i=0; i<8; i++) n_prom[i] = read_prom(i);
    // TODO: check CRC

    ms5611[i].C1s = n_prom[1] << 15;
    ms5611[i].C2s = n_prom[2] << 16;
    ms5611[i].C3  = n_prom[3];
    ms5611[i].C4  = n_prom[4];
    ms5611[i].C5s = n_prom[5] << 8;
    ms5611[i].C6  = n_prom[6];
  }
}

void ms5611_exit()
{
  bcm2835_i2c_end();
}

void ms5611_start_temp(unsigned nr)
{
  for (unsigned i=0; i<nr; i++) {
    struct ms5611_t *data = &ms5611[i];
    bcm2835_gpio_write(I2C_SEL, data->select);
    bcm2835_i2c_setSlaveAddress(data->address);
    bcm2835_i2c_write_byte(0b01011000); // Convert D2, OSR=4096
  }
  bcm2835_gpio_write(I2C_SEL, HIGH);
}

void ms5611_start_press(unsigned nr)
{
  for (unsigned i=0; i<nr;i++) {
    struct ms5611_t *data = &ms5611[i];
    bcm2835_gpio_write(I2C_SEL, data->select);
    bcm2835_i2c_setSlaveAddress(data->address);
    bcm2835_i2c_write_byte(0b01001000); // Convert D1, OSR=4096
  }
  bcm2835_gpio_write(I2C_SEL, HIGH);
}

void ms5611_read_temp(unsigned nr)
{
  uint8_t D[3];
  uint32_t D2;
  int32_t off2 = 0, sens2 = 0;

  for (unsigned i=0; i<nr; i++) {
    struct ms5611_t *data = &ms5611[i];
    bcm2835_gpio_write(I2C_SEL, data->select);
    bcm2835_i2c_setSlaveAddress(data->address);
    bcm2835_i2c_write_byte(0);
    bcm2835_i2c_read((char *)&D, sizeof(D));
    D2 = (D[0]<<16) + (D[1]<<8) + D[2];
    int32_t ldT = D2 - data->C5s;
    int64_t dT = ldT;
    int32_t TEMP = (int)(2000 + ((dT * data->C6) >> 23));
    data->TEMP = TEMP;

    // 2nd order pressure correction
    if (TEMP < 2000) {
      int32_t d2 = TEMP - 2000; d2 = d2 * d2;
      off2 = (5 * d2)/2;
      sens2 = (5 * d2)/4;
      if (TEMP < -1500) {
        d2 = TEMP + 1500; d2 = d2 * d2;
        off2 += 7 * d2;
        sens2 += (11 * d2) / 2;
      }
    }

    data->OFFSET  = data->C2s + ((dT * data->C4) / 128L) - off2;
    data->SENS = data->C1s + ((dT * data->C3) / 256L) - sens2;
#if 0
    printf("dT = %ld\n", dT);
    printf("D2 = %lx\n", D2);
    printf("TEMP = %ld\n", TEMP);
#endif
  }
  bcm2835_gpio_write(I2C_SEL, HIGH);
}

void ms5611_read_press(unsigned nr)
{
  uint8_t D[3];
  uint32_t D1;

  for (unsigned i=0; i<nr;i++) {
    struct ms5611_t *data = &ms5611[i];
    bcm2835_gpio_write(I2C_SEL, data->select);
    bcm2835_i2c_setSlaveAddress(data->address);
    bcm2835_i2c_write_byte(0);
    bcm2835_i2c_read((char *)&D, sizeof(D));
    D1 = (D[0]<<16) + (D[1]<<8) + D[2];
    data->P = (unsigned)((((D1 * data->SENS) >> 21) - data->OFFSET) >> 15);
#if 0
    printf("P = %d\n", data->P);
#endif
  }
  bcm2835_gpio_write(I2C_SEL, HIGH);
}

