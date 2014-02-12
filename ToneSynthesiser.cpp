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

#include <unistd.h>
#include <string.h>

#include "ToneSynthesiser.hpp"
#include "ISINETABLE.h"

#define ARRAY_SIZE(x) (sizeof x / sizeof x[0])

extern int beep_file;

void
ToneSynthesiser::SetVolume(unsigned _volume)
{
  volume = _volume;
}

void
ToneSynthesiser::SetTone(unsigned sample_rate, unsigned tone_hz)
{
  increment = ARRAY_SIZE(ISINETABLE) * tone_hz / sample_rate;
}

void
ToneSynthesiser::Synthesise(int16_t *buffer, size_t n)
{
  //assert(angle < ARRAY_SIZE(ISINETABLE));

  if (beep_file >= 0) {
    memset(buffer, 0, n*sizeof(*buffer));
    size_t nn = read(beep_file, buffer, n * sizeof(*buffer));
    if (nn == n) return;
    close(beep_file); beep_file = -1;
    if (nn) return;
  }
  for (int16_t *end = buffer + n; buffer != end; ++buffer) {
    *buffer = ISINETABLE[angle] * (32767 / 1024) * (int)volume / 100;
    angle = (angle + increment) & (ARRAY_SIZE(ISINETABLE) - 1);
  }
}

unsigned
ToneSynthesiser::ToZero() const
{
  //assert(angle < ARRAY_SIZE(ISINETABLE));

  if (angle < increment)
    /* close enough */
    return 0;

  return (ARRAY_SIZE(ISINETABLE) - angle) / increment;
}
