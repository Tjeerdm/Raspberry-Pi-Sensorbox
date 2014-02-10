////////////////////////////////////////////////////////////////////////////
//
//  This file is part of MPU9150Lib
//
//  Copyright (c) 2013 Pansenti, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of 
//  this software and associated documentation files (the "Software"), to deal in 
//  the Software without restriction, including without limitation the rights to use, 
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
//  Software, and to permit persons to whom the Software is furnished to do so, 
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all 
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION 
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <stdio.h>
#include <string.h>
#include "CalLib.h"
#include "24lc16.hpp"

#if 0
void calLibErase(byte device)
{
    char null = 0;
    eeprom_write(&null, MPU9150_CAL + sizeof(CALLIB_DATA) * device, 1); // just destroy the valid byte
}
#endif

void calLibWrite(byte device, CALLIB_DATA *calData)
{
  byte *ptr = (byte *)calData;
  byte length = sizeof(CALLIB_DATA);
  int eeprom = MPU9150_CAL + sizeof(CALLIB_DATA) * device;

  calData->valid = CALLIB_DATA_VALID;
  
  for (unsigned i = 0; i < length; i+=16)
    eeprom_write(ptr+i, eeprom+i, 16);
}

boolean calLibRead(byte device, CALLIB_DATA *calData)
{
  byte *ptr = (byte *)calData;
  byte length = sizeof(CALLIB_DATA);
  int eeprom = MPU9150_CAL + sizeof(CALLIB_DATA) * device;

  calData->magValid = false;
  calData->accelValid = false;

  char d[2];
  memset(d, 0, sizeof d);
  eeprom_read(&d[0], eeprom, 1);
  eeprom_read(&d[1], eeprom+1, 1);
  if ((d[0] != CALLIB_DATA_VALID_LOW) ||
      (d[1] != CALLIB_DATA_VALID_HIGH)) {
    printf("NO CALIBRATION DATA FOUND\n");
    return false;                                  // invalid data
  }
    
  for (unsigned i = 0; i < length; i++)
    eeprom_read(ptr++, eeprom + i, 1);

  printf("Calibration data found\n");
  return true;  
}

