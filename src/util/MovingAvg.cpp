/******************************************************************************
 * Copyright (C) 2022 dyifm.com
 * This file is part of FM3-public <https://github.com/dyifm/fm3-public>.
 *
 * FM3-public is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * FM3-public is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with FM3-public.  If not, see <http://www.gnu.org/licenses/>.
 *******************************************************************************/
#include "MovingAvg.h"

MovingAvg::MovingAvg()
{
}

float MovingAvg::add(uint16_t value)
{
  SUM = SUM - mBuffer[INDEX];        // Remove the oldest entry from the sum
  mBuffer[INDEX] = value;            // Add the newest reading to the window
  SUM = SUM + value;                 // Add the newest reading to the sum
  INDEX = (INDEX + 1) % BUFFER_SIZE; // Increment the index, and wrap to 0 if it exceeds the window size

  mAverage = SUM / BUFFER_SIZE; // Divide the sum of the window by the window size for the result

  return mAverage;
}

float MovingAvg::getAverage()
{
  return mAverage;
}

float MovingAvg::get()
{
  return mAverage;
}

int MovingAvg::getSize()
{
  return BUFFER_SIZE;
}
