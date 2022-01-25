/******************************************************************************
 * Copyright (C) 2022 dyifm.com
 * This file is part of FM3-public <https://github.com/dyifm/fm3-public>.
 *
 * fm3-public is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * fm3-public is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with fm3-public.  If not, see <http://www.gnu.org/licenses/>.
 *******************************************************************************/
#ifndef MOVING_AVG_H
#define MOVING_AVG_H

#include "global.h"

#define BUFFER_SIZE 50

class MovingAvg
{
  public:
    MovingAvg();

    float add(uint16_t);
    float getAverage();
    float get();
    int getSize();

  private:
    float mAverage = 0;
    int INDEX = 0;
    int SUM = 0;

    uint16_t mBuffer[BUFFER_SIZE];
};

#endif
