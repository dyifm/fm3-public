FILENAME_BUILDNO = 'versioning'
FILENAME_VERSION_H = 'src/appversion.h'
majorVersion ='0'
minorVersion = '1'

import datetime

patchVersion = 0
try:
    with open(FILENAME_BUILDNO) as f:
        patchVersion = int(f.readline()) + 1
except:
    print('Starting build number from 1..')
    patchVersion = 1
with open(FILENAME_BUILDNO, 'w+') as f:
    f.write(str(patchVersion))
    print('Build number: {}'.format(patchVersion))

buildDateTime = datetime.datetime.now().isoformat()

hf = """
#ifndef VERSION_H
#define VERSION_H

// DO NOT MODIFY - AUTOGENERATED -

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

#include <Arduino.h>

struct TVersion 
{{
  int major = {};
  int minor = {};
  int patch = {};
  String buildDateTime = "{}";
  String stringVersion = "{}.{}.{}";
  String stringVersionDateTime = "{}.{}.{}+{}";
}};

#ifndef APPVERSION
const TVersion APPVERSION;
#endif

#endif
""".format(majorVersion, minorVersion, patchVersion, buildDateTime, 
majorVersion, minorVersion, patchVersion, 
majorVersion, minorVersion, patchVersion, buildDateTime)
with open(FILENAME_VERSION_H, 'w+') as f:
    f.write(hf)
