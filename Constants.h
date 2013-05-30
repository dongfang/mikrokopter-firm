#ifndef CONSTANTS_h
#define CONSTANTS_h

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define PI 3.1415926535897932384626433832795
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105
#define ToRad(x) ((x)*DEG_TO_RAD)  // *pi/180
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define INT16DEG_PI_FACTOR ((1L<<15) / PI)

/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 * rotations.h
 * Copyright (C) Andrew Tridgell 2012
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// these rotations form a full set - every rotation in the following
// list when combined with another in the list forms an entry which is
// also in the list. This is an important property. Please run the
// rotations test suite if you add to the list.

// these rotation values are stored to EEPROM, so be careful not to
// change the numbering of any existing entry when adding a new entry.
enum Rotation {
	ROTATION_NONE = 0,
	ROTATION_YAW_45,
	ROTATION_YAW_90,
	ROTATION_YAW_135,
	ROTATION_YAW_180,
	ROTATION_YAW_225,
	ROTATION_YAW_270,
	ROTATION_YAW_315,
	ROTATION_ROLL_180,
	ROTATION_ROLL_180_YAW_45,
	ROTATION_ROLL_180_YAW_90,
	ROTATION_ROLL_180_YAW_135,
	ROTATION_PITCH_180,
	ROTATION_ROLL_180_YAW_225,
	ROTATION_ROLL_180_YAW_270,
	ROTATION_ROLL_180_YAW_315,
    ROTATION_MAX
};

#endif
