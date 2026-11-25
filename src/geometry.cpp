// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "geometry.h"

#include "box2d/math_functions.h"

bool b2IsValidCircle( const b2Circle* circle )
{
	return b2IsValidVec2( circle->center ) && b2IsValidFloat( circle->radius ) && circle->radius >= 0.0f;
}

bool b2IsValidSegment( const b2Segment* segment )
{
	return b2IsValidVec2( segment->point1 ) && b2IsValidVec2( segment->point2 );
}

bool b2IsValidCapsule( const b2Capsule* capsule )
{
	return b2IsValidVec2( capsule->center1 ) && b2IsValidVec2( capsule->center2 ) && b2IsValidFloat( capsule->radius ) &&
		   capsule->radius >= 0.0f;
}
