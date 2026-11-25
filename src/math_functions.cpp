// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/math_functions.h"

bool b2IsValidFloat( float a )
{
	if ( isnan( a ) )
	{
		return false;
	}

	if ( isinf( a ) )
	{
		return false;
	}

	return true;
}

bool b2IsValidVec2( b2Vec2 v )
{
	if ( isnan( v.x ) || isnan( v.y ) )
	{
		return false;
	}

	if ( isinf( v.x ) || isinf( v.y ) )
	{
		return false;
	}

	return true;
}

b2Rot b2ComputeRotationBetweenUnitVectors( b2Vec2 v1, b2Vec2 v2 )
{
	B2_ASSERT( b2AbsFloat( 1.0f - b2Length( v1 ) ) < 100.0f * FLT_EPSILON );
	B2_ASSERT( b2AbsFloat( 1.0f - b2Length( v2 ) ) < 100.0f * FLT_EPSILON );

	b2Rot rot;
	rot.c = b2Dot( v1, v2 );
	rot.s = b2Cross( v1, v2 );
	return b2NormalizeRot( rot );
}
