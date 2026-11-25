// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/math_functions.h"

#include <float.h>
#include <math.h>

static int Near( float a, float b )
{
	float d = a - b;
	if ( d < 0.0f )
	{
		d = -d;
	}
	return d <= 0.00001f;
}

int VectorTest( void )
{
	b2Vec2 zero = b2Vec2_zero;
	b2Vec2 one = { 1.0f, 1.0f };
	b2Vec2 two = { 2.0f, 2.0f };
	b2Vec2 sum = b2Add( one, two );
	b2Vec2 difference = b2Sub( zero, two );
	b2Vec2 negated = b2Neg( two );
	if ( sum.x != 3.0f || sum.y != 3.0f )
	{
		return 1;
	}
	if ( difference.x != -2.0f || difference.y != -2.0f )
	{
		return 1;
	}
	if ( negated.x != -2.0f || negated.y != -2.0f )
	{
		return 1;
	}

	b2Vec2 xAxis = { 1.0f, 0.0f };
	b2Vec2 yAxis = { 0.0f, 1.0f };
	if ( b2Dot( xAxis, yAxis ) != 0.0f || b2Cross( xAxis, yAxis ) != 1.0f )
	{
		return 1;
	}
	b2Vec2 crossed = b2CrossVS( xAxis, 2.0f );
	if ( crossed.x != 0.0f || crossed.y != -2.0f )
	{
		return 1;
	}
	b2Vec2 left = b2LeftPerp( xAxis );
	if ( left.x != 0.0f || left.y != 1.0f )
	{
		return 1;
	}

	b2Vec2 threeFour = { 3.0f, 4.0f };
	if ( Near( b2Length( threeFour ), 5.0f ) == 0 || b2LengthSquared( threeFour ) != 25.0f )
	{
		return 1;
	}
	if ( b2Length( zero ) != 0.0f )
	{
		return 1;
	}

	b2Vec2 unit = b2Normalize( threeFour );
	if ( Near( unit.x, 0.6f ) == 0 || Near( unit.y, 0.8f ) == 0 || b2IsNormalized( unit ) == false )
	{
		return 1;
	}
	b2Vec2 normalizedZero = b2Normalize( zero );
	if ( normalizedZero.x != 0.0f || normalizedZero.y != 0.0f )
	{
		return 1;
	}

	b2Vec2 finite = { -2.0f, FLT_MAX };
	b2Vec2 bad = { 1.0f, NAN };
	if ( b2IsValidVec2( finite ) == false || b2IsValidVec2( bad ) || b2IsValidVec2( b2Vec2{ INFINITY, 0.0f } ) )
	{
		return 1;
	}

	return 0;
}
