// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/math_functions.h"

static int Near( float a, float b )
{
	float d = a - b;
	if ( d < 0.0f )
	{
		d = -d;
	}
	return d <= 0.00001f;
}

int RotationTest( void )
{
	b2Rot identity = b2Rot_identity;
	if ( b2IsNormalizedRot( identity ) == false )
	{
		return 1;
	}

	b2Rot quarter = { 0.0f, 1.0f };
	b2Vec2 xAxis = { 1.0f, 0.0f };
	b2Vec2 rotated = b2RotateVector( quarter, xAxis );
	if ( Near( rotated.x, 0.0f ) == 0 || Near( rotated.y, 1.0f ) == 0 )
	{
		return 1;
	}

	b2Vec2 restored = b2InvRotateVector( quarter, rotated );
	if ( Near( restored.x, 1.0f ) == 0 || Near( restored.y, 0.0f ) == 0 )
	{
		return 1;
	}

	b2Rot inverse = b2InvertRot( quarter );
	b2Rot product = b2MulRot( quarter, inverse );
	if ( b2IsNormalizedRot( product ) == false || Near( product.c, 1.0f ) == 0 || Near( product.s, 0.0f ) == 0 )
	{
		return 1;
	}

	b2Rot halfTurn = b2MulRot( quarter, quarter );
	if ( Near( halfTurn.c, -1.0f ) == 0 || Near( halfTurn.s, 0.0f ) == 0 )
	{
		return 1;
	}

	b2Rot scaled = { 2.0f, 0.0f };
	b2Rot normalized = b2NormalizeRot( scaled );
	if ( b2IsNormalizedRot( normalized ) == false || Near( normalized.c, 1.0f ) == 0 )
	{
		return 1;
	}

	b2Vec2 from = { 1.0f, 0.0f };
	b2Vec2 to = { 0.0f, 1.0f };
	b2Rot between = b2ComputeRotationBetweenUnitVectors( from, to );
	b2Vec2 mapped = b2RotateVector( between, from );
	if ( Near( mapped.x, to.x ) == 0 || Near( mapped.y, to.y ) == 0 )
	{
		return 1;
	}

	b2Rot degenerate = { 0.0f, 0.0f };
	b2Rot normalizedDegenerate = b2NormalizeRot( degenerate );
	if ( normalizedDegenerate.c != 0.0f || normalizedDegenerate.s != 0.0f )
	{
		return 1;
	}

	return 0;
}
