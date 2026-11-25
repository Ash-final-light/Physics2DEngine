// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "box2d/collision.h"
#include "box2d/math_functions.h"
#include "test_macros.h"

static bool HullHasPoint( const b2Hull* hull, float x, float y )
{
	for ( int i = 0; i < hull->count; ++i )
	{
		if ( hull->points[i].x == x && hull->points[i].y == y )
		{
			return true;
		}
	}
	return false;
}

static float HullSignedArea( const b2Hull* hull )
{
	float area = 0.0f;
	for ( int i = 0; i < hull->count; ++i )
	{
		b2Vec2 a = hull->points[i];
		b2Vec2 b = hull->points[( i + 1 ) % hull->count];
		area += b2Cross( a, b );
	}
	return 0.5f * area;
}

int HullTest( void )
{
	b2Vec2 square[5] = {
		{ 0.0f, 0.0f },
		{ 1.0f, 0.0f },
		{ 1.0f, 1.0f },
		{ 0.0f, 1.0f },
		{ 0.5f, 0.5f },
	};
	b2Hull hull = b2ComputeHull( square, 5 );
	ENSURE( hull.count == 4 );
	ENSURE( HullHasPoint( &hull, 0.0f, 0.0f ) );
	ENSURE( HullHasPoint( &hull, 1.0f, 0.0f ) );
	ENSURE( HullHasPoint( &hull, 1.0f, 1.0f ) );
	ENSURE( HullHasPoint( &hull, 0.0f, 1.0f ) );
	ENSURE( HullHasPoint( &hull, 0.5f, 0.5f ) == false );
	ENSURE( HullSignedArea( &hull ) > 0.0f );
	ENSURE( b2ValidateHull( &hull ) == true );

	b2Vec2 withEdge[5] = {
		{ 0.0f, 0.0f }, { 1.0f, 0.0f }, { 0.5f, 0.0f }, { 1.0f, 1.0f }, { 0.0f, 1.0f },
	};
	b2Hull trimmed = b2ComputeHull( withEdge, 5 );
	ENSURE( trimmed.count == 4 );
	ENSURE( HullHasPoint( &trimmed, 0.5f, 0.0f ) == false );
	ENSURE( b2ValidateHull( &trimmed ) == true );

	b2Vec2 welded[4] = {
		{ 0.0f, 0.0f },
		{ 2.0f, 0.0f },
		{ 1.0f, 2.0f },
		{ 0.001f, 0.0f },
	};
	b2Hull merged = b2ComputeHull( welded, 4 );
	ENSURE( merged.count == 3 );
	ENSURE( b2ValidateHull( &merged ) == true );

	b2Vec2 line[3] = { { 0.0f, 0.0f }, { 1.0f, 0.0f }, { 2.0f, 0.0f } };
	ENSURE( b2ComputeHull( line, 3 ).count == 0 );

	b2Vec2 pair[2] = { { 0.0f, 0.0f }, { 1.0f, 0.0f } };
	ENSURE( b2ComputeHull( pair, 2 ).count == 0 );

	b2Vec2 tooMany[9];
	for ( int i = 0; i < 9; ++i )
	{
		tooMany[i] = { (float)i, ( i % 2 == 0 ) ? 0.0f : 1.0f };
	}
	ENSURE( b2ComputeHull( tooMany, 9 ).count == 0 );

	b2Hull clockwise = {};
	clockwise.count = 3;
	clockwise.points[0] = { 0.0f, 0.0f };
	clockwise.points[1] = { 0.0f, 1.0f };
	clockwise.points[2] = { 1.0f, 0.0f };
	ENSURE( b2ValidateHull( &clockwise ) == false );

	b2Hull empty = {};
	ENSURE( b2ValidateHull( &empty ) == false );
	return 0;
}
