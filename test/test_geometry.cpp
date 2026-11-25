// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "geometry.h"
#include "box2d/math_functions.h"
#include "test_macros.h"

#include <math.h>

int CircleSegmentTest( void )
{
	b2Circle circle = { { 1.0f, 2.0f }, 0.5f };
	ENSURE( circle.center.x == 1.0f && circle.center.y == 2.0f && circle.radius == 0.5f );
	ENSURE( b2IsValidCircle( &circle ) == true );

	b2Circle point = { { 0.0f, 0.0f }, 0.0f };
	ENSURE( b2IsValidCircle( &point ) == true );

	b2Circle negative = circle;
	negative.radius = -0.1f;
	ENSURE( b2IsValidCircle( &negative ) == false );

	b2Circle bad = circle;
	bad.radius = NAN;
	ENSURE( b2IsValidCircle( &bad ) == false );
	bad = circle;
	bad.center.x = INFINITY;
	ENSURE( b2IsValidCircle( &bad ) == false );

	b2Segment segment = { { -1.0f, 0.0f }, { 1.0f, 0.0f } };
	ENSURE( segment.point1.x == -1.0f && segment.point2.x == 1.0f );
	ENSURE( b2IsValidSegment( &segment ) == true );

	b2Segment degenerate = { { 3.0f, 4.0f }, { 3.0f, 4.0f } };
	ENSURE( b2IsValidSegment( &degenerate ) == true );

	b2Segment broken = segment;
	broken.point2.y = NAN;
	ENSURE( b2IsValidSegment( &broken ) == false );
	return 0;
}

int CapsuleTest( void )
{
	b2Capsule capsule = { { 0.0f, 0.0f }, { 0.0f, 2.0f }, 0.25f };
	ENSURE( capsule.center1.y == 0.0f && capsule.center2.y == 2.0f && capsule.radius == 0.25f );
	ENSURE( b2IsValidCapsule( &capsule ) == true );

	b2Capsule degenerate = { { 1.0f, 1.0f }, { 1.0f, 1.0f }, 0.5f };
	ENSURE( b2IsValidCapsule( &degenerate ) == true );

	b2Capsule point = { { -2.0f, 3.0f }, { -2.0f, 3.0f }, 0.0f };
	ENSURE( b2IsValidCapsule( &point ) == true );

	b2Capsule negative = capsule;
	negative.radius = -1.0f;
	ENSURE( b2IsValidCapsule( &negative ) == false );

	b2Capsule bad = capsule;
	bad.center2.x = NAN;
	ENSURE( b2IsValidCapsule( &bad ) == false );
	bad = capsule;
	bad.radius = INFINITY;
	ENSURE( b2IsValidCapsule( &bad ) == false );
	return 0;
}

int PolygonTest( void )
{
	b2Polygon box = b2MakeBox( 0.5f, 1.0f );
	ENSURE( box.count == 4 && box.radius == 0.0f );
	ENSURE( box.centroid.x == 0.0f && box.centroid.y == 0.0f );
	ENSURE( box.vertices[0].x == -0.5f && box.vertices[0].y == -1.0f );
	ENSURE( box.vertices[2].x == 0.5f && box.vertices[2].y == 1.0f );
	ENSURE( box.normals[0].x == 0.0f && box.normals[0].y == -1.0f );
	ENSURE( box.normals[1].x == 1.0f && box.normals[1].y == 0.0f );
	ENSURE( box.normals[2].y == 1.0f && box.normals[3].x == -1.0f );

	b2Polygon square = b2MakeSquare( 0.5f );
	ENSURE( square.count == 4 );
	ENSURE( square.vertices[1].x == 0.5f && square.vertices[1].y == -0.5f );
	ENSURE( square.vertices[3].x == -0.5f && square.vertices[3].y == 0.5f );

	b2Vec2 triangle[3] = { { 0.0f, 0.0f }, { 2.0f, 0.0f }, { 0.0f, 2.0f } };
	b2Hull hull = b2ComputeHull( triangle, 3 );
	ENSURE( hull.count == 3 );
	b2Polygon polygon = b2MakePolygon( &hull, 0.25f );
	ENSURE( polygon.count == 3 && polygon.radius == 0.25f );
	ENSURE_SMALL( polygon.centroid.x - 2.0f / 3.0f, 0.0001f );
	ENSURE_SMALL( polygon.centroid.y - 2.0f / 3.0f, 0.0001f );
	for ( int i = 0; i < 3; ++i )
	{
		ENSURE( polygon.vertices[i].x == hull.points[i].x && polygon.vertices[i].y == hull.points[i].y );
		int i2 = ( i + 1 ) % 3;
		b2Vec2 edge = b2Sub( polygon.vertices[i2], polygon.vertices[i] );
		b2Vec2 expected = b2Normalize( b2CrossVS( edge, 1.0f ) );
		ENSURE_SMALL( polygon.normals[i].x - expected.x, 0.0001f );
		ENSURE_SMALL( polygon.normals[i].y - expected.y, 0.0001f );
	}

	b2Hull empty = {};
	b2Polygon fallback = b2MakePolygon( &empty, 1.0f );
	ENSURE( fallback.count == 4 && fallback.radius == 0.0f );
	ENSURE( fallback.vertices[0].x == -0.5f && fallback.vertices[2].x == 0.5f );
	return 0;
}

int OffsetPolygonTest( void )
{
	b2Polygon rounded = b2MakeRoundedBox( 1.0f, 0.5f, 0.2f );
	b2Polygon plain = b2MakeBox( 1.0f, 0.5f );
	ENSURE( rounded.count == 4 && rounded.radius == 0.2f );
	ENSURE( rounded.centroid.x == 0.0f && rounded.centroid.y == 0.0f );
	ENSURE( rounded.vertices[0].x == plain.vertices[0].x && rounded.vertices[2].y == plain.vertices[2].y );
	ENSURE( rounded.normals[0].y == -1.0f && rounded.normals[1].x == 1.0f );

	b2Vec2 center = { 2.0f, 3.0f };
	b2Polygon shifted = b2MakeOffsetBox( 1.0f, 0.5f, center, b2Rot_identity );
	ENSURE( shifted.radius == 0.0f );
	ENSURE( shifted.centroid.x == 2.0f && shifted.centroid.y == 3.0f );
	ENSURE( shifted.vertices[0].x == 1.0f && shifted.vertices[0].y == 2.5f );
	ENSURE( shifted.vertices[2].x == 3.0f && shifted.vertices[2].y == 3.5f );
	ENSURE( shifted.normals[0].x == 0.0f && shifted.normals[0].y == -1.0f );

	b2Rot quarter = { 0.0f, 1.0f };
	b2Polygon turned = b2MakeOffsetRoundedBox( 1.0f, 0.5f, center, quarter, 0.3f );
	ENSURE( turned.radius == 0.3f );
	ENSURE( turned.centroid.x == 2.0f && turned.centroid.y == 3.0f );
	ENSURE_SMALL( turned.normals[0].x - 1.0f, 0.0001f );
	ENSURE_SMALL( turned.normals[0].y, 0.0001f );
	ENSURE_SMALL( turned.normals[1].x, 0.0001f );
	ENSURE_SMALL( turned.normals[1].y - 1.0f, 0.0001f );
	b2Vec2 local = { -1.0f, -0.5f };
	b2Vec2 expected = b2TransformPoint( { center, quarter }, local );
	ENSURE_SMALL( turned.vertices[0].x - expected.x, 0.0001f );
	ENSURE_SMALL( turned.vertices[0].y - expected.y, 0.0001f );

	b2Vec2 triangle[3] = { { 0.0f, 0.0f }, { 2.0f, 0.0f }, { 0.0f, 2.0f } };
	b2Hull hull = b2ComputeHull( triangle, 3 );
	b2Polygon moved = b2MakeOffsetRoundedPolygon( &hull, center, b2Rot_identity, 0.4f );
	b2Polygon sharp = b2MakeOffsetPolygon( &hull, center, b2Rot_identity );
	ENSURE( moved.count == 3 && moved.radius == 0.4f );
	ENSURE( sharp.radius == 0.0f && sharp.count == 3 );
	ENSURE_SMALL( moved.centroid.x - ( 2.0f / 3.0f + 2.0f ), 0.0001f );
	ENSURE_SMALL( moved.centroid.y - ( 2.0f / 3.0f + 3.0f ), 0.0001f );
	ENSURE_SMALL( sharp.centroid.x - moved.centroid.x, 0.0001f );
	ENSURE_SMALL( moved.vertices[0].x - ( hull.points[0].x + 2.0f ), 0.0001f );
	ENSURE_SMALL( moved.vertices[0].y - ( hull.points[0].y + 3.0f ), 0.0001f );

	b2Hull empty = {};
	b2Polygon bad = b2MakeOffsetRoundedPolygon( &empty, center, quarter, 1.0f );
	ENSURE( bad.count == 4 && bad.radius == 0.0f );
	ENSURE( bad.vertices[0].x == -0.5f && bad.centroid.x == 0.0f );
	return 0;
}
