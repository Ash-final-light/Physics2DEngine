// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "geometry.h"
#include "box2d/math_functions.h"
#include "test_macros.h"

#include <float.h>
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

int CircleMassTest( void )
{
	b2Circle circle = { { 1.0f, 0.0f }, 1.0f };
	b2MassData md = b2ComputeCircleMass( &circle, 1.0f );
	ENSURE_SMALL( md.mass - B2_PI, FLT_EPSILON );
	ENSURE( md.center.x == 1.0f && md.center.y == 0.0f );
	ENSURE_SMALL( md.rotationalInertia - 0.5f * B2_PI, FLT_EPSILON );

	b2MassData heavy = b2ComputeCircleMass( &circle, 2.0f );
	ENSURE_SMALL( heavy.mass - 2.0f * B2_PI, FLT_EPSILON );
	ENSURE_SMALL( heavy.rotationalInertia - B2_PI, FLT_EPSILON );
	ENSURE( heavy.center.x == 1.0f && heavy.center.y == 0.0f );
	return 0;
}

int CapsuleMassTest( void )
{
	b2Capsule capsule = { { -1.0f, 0.0f }, { 1.0f, 0.0f }, 1.0f };
	b2MassData md = b2ComputeCapsuleMass( &capsule, 1.0f );
	float circleMass = B2_PI;
	float boxMass = 4.0f;
	ENSURE_SMALL( md.mass - ( circleMass + boxMass ), FLT_EPSILON );
	ENSURE_SMALL( md.center.x, FLT_EPSILON );
	ENSURE_SMALL( md.center.y, FLT_EPSILON );

	float lc = 4.0f / ( 3.0f * B2_PI );
	float circleInertia = circleMass * ( 0.5f + 1.0f + 2.0f * lc );
	float boxInertia = boxMass * ( 4.0f + 4.0f ) / 12.0f;
	ENSURE_SMALL( md.rotationalInertia - ( circleInertia + boxInertia ), 4.0f * FLT_EPSILON );

	b2Capsule swapped = { { 1.0f, 0.0f }, { -1.0f, 0.0f }, 1.0f };
	b2MassData flipped = b2ComputeCapsuleMass( &swapped, 1.0f );
	ENSURE_SMALL( flipped.mass - md.mass, FLT_EPSILON );
	ENSURE_SMALL( flipped.rotationalInertia - md.rotationalInertia, FLT_EPSILON );
	ENSURE_SMALL( flipped.center.x, FLT_EPSILON );

	b2Capsule point = { { 1.0f, 2.0f }, { 1.0f, 2.0f }, 1.0f };
	b2MassData collapsed = b2ComputeCapsuleMass( &point, 1.0f );
	b2Circle circle = { { 1.0f, 2.0f }, 1.0f };
	b2MassData round = b2ComputeCircleMass( &circle, 1.0f );
	ENSURE_SMALL( collapsed.mass - round.mass, FLT_EPSILON );
	ENSURE( collapsed.center.x == 1.0f && collapsed.center.y == 2.0f );
	ENSURE_SMALL( collapsed.rotationalInertia - round.rotationalInertia, FLT_EPSILON );

	b2Capsule longer = { { -2.0f, 0.0f }, { 2.0f, 0.0f }, 1.0f };
	b2MassData stretched = b2ComputeCapsuleMass( &longer, 1.0f );
	ENSURE( stretched.mass > md.mass );
	ENSURE( stretched.rotationalInertia > md.rotationalInertia );
	return 0;
}

int PolygonMassTest( void )
{
	b2Polygon box = b2MakeBox( 1.0f, 1.0f );
	b2MassData md = b2ComputePolygonMass( &box, 1.0f );
	ENSURE_SMALL( md.mass - 4.0f, FLT_EPSILON );
	ENSURE_SMALL( md.center.x, FLT_EPSILON );
	ENSURE_SMALL( md.center.y, FLT_EPSILON );
	ENSURE_SMALL( md.rotationalInertia - 8.0f / 3.0f, 2.0f * FLT_EPSILON );

	b2Vec2 offset = { 0.4f, -0.7f };
	b2Polygon local = b2MakeBox( 0.25f, 0.5f );
	b2Polygon shifted = b2MakeOffsetBox( 0.25f, 0.5f, offset, b2Rot_identity );
	b2MassData m1 = b2ComputePolygonMass( &local, 1.0f );
	b2MassData m2 = b2ComputePolygonMass( &shifted, 1.0f );
	ENSURE_SMALL( m1.mass - m2.mass, FLT_EPSILON );
	ENSURE_SMALL( m1.rotationalInertia - m2.rotationalInertia, FLT_EPSILON );
	ENSURE_SMALL( m2.center.x - offset.x, FLT_EPSILON );
	ENSURE_SMALL( m2.center.y - offset.y, FLT_EPSILON );

	b2Vec2 triangle[3] = { { 0.0f, 0.0f }, { 2.0f, 0.0f }, { 0.0f, 2.0f } };
	b2Hull hull = b2ComputeHull( triangle, 3 );
	b2Polygon poly = b2MakePolygon( &hull, 0.0f );
	b2MassData tri = b2ComputePolygonMass( &poly, 1.0f );
	ENSURE_SMALL( tri.mass - 2.0f, 0.0001f );
	ENSURE_SMALL( tri.center.x - ( 2.0f / 3.0f ), 0.0001f );
	ENSURE_SMALL( tri.center.y - ( 2.0f / 3.0f ), 0.0001f );
	ENSURE( tri.rotationalInertia > 0.0f );

	b2Polygon rounded = b2MakeRoundedBox( 1.0f, 1.0f, 0.2f );
	b2MassData round = b2ComputePolygonMass( &rounded, 1.0f );
	ENSURE( round.mass > md.mass );
	ENSURE_SMALL( round.center.x, 0.0001f );
	ENSURE_SMALL( round.center.y, 0.0001f );

	b2Polygon one = {};
	one.count = 1;
	one.vertices[0] = { 1.0f, 0.0f };
	one.radius = 1.0f;
	b2Circle circle = { { 1.0f, 0.0f }, 1.0f };
	b2MassData oneMass = b2ComputePolygonMass( &one, 1.0f );
	b2MassData circleMass = b2ComputeCircleMass( &circle, 1.0f );
	ENSURE_SMALL( oneMass.mass - circleMass.mass, FLT_EPSILON );
	ENSURE_SMALL( oneMass.rotationalInertia - circleMass.rotationalInertia, FLT_EPSILON );

	b2Polygon two = {};
	two.count = 2;
	two.vertices[0] = { -1.0f, 0.0f };
	two.vertices[1] = { 1.0f, 0.0f };
	two.radius = 1.0f;
	b2Capsule capsule = { { -1.0f, 0.0f }, { 1.0f, 0.0f }, 1.0f };
	b2MassData twoMass = b2ComputePolygonMass( &two, 1.0f );
	b2MassData capsuleMass = b2ComputeCapsuleMass( &capsule, 1.0f );
	ENSURE_SMALL( twoMass.mass - capsuleMass.mass, FLT_EPSILON );
	ENSURE_SMALL( twoMass.rotationalInertia - capsuleMass.rotationalInertia, FLT_EPSILON );
	return 0;
}

int ShapeBoundsTest( void )
{
	b2Circle circle = { { 1.0f, 0.0f }, 1.0f };
	b2AABB circleBox = b2ComputeCircleBounds( &circle, b2WorldTransform_identity );
	ENSURE_SMALL( circleBox.lowerBound.x, FLT_EPSILON );
	ENSURE_SMALL( circleBox.lowerBound.y + 1.0f, FLT_EPSILON );
	ENSURE_SMALL( circleBox.upperBound.x - 2.0f, FLT_EPSILON );
	ENSURE_SMALL( circleBox.upperBound.y - 1.0f, FLT_EPSILON );

	b2WorldTransform moved = { { 3.0f, 4.0f }, b2Rot_identity };
	b2AABB shifted = b2ComputeCircleBounds( &circle, moved );
	ENSURE_SMALL( shifted.lowerBound.x - 3.0f, FLT_EPSILON );
	ENSURE_SMALL( shifted.lowerBound.y - 3.0f, FLT_EPSILON );
	ENSURE_SMALL( shifted.upperBound.x - 5.0f, FLT_EPSILON );
	ENSURE_SMALL( shifted.upperBound.y - 5.0f, FLT_EPSILON );

	b2Capsule capsule = { { -1.0f, 0.0f }, { 1.0f, 0.0f }, 1.0f };
	b2AABB capsuleBox = b2ComputeCapsuleBounds( &capsule, b2WorldTransform_identity );
	ENSURE_SMALL( capsuleBox.lowerBound.x + 2.0f, FLT_EPSILON );
	ENSURE_SMALL( capsuleBox.lowerBound.y + 1.0f, FLT_EPSILON );
	ENSURE_SMALL( capsuleBox.upperBound.x - 2.0f, FLT_EPSILON );
	ENSURE_SMALL( capsuleBox.upperBound.y - 1.0f, FLT_EPSILON );

	b2Polygon box = b2MakeBox( 1.0f, 1.0f );
	b2AABB boxBounds = b2ComputePolygonBounds( &box, b2WorldTransform_identity );
	ENSURE_SMALL( boxBounds.lowerBound.x + 1.0f, FLT_EPSILON );
	ENSURE_SMALL( boxBounds.lowerBound.y + 1.0f, FLT_EPSILON );
	ENSURE_SMALL( boxBounds.upperBound.x - 1.0f, FLT_EPSILON );
	ENSURE_SMALL( boxBounds.upperBound.y - 1.0f, FLT_EPSILON );

	b2Polygon rounded = b2MakeRoundedBox( 1.0f, 1.0f, 0.25f );
	b2AABB roundBounds = b2ComputePolygonBounds( &rounded, b2WorldTransform_identity );
	ENSURE_SMALL( roundBounds.lowerBound.x + 1.25f, FLT_EPSILON );
	ENSURE_SMALL( roundBounds.upperBound.y - 1.25f, FLT_EPSILON );

	b2Rot quarter = { 0.0f, 1.0f };
	b2WorldTransform turned = { { 1.0f, 2.0f }, quarter };
	b2Polygon flat = b2MakeBox( 2.0f, 0.5f );
	b2AABB spun = b2ComputePolygonBounds( &flat, turned );
	ENSURE_SMALL( spun.lowerBound.x - 0.5f, 0.0001f );
	ENSURE_SMALL( spun.upperBound.x - 1.5f, 0.0001f );
	ENSURE_SMALL( spun.lowerBound.y - 0.0f, 0.0001f );
	ENSURE_SMALL( spun.upperBound.y - 4.0f, 0.0001f );

	b2Segment segment = { { 0.0f, 1.0f }, { 0.0f, -1.0f } };
	b2AABB segmentBox = b2ComputeSegmentBounds( &segment, b2WorldTransform_identity );
	ENSURE_SMALL( segmentBox.lowerBound.x, FLT_EPSILON );
	ENSURE_SMALL( segmentBox.lowerBound.y + 1.0f, FLT_EPSILON );
	ENSURE_SMALL( segmentBox.upperBound.x, FLT_EPSILON );
	ENSURE_SMALL( segmentBox.upperBound.y - 1.0f, FLT_EPSILON );

	b2AABB segmentTurned = b2ComputeSegmentBounds( &segment, turned );
	ENSURE_SMALL( segmentTurned.lowerBound.x - 0.0f, 0.0001f );
	ENSURE_SMALL( segmentTurned.upperBound.x - 2.0f, 0.0001f );
	ENSURE_SMALL( segmentTurned.lowerBound.y - 2.0f, 0.0001f );
	ENSURE_SMALL( segmentTurned.upperBound.y - 2.0f, 0.0001f );
	return 0;
}

int PointInCircleTest( void )
{
	b2Circle circle = { { 1.0f, 0.0f }, 1.0f };
	b2Vec2 inside = { 0.5f, 0.5f };
	b2Vec2 outside = { 4.0f, -4.0f };
	b2Vec2 center = { 1.0f, 0.0f };
	b2Vec2 boundary = { 2.0f, 0.0f };
	b2Vec2 justOut = { 2.0f + 0.001f, 0.0f };
	ENSURE( b2PointInCircle( &circle, inside ) == true );
	ENSURE( b2PointInCircle( &circle, outside ) == false );
	ENSURE( b2PointInCircle( &circle, center ) == true );
	ENSURE( b2PointInCircle( &circle, boundary ) == true );
	ENSURE( b2PointInCircle( &circle, justOut ) == false );

	b2Circle dot = { { 3.0f, 4.0f }, 0.0f };
	b2Vec2 same = { 3.0f, 4.0f };
	b2Vec2 near = { 3.0f, 4.001f };
	ENSURE( b2PointInCircle( &dot, same ) == true );
	ENSURE( b2PointInCircle( &dot, near ) == false );
	return 0;
}
