// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "solver.h"
#include "world.h"
#include "shape.h"
#include "box2d/math_functions.h"

static void b2IntegrateBodyVelocity( b2World* world, b2Body* body, float h )
{
    float gravityScale = body->invMass > 0.0f ? body->gravityScale : 0.0f;
    b2Vec2 delta = b2MulSV( h*gravityScale, world->gravity );
    body->linearVelocity = b2Add( delta, body->linearVelocity );
}

void b2SolveFreeBodies( b2World* world, float timeStep, int subStepCount )
{
    float h = timeStep / (float)subStepCount;
    for ( int i = 0; i < world->bodies.count; ++i )
    {
        b2Body* body = world->bodies.data+i;
        if ( body->id == B2_NULL_INDEX || body->type == b2_staticBody || body->isEnabled == false || body->isAwake == false )
        {
            continue;
        }
        b2Vec2 deltaPosition = b2Vec2_zero;
        b2Rot deltaRotation = b2Rot_identity;
        for ( int step = 0; step < subStepCount; ++step )
        {
            b2IntegrateBodyVelocity( world, body, h );
            deltaPosition = b2MulAdd( deltaPosition, h, body->linearVelocity );
            deltaRotation = b2IntegrateRotation( deltaRotation, h*body->angularVelocity );
        }
        body->center = b2Add( body->center, deltaPosition );
        body->transform.q = b2NormalizeRot( b2MulRot( deltaRotation, body->transform.q ) );
        body->transform.p = b2Sub( body->center, b2RotateVector( body->transform.q, body->localCenter ) );
        for ( int shapeId = body->headShapeId; shapeId != B2_NULL_INDEX; shapeId = world->shapes.data[shapeId].nextShapeId )
        {
            b2MoveShapeProxy( world, world->shapes.data+shapeId, body->transform );
        }
    }
}
