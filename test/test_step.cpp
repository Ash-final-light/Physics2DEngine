// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"
#include "box2d/types.h"
#include "box2d/math_functions.h"
#include <math.h>

int FreeBodyStepTest( void )
{
    size_t before = b2GetByteCount();
    b2WorldDef wd = b2DefaultWorldDef();
    wd.gravity = b2Vec2_zero;
    b2WorldId world = b2CreateWorld( &wd );
    b2BodyDef bd = b2DefaultBodyDef();
    bd.type = b2_dynamicBody;
    bd.linearVelocity = { 2.0f, -3.0f };
    bd.angularVelocity = 0.4f;
    b2BodyId moving = b2CreateBody( world, &bd );
    bd.type = b2_staticBody;
    b2BodyId fixed = b2CreateBody( world, &bd );
    bd.type = b2_dynamicBody;
    bd.isEnabled = false;
    b2BodyId disabled = b2CreateBody( world, &bd );
    b2World_Step( world, 0.5f, 4 );
    b2Pos position = b2Body_GetPosition( moving );
    ENSURE_SMALL( position.x-1.0f, 0.00001f );
    ENSURE_SMALL( position.y+1.5f, 0.00001f );
    b2Rot rotation = b2Body_GetRotation( moving );
    ENSURE_SMALL( rotation.c*rotation.c+rotation.s*rotation.s-1.0f, 0.00001f );
    ENSURE_SMALL( rotation.s-sinf(0.2f), 0.001f );
    ENSURE( b2Body_GetPosition( fixed ).x == 0.0f && b2Body_GetPosition( disabled ).x == 0.0f );
    b2World_Step( world, 0.0f, 4 );
    b2World_Step( world, -1.0f, 4 );
    b2World_Step( world, NAN, 4 );
    b2World_Step( world, 1.0f, 0 );
    ENSURE( b2Body_GetPosition( moving ).x == position.x );
    b2World_Step( world, 0.5f, 1 );
    ENSURE_SMALL( b2Body_GetPosition( moving ).x-2.0f, 0.00001f );
    b2DestroyWorld( world );
    ENSURE( b2GetByteCount() == before );
    return 0;
}
