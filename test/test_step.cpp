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


int BodyVelocityTest( void )
{
    b2WorldDef wd = b2DefaultWorldDef(); wd.gravity = b2Vec2_zero;
    b2WorldId world = b2CreateWorld( &wd );
    b2BodyDef bd = b2DefaultBodyDef(); bd.type = b2_dynamicBody;
    b2BodyId body = b2CreateBody( world, &bd );
    b2Body_SetLinearVelocity( body, { 4.0f, 2.0f } );
    b2Body_SetAngularVelocity( body, 0.5f );
    b2World_Step( world, 0.25f, 2 );
    ENSURE_SMALL( b2Body_GetPosition( body ).x-1.0f, 0.00001f );
    ENSURE( b2Body_GetLinearVelocity( body ).y == 2.0f && b2Body_GetAngularVelocity( body ) == 0.5f );
    b2Body_SetLinearVelocity( body, { NAN, 0.0f } );
    b2Body_SetAngularVelocity( body, INFINITY );
    ENSURE( b2Body_GetLinearVelocity( body ).x == 4.0f && b2Body_GetAngularVelocity( body ) == 0.5f );
    b2Body_SetTransform( body, { 3.0f, 4.0f }, { 0.0f, 1.0f } );
    b2ShapeDef sd = b2DefaultShapeDef();
    b2Circle circle = { { 1.0f, 0.0f }, 1.0f };
    b2CreateCircleShape( body, &sd, &circle );
    b2Body_SetLinearVelocity( body, { 2.0f, 3.0f } ); b2Body_SetAngularVelocity( body, 2.0f );
    b2Vec2 local = b2Body_GetLocalPointVelocity( body, { 2.0f, 0.0f } );
    b2Vec2 global = b2Body_GetWorldPointVelocity( body, { 3.0f, 6.0f } );
    ENSURE( local.x == 0.0f && local.y == 3.0f && global.x == local.x && global.y == local.y );
    bd.type = b2_staticBody;
    b2BodyId fixed = b2CreateBody( world, &bd );
    b2Body_SetLinearVelocity( fixed, { 1.0f, 2.0f } );
    ENSURE( b2Body_GetLinearVelocity( fixed ).x == 0.0f );
    b2DestroyWorld( world ); return 0;
}


int BodyGravityTest( void )
{
    b2WorldDef wd = b2DefaultWorldDef();
    b2WorldId world = b2CreateWorld( &wd );
    b2BodyDef bd = b2DefaultBodyDef(); bd.type = b2_dynamicBody;
    b2BodyId normal = b2CreateBody( world, &bd );
    b2BodyId scaled = b2CreateBody( world, &bd );
    b2BodyId massless = b2CreateBody( world, &bd );
    b2ShapeDef sd = b2DefaultShapeDef(); b2Circle circle = { { 0.0f, 0.0f }, 1.0f };
    b2CreateCircleShape( normal, &sd, &circle ); b2CreateCircleShape( scaled, &sd, &circle );
    b2Body_SetGravityScale( scaled, 2.0f ); b2Body_SetGravityScale( scaled, NAN );
    ENSURE( b2Body_GetGravityScale( scaled ) == 2.0f );
    b2World_Step( world, 0.5f, 4 );
    ENSURE_SMALL( b2Body_GetLinearVelocity( normal ).y+5.0f, 0.00001f );
    ENSURE_SMALL( b2Body_GetPosition( normal ).y+1.5625f, 0.00001f );
    ENSURE_SMALL( b2Body_GetLinearVelocity( scaled ).y+10.0f, 0.00001f );
    ENSURE( b2Body_GetLinearVelocity( massless ).y == 0.0f );
    b2Body_SetGravityScale( normal, 0.0f );
    b2World_Step( world, 0.1f, 2 );
    ENSURE_SMALL( b2Body_GetLinearVelocity( normal ).y+5.0f, 0.00001f );
    b2DestroyWorld( world ); return 0;
}


int BodyDampingTest( void )
{
    b2WorldDef wd = b2DefaultWorldDef(); wd.gravity = b2Vec2_zero;
    b2WorldId world = b2CreateWorld( &wd );
    b2BodyDef bd = b2DefaultBodyDef(); bd.type = b2_dynamicBody;
    bd.linearVelocity = { 10.0f, 0.0f }; bd.angularVelocity = 4.0f;
    b2BodyId body = b2CreateBody( world, &bd );
    b2Body_SetLinearDamping( body, 2.0f ); b2Body_SetAngularDamping( body, 3.0f );
    b2Body_SetLinearDamping( body, -1.0f ); b2Body_SetAngularDamping( body, NAN );
    ENSURE( b2Body_GetLinearDamping( body ) == 2.0f && b2Body_GetAngularDamping( body ) == 3.0f );
    b2World_Step( world, 0.5f, 1 );
    ENSURE_SMALL( b2Body_GetLinearVelocity( body ).x-5.0f, 0.00001f );
    ENSURE_SMALL( b2Body_GetAngularVelocity( body )-1.6f, 0.00001f );
    b2Body_SetLinearDamping( body, 0.0f ); b2Body_SetAngularDamping( body, 0.0f );
    b2World_Step( world, 0.25f, 2 );
    ENSURE_SMALL( b2Body_GetLinearVelocity( body ).x-5.0f, 0.00001f );
    ENSURE_SMALL( b2Body_GetAngularVelocity( body )-1.6f, 0.00001f );
    b2DestroyWorld( world ); return 0;
}
