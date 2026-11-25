// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"
#include "world.h"
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
    bd.position.x=10.0f;
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


int BodyForceTest( void )
{
    b2WorldDef wd = b2DefaultWorldDef(); wd.gravity = b2Vec2_zero;
    b2WorldId world = b2CreateWorld( &wd );
    b2BodyDef bd = b2DefaultBodyDef(); bd.type = b2_dynamicBody;
    b2BodyId body = b2CreateBody( world, &bd );
    b2Body_SetMassData( body, b2MassData{ 2.0f, { 0.0f, 0.0f }, 4.0f } );
    b2Body_ApplyForceToCenter( body, { 2.0f, 0.0f }, true );
    b2Body_ApplyForceToCenter( body, { 2.0f, 0.0f }, true );
    b2World_Step( world, 0.5f, 1 );
    ENSURE_SMALL( b2Body_GetLinearVelocity( body ).x-1.0f, 0.00001f );
    ENSURE_SMALL( b2Body_GetPosition( body ).x-0.5f, 0.00001f );
    b2World_Step( world, 0.5f, 1 );
    ENSURE_SMALL( b2Body_GetLinearVelocity( body ).x-1.0f, 0.00001f );
    b2Body_ApplyForceToCenter( body, { 100.0f, 0.0f }, true );
    b2Body_ClearForces( body );
    b2World_Step( world, 0.1f, 1 );
    ENSURE_SMALL( b2Body_GetLinearVelocity( body ).x-1.0f, 0.00001f );
    b2BodyId offCenter = b2CreateBody( world, &bd );
    b2Body_SetMassData( offCenter, b2MassData{ 2.0f, { 0.0f, 0.0f }, 4.0f } );
    b2Body_ApplyForce( offCenter, { 4.0f, 0.0f }, { 0.0f, 1.0f }, true );
    b2Body_ApplyForce( offCenter, { NAN, 0.0f }, { 0.0f, 1.0f }, true );
    b2World_Step( world, 0.5f, 1 );
    ENSURE_SMALL( b2Body_GetLinearVelocity( offCenter ).x-1.0f, 0.00001f );
    ENSURE_SMALL( b2Body_GetAngularVelocity( offCenter )+0.5f, 0.00001f );
    b2DestroyWorld( world ); return 0;
}


int BodyTorqueTest( void )
{
    b2WorldDef wd = b2DefaultWorldDef(); wd.gravity = b2Vec2_zero;
    b2WorldId world = b2CreateWorld( &wd );
    b2BodyDef bd = b2DefaultBodyDef(); bd.type = b2_dynamicBody;
    b2BodyId body = b2CreateBody( world, &bd );
    b2Body_SetMassData( body, b2MassData{ 2.0f, { 0.0f, 0.0f }, 4.0f } );
    b2Body_ApplyTorque( body, 4.0f, true ); b2Body_ApplyTorque( body, 4.0f, true );
    b2Body_ApplyTorque( body, NAN, true );
    b2World_Step( world, 0.5f, 2 );
    ENSURE_SMALL( b2Body_GetAngularVelocity( body )-1.0f, 0.00001f );
    ENSURE( b2Body_GetLinearVelocity( body ).x == 0.0f );
    b2World_Step( world, 0.25f, 1 );
    ENSURE_SMALL( b2Body_GetAngularVelocity( body )-1.0f, 0.00001f );
    b2Body_ApplyTorque( body, 100.0f, true ); b2Body_ClearForces( body );
    b2World_Step( world, 0.1f, 1 );
    ENSURE_SMALL( b2Body_GetAngularVelocity( body )-1.0f, 0.00001f );
    bd.isAwake = false;
    b2BodyId asleep = b2CreateBody( world, &bd );
    b2Body_SetMassData( asleep, b2MassData{ 2.0f, { 0.0f, 0.0f }, 4.0f } );
    b2Body_ApplyTorque( asleep, 8.0f, false );
    b2World_Step( world, 0.5f, 1 );
    ENSURE( b2Body_GetAngularVelocity( asleep ) == 0.0f );
    b2Body_ApplyTorque( asleep, 8.0f, true );
    b2World_Step( world, 0.5f, 1 );
    ENSURE_SMALL( b2Body_GetAngularVelocity( asleep )-1.0f, 0.00001f );
    b2DestroyWorld( world ); return 0;
}


int BodyLinearImpulseTest( void )
{
    b2WorldDef wd = b2DefaultWorldDef(); wd.gravity = b2Vec2_zero;
    b2WorldId world = b2CreateWorld( &wd );
    b2BodyDef bd = b2DefaultBodyDef(); bd.type = b2_dynamicBody;
    b2BodyId body = b2CreateBody( world, &bd );
    b2Body_SetMassData( body, b2MassData{2.0f,{0.0f,0.0f},4.0f} );
    b2Body_ApplyLinearImpulse( body, {4.0f,0.0f}, {0.0f,1.0f}, true );
    ENSURE( b2Body_GetLinearVelocity(body).x == 2.0f && b2Body_GetAngularVelocity(body) == -1.0f );
    ENSURE( b2Body_GetPosition(body).x == 0.0f );
    b2Body_ApplyLinearImpulseToCenter( body, {2.0f,0.0f}, true );
    ENSURE( b2Body_GetLinearVelocity(body).x == 3.0f && b2Body_GetAngularVelocity(body) == -1.0f );
    b2Body_ApplyLinearImpulseToCenter( body, {NAN,0.0f}, true );
    ENSURE( b2Body_GetLinearVelocity(body).x == 3.0f );
    b2Body_ApplyLinearImpulseToCenter( body, {1000000.0f,0.0f}, true );
    ENSURE_SMALL( b2Length(b2Body_GetLinearVelocity(body))-wd.maximumLinearSpeed, 0.001f );
    b2DestroyWorld(world); return 0;
}


int BodyAngularImpulseTest( void )
{
    b2WorldDef wd = b2DefaultWorldDef(); wd.gravity = b2Vec2_zero;
    b2WorldId world = b2CreateWorld(&wd);
    b2BodyDef bd = b2DefaultBodyDef(); bd.type=b2_dynamicBody;
    b2BodyId body=b2CreateBody(world,&bd);
    b2Body_SetMassData(body,b2MassData{2.0f,{0.0f,0.0f},4.0f});
    b2Body_ApplyAngularImpulse(body,8.0f,true);
    ENSURE(b2Body_GetAngularVelocity(body)==2.0f);
    ENSURE(b2Body_GetLinearVelocity(body).x==0.0f);
    b2Body_ApplyAngularImpulse(body,NAN,true);
    ENSURE(b2Body_GetAngularVelocity(body)==2.0f);
    bd.motionLocks.angularZ=true;
    b2BodyId locked=b2CreateBody(world,&bd);
    b2Body_SetMassData(locked,b2MassData{2.0f,{0.0f,0.0f},4.0f});
    b2Body_ApplyAngularImpulse(locked,8.0f,true);
    ENSURE(b2Body_GetAngularVelocity(locked)==0.0f);
    b2DestroyWorld(world);return 0;
}


int BodyTargetTransformTest( void )
{
    b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;
    b2WorldId world=b2CreateWorld(&wd);
    b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;
    b2BodyId body=b2CreateBody(world,&bd);
    b2WorldTransform target={{2.0f,4.0f},b2Rot_identity};
    b2Body_SetTargetTransform(body,target,0.5f,true);
    ENSURE(b2Body_GetPosition(body).x==0.0f);
    ENSURE(b2Body_GetLinearVelocity(body).x==4.0f && b2Body_GetLinearVelocity(body).y==8.0f);
    b2World_Step(world,0.5f,4);
    ENSURE_SMALL(b2Body_GetPosition(body).x-2.0f,0.00001f);
    ENSURE_SMALL(b2Body_GetPosition(body).y-4.0f,0.00001f);
    target.q={0.0f,1.0f};
    b2Body_SetTargetTransform(body,target,2.0f,true);
    ENSURE_SMALL(b2Body_GetAngularVelocity(body)-0.25f*B2_PI,0.0001f);
    b2Body_SetTargetTransform(body,b2WorldTransform_identity,0.0f,true);
    ENSURE_SMALL(b2Body_GetAngularVelocity(body)-0.25f*B2_PI,0.0001f);
    bd.isAwake=false;
    b2BodyId asleep=b2CreateBody(world,&bd);
    target={{1.0f,0.0f},b2Rot_identity};
    b2Body_SetTargetTransform(asleep,target,1.0f,false);
    ENSURE(b2Body_GetLinearVelocity(asleep).x==0.0f);
    b2Body_SetTargetTransform(asleep,target,1.0f,true);
    ENSURE(b2Body_GetLinearVelocity(asleep).x==1.0f);
    b2DestroyWorld(world);return 0;
}


int BodyTypesTest( void )
{
    b2WorldDef wd=b2DefaultWorldDef();
    b2WorldId world=b2CreateWorld(&wd);
    b2BodyDef bd=b2DefaultBodyDef();bd.linearVelocity={2.0f,0.0f};
    b2ShapeDef sd=b2DefaultShapeDef();b2Circle circle={{0.0f,0.0f},1.0f};
    b2BodyId bodies[b2_bodyTypeCount];
    for(int type=0;type<b2_bodyTypeCount;++type)
    {
        bd.type=(b2BodyType)type;bd.position.x=10.0f*type;bodies[type]=b2CreateBody(world,&bd);
        b2CreateCircleShape(bodies[type],&sd,&circle);
        ENSURE(b2Body_GetType(bodies[type])==type);
    }
    ENSURE(b2Body_GetMass(bodies[b2_staticBody])==0.0f);
    ENSURE(b2Body_GetMass(bodies[b2_kinematicBody])==0.0f);
    ENSURE(b2Body_GetMass(bodies[b2_dynamicBody])>0.0f);
    b2Body_ApplyForceToCenter(bodies[b2_kinematicBody],{1000.0f,0.0f},true);
    b2Body_ApplyLinearImpulseToCenter(bodies[b2_staticBody],{1000.0f,0.0f},true);
    b2World_Step(world,1.0f,2);
    ENSURE(b2Body_GetPosition(bodies[b2_staticBody]).x==0.0f);
    ENSURE_SMALL(b2Body_GetPosition(bodies[b2_kinematicBody]).x-(10.0f*b2_kinematicBody+2.0f),0.00001f);
    ENSURE(b2Body_GetPosition(bodies[b2_kinematicBody]).y==0.0f);
    ENSURE_SMALL(b2Body_GetPosition(bodies[b2_dynamicBody]).y+7.5f,0.00001f);
    bd.type=(b2BodyType)b2_bodyTypeCount;
    ENSURE(b2CreateBody(world,&bd).index1==0);
    b2DestroyWorld(world);return 0;
}


int BodyTypeChangeTest( void )
{
    size_t before=b2GetByteCount();
    b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;
    b2WorldId worldId=b2CreateWorld(&wd);
    b2World* world=b2_worlds+worldId.index1-1;
    b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;
    b2BodyId body=b2CreateBody(worldId,&bd);
    b2ShapeDef sd=b2DefaultShapeDef();b2Circle circle={{0.0f,0.0f},1.0f};
    b2CreateCircleShape(body,&sd,&circle);
    b2Body_SetLinearVelocity(body,{3.0f,0.0f});
    b2Body_SetType(body,b2_staticBody);
    ENSURE(b2Body_GetType(body)==b2_staticBody && b2Body_GetMass(body)==0.0f);
    ENSURE(b2DynamicTree_GetProxyCount(world->broadPhase.trees+b2_dynamicBody)==0);
    ENSURE(b2DynamicTree_GetProxyCount(world->broadPhase.trees+b2_staticBody)==1);
    b2World_Step(worldId,0.1f,1);
    ENSURE(b2Body_GetPosition(body).x==0.0f);
    b2Body_SetType(body,b2_kinematicBody);
    ENSURE(b2Body_GetLinearVelocity(body).x==0.0f);
    b2Body_SetLinearVelocity(body,{4.0f,0.0f});
    b2World_Step(worldId,0.25f,1);
    ENSURE_SMALL(b2Body_GetPosition(body).x-1.0f,0.00001f);
    b2Body_SetType(body,b2_dynamicBody);
    ENSURE(b2Body_GetMass(body)>0.0f && b2Body_GetLinearVelocity(body).x==4.0f);
    ENSURE(b2DynamicTree_GetProxyCount(world->broadPhase.trees+b2_dynamicBody)==1);
    b2Body_SetType(body,(b2BodyType)99);
    ENSURE(b2Body_GetType(body)==b2_dynamicBody);
    bd.isEnabled=false;
    b2BodyId disabled=b2CreateBody(worldId,&bd);
    b2ShapeId shape=b2CreateCircleShape(disabled,&sd,&circle);
    b2Body_SetType(disabled,b2_kinematicBody);
    ENSURE(b2Body_GetType(disabled)==b2_kinematicBody);
    ENSURE(world->shapes.data[shape.index1-1].proxyKey==B2_NULL_INDEX);
    b2DestroyWorld(worldId);
    ENSURE(b2GetByteCount()==before);return 0;
}
