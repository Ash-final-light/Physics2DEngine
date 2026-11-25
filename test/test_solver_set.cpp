// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"
#include "world.h"
#include "box2d/types.h"

int SolverSetOrganizationTest( void )
{
    size_t before=b2GetByteCount();
    b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;
    b2WorldId worldId=b2CreateWorld(&wd);
    b2World* world=b2_worlds+worldId.index1-1;
    b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;
    b2BodyId first=b2CreateBody(worldId,&bd);
    bd.position={10.0f,0.0f};b2BodyId moving=b2CreateBody(worldId,&bd);
    bd.type=b2_kinematicBody;bd.position={20.0f,0.0f};b2BodyId kinematic=b2CreateBody(worldId,&bd);
    bd.type=b2_staticBody;bd.position={10.0f,1.5f};b2BodyId fixed=b2CreateBody(worldId,&bd);
    bd.type=b2_dynamicBody;bd.isEnabled=false;b2BodyId disabled=b2CreateBody(worldId,&bd);
    bd.isEnabled=true;bd.isAwake=false;bd.position={30.0f,0.0f};b2BodyId sleeping=b2CreateBody(worldId,&bd);
    b2Body* sleepBody=world->bodies.data+sleeping.index1-1;
    int sleepingSet=sleepBody->setIndex;
    ENSURE(sleepingSet>=b2_firstSleepingSet);
    ENSURE(world->solverSets.data[b2_awakeSet].bodySims.count==3);
    ENSURE(world->solverSets.data[b2_awakeSet].bodyStates.count==3);
    ENSURE(world->solverSets.data[b2_staticSet].bodyStates.count==0);
    ENSURE(world->solverSets.data[b2_disabledSet].bodyStates.count==0);
    ENSURE(world->bodies.data[disabled.index1-1].setIndex==b2_disabledSet);
    ENSURE(b2GetBodyState(world,sleepBody)==NULL);
    b2ShapeDef sd=b2DefaultShapeDef();b2Circle circle={{0.0f,0.0f},1.0f};
    b2CreateCircleShape(moving,&sd,&circle);b2CreateCircleShape(fixed,&sd,&circle);
    b2UpdateBroadPhaseContacts(world);b2UpdateContacts(world);
    ENSURE(world->broadPhase.pairSet.count==1);
    b2Contact* contact=world->contacts.data;
    b2Body* body=world->bodies.data+moving.index1-1;
    ENSURE(b2GetContactSim(world,contact)->encodedBodySimA==body->localIndex);
    b2DestroyBody(first);
    for(int i=0;i<world->solverSets.data[b2_awakeSet].bodySims.count;++i)
    {
        int id=world->solverSets.data[b2_awakeSet].bodySims.data[i].bodyId;
        ENSURE(world->bodies.data[id].localIndex==i);
        ENSURE(world->bodies.data[id].setIndex==b2_awakeSet);
    }
    ENSURE(b2GetContactSim(world,contact)->encodedBodySimA==world->bodies.data[moving.index1-1].localIndex);
    ENSURE(b2Body_GetPosition(kinematic).x==20.0f && b2Body_GetPosition(moving).x==10.0f);
    b2Body_SetLinearVelocity(sleeping,{3.0f,0.0f});
    ENSURE(world->bodies.data[sleeping.index1-1].setIndex==b2_awakeSet);
    ENSURE(world->solverSets.data[sleepingSet].setIndex==B2_NULL_INDEX);
    ENSURE(b2Body_GetLinearVelocity(sleeping).x==3.0f);
    ENSURE(world->solverSets.data[b2_awakeSet].bodySims.count==3);
    b2World_Step(worldId,0.1f,1);
    ENSURE_SMALL(b2Body_GetPosition(sleeping).x-30.3f,0.00001f);
    b2DestroyWorld(worldId);ENSURE(b2GetByteCount()==before);return 0;
}
