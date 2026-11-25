// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "solver_set.h"
#include "world.h"

int b2CreateSolverSet( b2World* world )
{
    int id=b2AllocId(&world->solverSetIdPool);
    if(id==world->solverSets.count)
    {
        b2Array_Push(world->solverSets,b2SolverSet{});
    }
    world->solverSets.data[id]=b2SolverSet{};
    world->solverSets.data[id].setIndex=id;
    return id;
}

void b2CreateSolverSets( b2World* world )
{
    world->solverSetIdPool=b2CreateIdPool();
    b2Array_Create(world->solverSets);
    for(int i=0;i<b2_firstSleepingSet;++i)
    {
        int id=b2CreateSolverSet(world);
        B2_ASSERT(id==i);
    }
}

void b2DestroySolverSet( b2World* world, int index )
{
    b2SolverSet* set=world->solverSets.data+index;
    b2Array_Destroy(set->bodySims);
    b2Array_Destroy(set->bodyStates);
    b2Array_Destroy(set->contactSims);
    set->setIndex=B2_NULL_INDEX;
    b2FreeId(&world->solverSetIdPool,index);
}

void b2DestroySolverSets( b2World* world )
{
    for(int i=0;i<world->solverSets.count;++i)
    {
        if(world->solverSets.data[i].setIndex!=B2_NULL_INDEX)
        {
            b2DestroySolverSet(world,i);
        }
    }
    b2Array_Destroy(world->solverSets);
    b2DestroyIdPool(&world->solverSetIdPool);
}

b2BodySim* b2GetBodySim( b2World* world, b2Body* body )
{
    return world->solverSets.data[body->setIndex].bodySims.data+body->localIndex;
}

b2BodyState* b2GetBodyState( b2World* world, b2Body* body )
{
    return body->setIndex==b2_awakeSet ? world->solverSets.data[b2_awakeSet].bodyStates.data+body->localIndex : NULL;
}

int b2EncodeBodySimIndex( const b2Body* body )
{
    if(body->setIndex==b2_awakeSet) { return body->localIndex; }
    if(body->setIndex==b2_staticSet) { return -(body->localIndex+2); }
    return B2_NULL_INDEX;
}

void b2SyncBodyFlags( b2World* world, b2Body* body )
{
    b2GetBodySim(world,body)->flags=body->flags;
    b2BodyState* state=b2GetBodyState(world,body);
    if(state!=NULL) { state->flags=body->flags; }
}

void b2RefreshContactSets( b2World* world )
{
    for(int i=0;i<world->contacts.count;++i)
    {
        b2Contact* contact=world->contacts.data+i;
        if(contact->contactId==B2_NULL_INDEX) { continue; }
        b2Body* a=world->bodies.data+contact->edges[0].bodyId;
        b2Body* b=world->bodies.data+contact->edges[1].bodyId;
        int targetIndex=(a->setIndex==b2_awakeSet || b->setIndex==b2_awakeSet) ? b2_awakeSet : b2_disabledSet;
        if(contact->setIndex!=targetIndex)
        {
            b2SolverSet* source=world->solverSets.data+contact->setIndex;
            b2SolverSet* target=world->solverSets.data+targetIndex;
            b2ContactSim copy=source->contactSims.data[contact->localIndex];
            int oldIndex=contact->localIndex;
            int moved=b2Array_RemoveSwap(source->contactSims,oldIndex);
            if(moved!=B2_NULL_INDEX)
            {
                world->contacts.data[source->contactSims.data[oldIndex].contactId].localIndex=oldIndex;
            }
            contact->setIndex=targetIndex;
            contact->localIndex=target->contactSims.count;
            b2Array_Push(target->contactSims,copy);
        }
        b2ContactSim* sim=b2GetContactSim(world,contact);
        sim->encodedBodySimA=b2EncodeBodySimIndex(a);
        sim->encodedBodySimB=b2EncodeBodySimIndex(b);
    }
}

void b2RemoveBodyFromSolverSet( b2World* world, b2Body* body )
{
    int setIndex=body->setIndex;
    int index=body->localIndex;
    b2SolverSet* set=world->solverSets.data+setIndex;
    int moved=b2Array_RemoveSwap(set->bodySims,index);
    if(setIndex==b2_awakeSet) { b2Array_RemoveSwap(set->bodyStates,index); }
    if(moved!=B2_NULL_INDEX)
    {
        world->bodies.data[set->bodySims.data[index].bodyId].localIndex=index;
    }
    if(setIndex>=b2_firstSleepingSet && set->bodySims.count==0 && set->contactSims.count==0)
    {
        b2DestroySolverSet(world,setIndex);
    }
    b2RefreshContactSets(world);
}

void b2TransferBody( b2World* world, b2SolverSet* target, b2SolverSet* source, b2Body* body )
{
    if(target==source) { return; }
    b2BodySim sim=source->bodySims.data[body->localIndex];
    b2BodyState state=b2_identityBodyState;
    if(source->setIndex==b2_awakeSet) { state=source->bodyStates.data[body->localIndex]; }
    int oldSet=body->setIndex;
    int oldIndex=body->localIndex;
    int newIndex=target->bodySims.count;
    b2Array_Push(target->bodySims,sim);
    if(target->setIndex==b2_awakeSet)
    {
        state.flags=body->flags;
        b2Array_Push(target->bodyStates,state);
    }
    int moved=b2Array_RemoveSwap(source->bodySims,oldIndex);
    if(oldSet==b2_awakeSet) { b2Array_RemoveSwap(source->bodyStates,oldIndex); }
    if(moved!=B2_NULL_INDEX)
    {
        world->bodies.data[source->bodySims.data[oldIndex].bodyId].localIndex=oldIndex;
    }
    body->setIndex=target->setIndex;
    body->localIndex=newIndex;
    if(oldSet>=b2_firstSleepingSet && source->bodySims.count==0 && source->contactSims.count==0)
    {
        b2DestroySolverSet(world,oldSet);
    }
    b2RefreshContactSets(world);
}

bool b2WakeBody( b2World* world, b2Body* body )
{
    if(body->setIndex<b2_firstSleepingSet) { return false; }
    b2TransferBody(world,world->solverSets.data+b2_awakeSet,world->solverSets.data+body->setIndex,body);
    return true;
}
