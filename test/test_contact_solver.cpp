// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
#include "test_macros.h"
#include "contact_solver.h"
#include "world.h"

int ContactPrepareTest(void)
{
    size_t before=b2GetByteCount();
    b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;
    b2WorldId id=b2CreateWorld(&wd);b2World* world=b2_worlds+id.index1-1;
    b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;
    b2BodyId body=b2CreateBody(id,&bd);b2ShapeDef sd=b2DefaultShapeDef();
    b2Polygon box=b2MakeBox(1,0.5f);b2CreatePolygonShape(body,&sd,&box);
    bd.type=b2_staticBody;bd.position={0,-0.9f};b2BodyId ground=b2CreateBody(id,&bd);
    b2Polygon floor=b2MakeBox(5,0.5f);b2CreatePolygonShape(ground,&sd,&floor);
    b2UpdateBroadPhaseContacts(world);b2UpdateContacts(world);
    b2Contact* contact=world->contacts.data;b2ContactSim* sim=b2GetContactSim(world,contact);
    ENSURE(sim->manifold.pointCount==2);
    // Edit mass after graph insertion, exercising current mass rather than stale cache.
    b2MassData mass={2,{0,0},4};b2Body_SetMassData(body,mass);
    b2Body_SetLinearVelocity(body,{1,-2});
    sim->manifold.points[0].normalImpulse=7;
    b2StepContext context={world,&world->constraintGraph,world->solverSets.data[b2_awakeSet].bodyStates.data,0.1f,10.0f};
    b2PrepareContactConstraints(&context);
    b2ContactConstraint* constraint=world->constraintGraph.colors[contact->colorIndex].overflowConstraints+contact->localIndex;
    ENSURE(constraint->indexA==world->bodies.data[body.index1-1].localIndex+1);
    ENSURE(constraint->indexB==0 && constraint->invMassB==0 && constraint->invIB==0);
    ENSURE(constraint->pointCount==2);
    for(int j=0;j<2;++j)
    {
        b2ContactConstraintPoint* cp=constraint->points+j;
        ENSURE_SMALL(cp->normalMass-4.0f/3.0f,0.00001f);
        float separation=cp->baseSeparation+b2Dot(b2Sub(cp->anchorB,cp->anchorA),constraint->normal);
        ENSURE_SMALL(separation-sim->manifold.points[j].separation,0.00001f);
        ENSURE(cp->normalImpulse==0 && cp->totalNormalImpulse==0);
    }
    ENSURE(sim->manifold.points[0].normalImpulse==7);
    ENSURE(b2Body_GetLinearVelocity(body).x==1 && b2Body_GetLinearVelocity(body).y==-2);
    b2ReleaseContactConstraints(&context);
    for(int i=0;i<B2_GRAPH_COLOR_COUNT;++i) { ENSURE(world->constraintGraph.colors[i].overflowConstraints==NULL); }
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}
