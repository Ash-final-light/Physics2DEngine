// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
#include "test_macros.h"
#include "world.h"

static int CheckGraph(b2World* w)
{
    int graphCount=0;
    for(int c=0;c<B2_GRAPH_COLOR_COUNT;++c)
    {
        b2GraphColor* color=w->constraintGraph.colors+c;
        b2DynamicArray<int> seen={};
        for(int i=0;i<color->contactSims.count;++i)
        {
            b2ContactSim* sim=color->contactSims.data+i;
            b2Contact* contact=w->contacts.data+sim->contactId;
            ENSURE(contact->colorIndex==c && contact->localIndex==i && contact->setIndex==b2_awakeSet);
            ENSURE(b2GetContactSim(w,contact)==sim && sim->manifold.pointCount>0);
            for(int e=0;e<2;++e)
            {
                b2Body* body=w->bodies.data+contact->edges[e].bodyId;
                ENSURE((e==0?sim->encodedBodySimA:sim->encodedBodySimB)==b2EncodeBodySimIndex(body));
                if(body->type!=b2_dynamicBody || c==B2_OVERFLOW_INDEX) { continue; }
                ENSURE(b2GetBit(&color->bodySet,body->id));
                for(int j=0;j<seen.count;++j) { ENSURE(seen.data[j]!=body->id); }
                b2Array_Push(seen,body->id);
            }
            ++graphCount;
        }
        for(int i=0;i<w->bodies.count && c!=B2_OVERFLOW_INDEX;++i)
        {
            bool found=false;for(int j=0;j<seen.count;++j) { found|=seen.data[j]==i; }
            ENSURE(b2GetBit(&color->bodySet,i)==found);
        }
        b2Array_Destroy(seen);
    }
    int touching=0;
    for(int i=0;i<w->contacts.count;++i)
    {
        b2Contact* contact=w->contacts.data+i;
        if(contact->contactId==B2_NULL_INDEX) { continue; }
        bool touch=(contact->flags&b2_contactTouchingFlag)!=0;
        ENSURE((contact->colorIndex!=B2_NULL_INDEX)==touch);touching+=touch?1:0;
    }
    ENSURE(graphCount==touching);return 0;
}

int ConstraintGraphTest(void)
{
    size_t before=b2GetByteCount();
    b2WorldDef wd=b2DefaultWorldDef();wd.gravity=b2Vec2_zero;wd.capacity.staticBodyCount=0;wd.capacity.dynamicBodyCount=0;
    b2WorldId id=b2CreateWorld(&wd);b2World* w=b2_worlds+id.index1-1;
    b2BodyDef bd=b2DefaultBodyDef();bd.type=b2_dynamicBody;
    b2ShapeDef sd=b2DefaultShapeDef();b2Circle circle={{0,0},10};
    b2BodyId center=b2CreateBody(id,&bd);b2CreateCircleShape(center,&sd,&circle);
    circle.radius=0.05f;
    for(int i=0;i<30;++i)
    {
        b2Rot q=b2MakeRot(2.0f*B2_PI*i/30.0f);bd.position={9.98f*q.c,9.98f*q.s};
        b2BodyId satellite=b2CreateBody(id,&bd);b2CreateCircleShape(satellite,&sd,&circle);
    }
    b2UpdateBroadPhaseContacts(w);b2UpdateContacts(w);
    ENSURE(w->broadPhase.pairSet.count==30);
    ENSURE(w->constraintGraph.colors[B2_OVERFLOW_INDEX].contactSims.count==30-B2_DYNAMIC_COLOR_COUNT);
    ENSURE(CheckGraph(w)==0);
    b2Contact* first=w->contacts.data;
    int oldColor=first->colorIndex;
    b2DestroyContact(w,first);
    ENSURE(!b2GetBit(&w->constraintGraph.colors[oldColor].bodySet,center.index1-1));
    ENSURE(CheckGraph(w)==0);
    // Move beyond the fat bounds and back so broad phase revisits the removed pair.
    b2Body_SetTransform(center,{20.0f,0},b2Rot_identity);
    b2Body_SetTransform(center,{0,0},b2Rot_identity);
    b2UpdateBroadPhaseContacts(w);b2UpdateContacts(w);ENSURE(CheckGraph(w)==0);
    ENSURE(w->broadPhase.pairSet.count==30);
    b2DestroyBody(center);ENSURE(CheckGraph(w)==0);
    for(int c=0;c<B2_GRAPH_COLOR_COUNT;++c) { ENSURE(w->constraintGraph.colors[c].contactSims.count==0); }
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);
    // Independent bodies sharing static ground may use the same color.
    id=b2CreateWorld(&wd);w=b2_worlds+id.index1-1;
    bd=b2DefaultBodyDef();bd.position={0,-1.5f};b2BodyId ground=b2CreateBody(id,&bd);
    b2Polygon floor=b2MakeBox(10,0.6f);b2CreatePolygonShape(ground,&sd,&floor);
    bd.type=b2_dynamicBody;bd.position={-3,0};circle.radius=1;
    b2BodyId a=b2CreateBody(id,&bd);b2CreateCircleShape(a,&sd,&circle);
    bd.position={3,0};b2BodyId b=b2CreateBody(id,&bd);b2CreateCircleShape(b,&sd,&circle);
    b2UpdateBroadPhaseContacts(w);b2UpdateContacts(w);
    ENSURE(w->constraintGraph.colors[B2_OVERFLOW_INDEX-1].contactSims.count==2);ENSURE(CheckGraph(w)==0);
    b2DestroyBody(a);ENSURE(CheckGraph(w)==0);
    b2Body_SetTransform(b,{3,0.53f},b2Rot_identity);b2UpdateContacts(w);
    ENSURE(CheckGraph(w)==0);ENSURE(w->constraintGraph.colors[B2_OVERFLOW_INDEX-1].contactSims.count==0);
    b2Body_SetTransform(b,{3,0},b2Rot_identity);b2UpdateContacts(w);
    ENSURE(CheckGraph(w)==0);ENSURE(w->constraintGraph.colors[B2_OVERFLOW_INDEX-1].contactSims.count==1);
    b2DestroyWorld(id);ENSURE(b2GetByteCount()==before);return 0;
}
