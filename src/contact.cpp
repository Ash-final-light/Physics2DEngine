// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "contact.h"
#include "world.h"
#include "island.h"
#include "body.h"
#include "shape.h"
#include "solver.h"
#include "parallel_for.h"
#include "simd.h"
#include <bit>

typedef b2LocalManifold b2ManifoldFcn( const b2Shape* shapeA, const b2Shape* shapeB, b2Transform xf, b2SimplexCache* cache );

struct b2ContactRegister
{
	b2ManifoldFcn* fcn;
	bool primary;
};

static b2LocalManifold b2CircleManifold( const b2Shape* shapeA, const b2Shape* shapeB, b2Transform xf, b2SimplexCache* cache )
{
	(void)cache;
	return b2CollideCircles( &shapeA->circle, &shapeB->circle, xf );
}

static b2LocalManifold b2CapsuleAndCircleManifold( const b2Shape* shapeA, const b2Shape* shapeB, b2Transform xf,
												   b2SimplexCache* cache )
{
	(void)cache;
	return b2CollideCapsuleAndCircle( &shapeA->capsule, &shapeB->circle, xf );
}

static b2LocalManifold b2CapsuleManifold( const b2Shape* shapeA, const b2Shape* shapeB, b2Transform xf, b2SimplexCache* cache )
{
	(void)cache;
	return b2CollideCapsules( &shapeA->capsule, &shapeB->capsule, xf );
}

static b2LocalManifold b2PolygonAndCircleManifold( const b2Shape* shapeA, const b2Shape* shapeB, b2Transform xf,
												   b2SimplexCache* cache )
{
	(void)cache;
	return b2CollidePolygonAndCircle( &shapeA->polygon, &shapeB->circle, xf );
}

static b2LocalManifold b2PolygonAndCapsuleManifold( const b2Shape* shapeA, const b2Shape* shapeB, b2Transform xf,
													b2SimplexCache* cache )
{
	(void)cache;
	return b2CollidePolygonAndCapsule( &shapeA->polygon, &shapeB->capsule, xf );
}

static b2LocalManifold b2PolygonManifold( const b2Shape* shapeA, const b2Shape* shapeB, b2Transform xf, b2SimplexCache* cache )
{
	(void)cache;
	return b2CollidePolygons( &shapeA->polygon, &shapeB->polygon, xf );
}

static b2LocalManifold b2SegmentAndCircleManifold( const b2Shape* shapeA, const b2Shape* shapeB, b2Transform xf,
												   b2SimplexCache* cache )
{
	(void)cache;
	return b2CollideSegmentAndCircle( &shapeA->segment, &shapeB->circle, xf );
}

static b2LocalManifold b2SegmentAndCapsuleManifold( const b2Shape* shapeA, const b2Shape* shapeB, b2Transform xf,
													b2SimplexCache* cache )
{
	(void)cache;
	return b2CollideSegmentAndCapsule( &shapeA->segment, &shapeB->capsule, xf );
}

static b2LocalManifold b2SegmentAndPolygonManifold( const b2Shape* shapeA, const b2Shape* shapeB, b2Transform xf,
													b2SimplexCache* cache )
{
	(void)cache;
	return b2CollideSegmentAndPolygon( &shapeA->segment, &shapeB->polygon, xf );
}

static b2LocalManifold b2ChainSegmentAndCircleManifold( const b2Shape* shapeA, const b2Shape* shapeB, b2Transform xf,
														b2SimplexCache* cache )
{
	(void)cache;
	return b2CollideChainSegmentAndCircle( &shapeA->chainSegment, &shapeB->circle, xf );
}

static b2LocalManifold b2ChainSegmentAndCapsuleManifold( const b2Shape* shapeA, const b2Shape* shapeB, b2Transform xf,
														 b2SimplexCache* cache )
{
	return b2CollideChainSegmentAndCapsule( &shapeA->chainSegment, &shapeB->capsule, xf, cache );
}

static b2LocalManifold b2ChainSegmentAndPolygonManifold( const b2Shape* shapeA, const b2Shape* shapeB, b2Transform xf,
														 b2SimplexCache* cache )
{
	return b2CollideChainSegmentAndPolygon( &shapeA->chainSegment, &shapeB->polygon, xf, cache );
}

// 支持 DLL 热重载。
static_assert( b2_circleShape == 0 && b2_capsuleShape == 1 && b2_segmentShape == 2 &&
               b2_polygonShape == 3 && b2_chainSegmentShape == 4 && b2_shapeTypeCount == 5 );
static constexpr b2ContactRegister b2_contactRegistry[b2_shapeTypeCount][b2_shapeTypeCount] = {
    { { .fcn = b2CircleManifold, .primary = true }, { .fcn = b2CapsuleAndCircleManifold, .primary = false }, { .fcn = b2SegmentAndCircleManifold, .primary = false }, { .fcn = b2PolygonAndCircleManifold, .primary = false }, { .fcn = b2ChainSegmentAndCircleManifold, .primary = false } },
    { { .fcn = b2CapsuleAndCircleManifold, .primary = true }, { .fcn = b2CapsuleManifold, .primary = true }, { .fcn = b2SegmentAndCapsuleManifold, .primary = false }, { .fcn = b2PolygonAndCapsuleManifold, .primary = false }, { .fcn = b2ChainSegmentAndCapsuleManifold, .primary = false } },
    { { .fcn = b2SegmentAndCircleManifold, .primary = true }, { .fcn = b2SegmentAndCapsuleManifold, .primary = true }, {}, { .fcn = b2SegmentAndPolygonManifold, .primary = true }, {} },
    { { .fcn = b2PolygonAndCircleManifold, .primary = true }, { .fcn = b2PolygonAndCapsuleManifold, .primary = true }, { .fcn = b2SegmentAndPolygonManifold, .primary = false }, { .fcn = b2PolygonManifold, .primary = true }, { .fcn = b2ChainSegmentAndPolygonManifold, .primary = false } },
    { { .fcn = b2ChainSegmentAndCircleManifold, .primary = true }, { .fcn = b2ChainSegmentAndCapsuleManifold, .primary = true }, {}, { .fcn = b2ChainSegmentAndPolygonManifold, .primary = true }, {} },
};

bool b2CanCollide( b2ShapeType a, b2ShapeType b )
{
    return b2_contactRegistry[a][b].fcn != NULL;
}

bool b2Contact_IsValid( b2ContactId id )
{
    if ( id.world0 >= B2_MAX_WORLDS )
    {
        return false;
    }
    b2World* world = b2_worlds+id.world0;
    if ( world->inUse == false || id.index1 < 1 || id.index1 > world->contacts.count )
    {
        return false;
    }
    const b2Contact* contact = world->contacts.data+id.index1-1;
    return contact->contactId == id.index1-1 && contact->generation == id.generation;
}

void b2CreateContact( b2World* world, b2Shape* a, b2Shape* b )
{
    if ( b2CanCollide(a->type,b->type) == false )
    {
        return;
    }
    if ( b2_contactRegistry[a->type][b->type].primary == false )
    {
        b2CreateContact(world,b,a);
        return;
    }
    int id = b2AllocId(&world->contactIdPool);
    if ( id == world->contacts.count )
    {
        b2Array_Push(world->contacts,b2Contact{});
    }
    b2Contact* contact = world->contacts.data+id;
    uint32_t generation = contact->generation+1;
    *contact = b2Contact{};
    contact->contactId = id;
    contact->generation = generation;
    contact->shapeIdA = a->id;
    contact->shapeIdB = b->id;
    contact->islandId = B2_NULL_INDEX;
    contact->islandIndex = B2_NULL_INDEX;
    b2Body* bodyA=world->bodies.data+a->bodyId;
    b2Body* bodyB=world->bodies.data+b->bodyId;
    contact->setIndex=(bodyA->setIndex==b2_awakeSet || bodyB->setIndex==b2_awakeSet)?b2_awakeSet:b2_disabledSet;
    if((bodyA->flags&b2_bodyEnableContactRecycling)!=0 && (bodyB->flags&b2_bodyEnableContactRecycling)!=0 && !a->enablePreSolveEvents && !b->enablePreSolveEvents)
    { contact->flags|=b2_contactRecycleFlag; }
    if(a->enableContactEvents || b->enableContactEvents) { contact->flags|=b2_contactEnableContactEvents; }
    contact->colorIndex = B2_NULL_INDEX;
    b2SolverSet* set=world->solverSets.data+contact->setIndex;
    contact->localIndex=set->contactSims.count;
    b2ContactSim* sim=b2Array_Emplace(set->contactSims);
    *sim = b2ContactSim{};
    sim->contactId = id;
    sim->simFlags=contact->flags;
    if(a->enablePreSolveEvents || b->enablePreSolveEvents) { sim->simFlags|=b2_simEnablePreSolveEvents; }
    sim->shapeIdA = a->id;
    sim->shapeIdB = b->id;
#if B2_ENABLE_VALIDATION
    sim->bodyIdA=a->bodyId;
    sim->bodyIdB=b->bodyId;
#endif
    sim->encodedBodySimA=b2EncodeBodySimIndex(bodyA);
    sim->encodedBodySimB=b2EncodeBodySimIndex(bodyB);
    int bodyIds[2] = { a->bodyId,b->bodyId };
    for ( int edge = 0; edge < 2; ++edge )
    {
        b2Body* body = world->bodies.data+bodyIds[edge];
        b2ContactEdge* link = contact->edges+edge;
        link->bodyId = body->id;
        link->prevKey = B2_NULL_INDEX;
        link->nextKey = body->headContactKey;
        int key = (id<<1)|edge;
        if ( body->headContactKey != B2_NULL_INDEX )
        {
            b2Contact* head = world->contacts.data+(body->headContactKey>>1);
            head->edges[body->headContactKey&1].prevKey = key;
        }
        body->headContactKey = key;
        body->contactCount += 1;
    }
    uint64_t key = B2_SHAPE_PAIR_KEY(a->id,b->id);
    B2_ASSERT(b2ContainsKey(&world->broadPhase.pairSet,key)==false);
    b2AddKey(&world->broadPhase.pairSet,key);
}

static void b2EmitContactEnd(b2World* world,const b2Contact* contact)
{
    if((contact->flags&b2_contactEnableContactEvents)==0) { return; }
    const b2Shape* a=world->shapes.data+contact->shapeIdA;const b2Shape* b=world->shapes.data+contact->shapeIdB;
    b2ContactEndTouchEvent event={b2ShapeId{a->id+1,world->worldId,a->generation},b2ShapeId{b->id+1,world->worldId,b->generation},
        b2ContactId{contact->contactId+1,world->worldId,0,contact->generation}};
    b2Array_Push(world->contactEndEvents[world->endEventArrayIndex],event);
}

void b2DestroyContact( b2World* world, b2Contact* contact )
{
    bool touching=(contact->flags&b2_contactTouchingFlag)!=0;
    if(touching) { b2EmitContactEnd(world,contact); }
    b2Body* bodyA=world->bodies.data+contact->edges[0].bodyId;
    b2Body* bodyB=world->bodies.data+contact->edges[1].bodyId;
    if(contact->islandId!=B2_NULL_INDEX) { b2UnlinkContact(world,contact); }
    uint64_t key = B2_SHAPE_PAIR_KEY(contact->shapeIdA,contact->shapeIdB);
    b2RemoveKey(&world->broadPhase.pairSet,key);
    for ( int edge = 0; edge < 2; ++edge )
    {
        b2ContactEdge* link = contact->edges+edge;
        b2Body* body = world->bodies.data+link->bodyId;
        if ( link->prevKey != B2_NULL_INDEX )
        {
            world->contacts.data[link->prevKey>>1].edges[link->prevKey&1].nextKey = link->nextKey;
        }
        if ( link->nextKey != B2_NULL_INDEX )
        {
            world->contacts.data[link->nextKey>>1].edges[link->nextKey&1].prevKey = link->prevKey;
        }
        if ( body->headContactKey == ((contact->contactId<<1)|edge) )
        {
            body->headContactKey = link->nextKey;
        }
        body->contactCount -= 1;
    }
    if(contact->colorIndex!=B2_NULL_INDEX)
    {
        b2RemoveContactFromGraph(world,contact->edges[0].bodyId,contact->edges[1].bodyId,contact->colorIndex,contact->localIndex);
    }
    else
    {
        b2SolverSet* set=world->solverSets.data+contact->setIndex;
        int index=contact->localIndex;
        int moved=b2Array_RemoveSwap(set->contactSims,index);
        if(moved!=B2_NULL_INDEX)
        { world->contacts.data[set->contactSims.data[index].contactId].localIndex=index; }
    }
    b2FreeId(&world->contactIdPool,contact->contactId);
    uint32_t generation = contact->generation;
    *contact = b2Contact{};
    contact->contactId = B2_NULL_INDEX;
    contact->generation = generation;
    if(touching) { b2WakeBody(world,bodyA);b2WakeBody(world,bodyB); }
}

void b2DestroyBodyContacts( b2World* world, b2Body* body )
{
    while ( body->headContactKey != B2_NULL_INDEX )
    {
        b2DestroyContact(world,world->contacts.data+(body->headContactKey>>1));
    }
}

void b2DestroyShapeContacts( b2World* world, const b2Shape* shape )
{
    int key = world->bodies.data[shape->bodyId].headContactKey;
    while ( key != B2_NULL_INDEX )
    {
        b2Contact* contact = world->contacts.data+(key>>1);
        key = contact->edges[key&1].nextKey;
        if ( contact->shapeIdA == shape->id || contact->shapeIdB == shape->id )
        {
            b2DestroyContact(world,contact);
        }
    }
}

void b2UpdateBroadPhaseContacts( b2World* world )
{
    for ( int i = 0; i < world->contacts.count; ++i )
    {
        b2Contact* contact = world->contacts.data+i;
        if ( contact->contactId != B2_NULL_INDEX &&
             b2AABB_Overlaps(world->fatAABBs.data[contact->shapeIdA],world->fatAABBs.data[contact->shapeIdB]) == false )
        {
            b2DestroyContact(world,contact);
        }
    }
    b2DynamicArray<uint64_t> keys = {};
    b2CollectCandidatePairs(world,&keys);
    for ( int i = 0; i < keys.count; ++i )
    {
        int a = (int)(keys.data[i]>>32);
        int b = (int)(keys.data[i]&0xFFFFFFFF);
        b2CreateContact(world,world->shapes.data+a,world->shapes.data+b);
    }
    b2Array_Destroy(keys);
}


b2ContactSim* b2GetContactSim( b2World* world, b2Contact* contact )
{
    if(contact->colorIndex!=B2_NULL_INDEX)
    { return world->constraintGraph.colors[contact->colorIndex].contactSims.data+contact->localIndex; }
    return world->solverSets.data[contact->setIndex].contactSims.data+contact->localIndex;
}

bool b2UpdateContact( b2World* world, b2ContactSim* sim, b2Shape* a, b2WorldTransform transformA,
                      b2Vec2 centerOffsetA, b2Shape* b, b2WorldTransform transformB, b2Vec2 centerOffsetB )
{
    sim->friction=world->frictionCallback==NULL?sqrtf(a->material.friction*b->material.friction):
        world->frictionCallback(a->material.friction,a->material.userMaterialId,b->material.friction,b->material.userMaterialId);
    sim->restitution=world->restitutionCallback==NULL?b2MaxFloat(a->material.restitution,b->material.restitution):
        world->restitutionCallback(a->material.restitution,a->material.userMaterialId,b->material.restitution,b->material.userMaterialId);
    sim->tangentSpeed=a->material.tangentSpeed+b->material.tangentSpeed;
    sim->rollingResistance=0.0f;
    if(a->material.rollingResistance>0 || b->material.rollingResistance>0)
    {
        float maxRadius=b2MaxFloat(b2GetShapeRadius(a),b2GetShapeRadius(b));
        sim->rollingResistance=b2MaxFloat(a->material.rollingResistance,b->material.rollingResistance)*maxRadius;
    }
    b2Manifold old = sim->manifold;
    b2Transform relative = b2InvMulWorldTransforms(transformA,transformB);
    b2ManifoldFcn* function = b2_contactRegistry[a->type][b->type].fcn;
    b2LocalManifold local = function(a,b,relative,&sim->cache);
    sim->manifold = b2Manifold{};
    sim->manifold.normal = b2RotateVector(transformA.q,local.normal);
    sim->manifold.pointCount = local.pointCount;
    b2Vec2 originDelta = b2SubPos(transformA.p,transformB.p);
    for ( int i = 0; i < local.pointCount; ++i )
    {
        b2ManifoldPoint* point = sim->manifold.points+i;
        point->anchorA = b2RotateVector(transformA.q,local.points[i].point);
        point->anchorB = b2Add(b2RotateVector(transformA.q,local.points[i].point),originDelta);
        point->separation = local.points[i].separation;
        point->id = local.points[i].id;
    }
    if(sim->manifold.pointCount>0 && world->preSolveFcn!=NULL && (sim->simFlags&b2_simEnablePreSolveEvents)!=0)
    {
        b2ShapeId idA={a->id+1,world->worldId,a->generation},idB={b->id+1,world->worldId,b->generation};
        world->preSolveFcn(idA,idB,&sim->manifold,world->preSolveContext);
        if(sim->manifold.pointCount==0) { sim->simFlags&=~b2_simRelativeTransformValid; }
    }
    if ( sim->manifold.pointCount > 0 )
    {
        sim->manifold.rollingImpulse = old.rollingImpulse;
        for ( int i = 0; i < sim->manifold.pointCount; ++i )
        {
            b2ManifoldPoint* point = sim->manifold.points+i;
            point->anchorA=b2Sub(point->anchorA,centerOffsetA);point->anchorB=b2Sub(point->anchorB,centerOffsetB);
            for ( int j = 0; j < old.pointCount; ++j )
            {
                if ( old.points[j].id == point->id )
                {
                    point->normalImpulse = old.points[j].normalImpulse;
                    point->tangentImpulse = old.points[j].tangentImpulse;
                    point->persisted = true;
                    break;
                }
            }
        }
    }
    bool touching = sim->manifold.pointCount > 0;
    if(touching && (a->enableHitEvents || b->enableHitEvents)) { sim->simFlags|=b2_simEnableHitEvent; }
    else { sim->simFlags&=~b2_simEnableHitEvent; }
    if ( touching ) { sim->simFlags |= b2_simTouchingFlag; }
    else { sim->simFlags &= ~b2_simTouchingFlag; }
    return touching;
}

static inline float b2RelativeCos(b2Rot a,b2Rot b)
{ return a.c*b.c+a.s*b.s; }

static bool b2TryRecycleContact(b2World* world,b2ContactSim* contactSim,b2BodySim* bodySimA,b2BodySim* bodySimB,bool wasTouching)
{
    float recycleDistance=world->contactRecycleDistance;
    float recycleDistanceNonTouching=b2MinFloat(recycleDistance,B2_SPECULATIVE_DISTANCE);
    b2WorldTransform transformA=bodySimA->transform,transformB=bodySimB->transform;
    int encodedA=contactSim->encodedBodySimA,encodedB=contactSim->encodedBodySimB;
    bool hadTimeOfImpact=((bodySimA->flags|bodySimB->flags)&b2_hadTimeOfImpact)!=0;
// 接触回收优化。若使用此优化请引用此代码。
// 灵感来自某些物理引擎（如 PhysX）使用的持久接触流形。
// 但此方案允许更大的相对运动，调参更少（只有一个）。
if ( ( hadTimeOfImpact == false || wasTouching ) && recycleDistance > 0.0f &&
	 ( contactSim->simFlags & b2_simRelativeTransformValid ) &&
	 ( contactSim->simFlags & b2_contactRecycleFlag ) )
{
	b2Rot cachedQA = contactSim->cachedRotationA;
	b2Rot cachedQB = contactSim->cachedRotationB;
	b2Transform xfc = contactSim->cachedRelativePose;
	b2Transform xf = b2InvMulWorldTransforms( transformA, transformB );

	float cosA = b2RelativeCos( transformA.q, cachedQA );
	float cosB = b2RelativeCos( transformB.q, cachedQB );
	float minCos = b2MinFloat( cosA, cosB );

	float maxExtentA = ( encodedA <= -2 ) ? 0.0f : bodySimA->maxExtent;
	float maxExtentB = ( encodedB <= -2 ) ? 0.0f : bodySimB->maxExtent;
	float maxExtent = b2MaxFloat( maxExtentA, maxExtentB );
	float distance = b2Distance( xf.p, xfc.p );
	b2Rot qr = b2InvMulRot( xf.q, xfc.q );

	// 此度量用于快速刚体和休眠。来自保守推进。
	// 注意小角度时 qr.s == sin(theta) ~= theta。非接触形状需要更紧的容差，以免漏掉接触。
	float tolerance = wasTouching ? recycleDistance : recycleDistanceNonTouching;

	if ( minCos > B2_CONTACT_RECYCLE_COS_ANGLE && distance + maxExtent * b2AbsFloat( qr.s ) < tolerance )
	{
		b2Rot dqA = b2MulRot( transformA.q, b2InvertRot( cachedQA ) );
		b2Rot dqB = b2MulRot( transformB.q, b2InvertRot( cachedQB ) );
		b2Vec2 normal = contactSim->manifold.normal;

		// 尽量减少舍入
		b2Vec2 dc = b2SubPos( bodySimB->center, bodySimA->center );

		for ( int i = 0; i < contactSim->manifold.pointCount; ++i )
		{
			// 保留锚点但更新分离，与子步相同。这可消除抖动。
			b2ManifoldPoint* mp = contactSim->manifold.points + i;
			b2Vec2 rA = b2RotateVector( dqA, mp->anchorA );
			b2Vec2 rB = b2RotateVector( dqB, mp->anchorB );
			b2Vec2 dp = b2Add( dc, b2Sub( rB, rA ) );

			mp->separation = mp->baseSeparation + b2Dot( dp, normal );
			mp->normalVelocity = 0.0f;
			mp->persisted = true;
		}

		// 接触被回收。这也跳过更新接触的其他方面，如材质参数。
		return true;
	}
}

    return false;
}

void b2UpdateContacts( b2World* world )
{
    world->recycledContactCount=0;
    for ( int i = 0; i < world->contacts.count; ++i )
    {
        b2Contact* contact = world->contacts.data+i;
        if ( contact->contactId == B2_NULL_INDEX || contact->setIndex != b2_awakeSet )
        {
            continue;
        }
        b2Shape* a = world->shapes.data+contact->shapeIdA;
        b2Shape* b = world->shapes.data+contact->shapeIdB;
        b2Body* bodyA = world->bodies.data+a->bodyId;
        b2Body* bodyB = world->bodies.data+b->bodyId;
        bool wasTouching=(contact->flags & b2_contactTouchingFlag)!=0;
        b2ContactSim* current=b2GetContactSim(world,contact);
        b2BodySim* simA=b2GetBodySim(world,bodyA);b2BodySim* simB=b2GetBodySim(world,bodyB);
        current->invMassA=simA->invMass;current->invIA=simA->invInertia;
        current->invMassB=simB->invMass;current->invIB=simB->invInertia;
        if(b2TryRecycleContact(world,current,simA,simB,wasTouching)) { ++world->recycledContactCount;continue; }
        current->cachedRotationA=simA->transform.q;current->cachedRotationB=simB->transform.q;
        current->cachedRelativePose=b2InvMulWorldTransforms(simA->transform,simB->transform);
        current->simFlags|=b2_simRelativeTransformValid;
        bool touching = b2UpdateContact(world,b2GetContactSim(world,contact),a,b2GetBodySim(world,bodyA)->transform,
            b2SubPos(b2GetBodySim(world,bodyA)->center,b2GetBodySim(world,bodyA)->transform.p),b,b2GetBodySim(world,bodyB)->transform,b2SubPos(b2GetBodySim(world,bodyB)->center,b2GetBodySim(world,bodyB)->transform.p));
        for(int p=0;p<current->manifold.pointCount;++p) { current->manifold.points[p].baseSeparation=current->manifold.points[p].separation; }
        if ( touching ) { contact->flags |= b2_contactTouchingFlag; }
        else { contact->flags &= ~b2_contactTouchingFlag; }
        if(touching && !wasTouching)
        {
            if(contact->flags&b2_contactEnableContactEvents)
            {
                b2ContactBeginTouchEvent event={b2ShapeId{a->id+1,world->worldId,a->generation},b2ShapeId{b->id+1,world->worldId,b->generation},
                    b2ContactId{contact->contactId+1,world->worldId,0,contact->generation}};
                b2Array_Push(world->contactBeginEvents,event);
            }
            b2LinkContact(world,contact);
            // Linking can wake an entire set and relocate contact arrays.
            b2ContactSim copy=*b2GetContactSim(world,contact);
            int oldIndex=contact->localIndex;
            b2SolverSet* set=world->solverSets.data+contact->setIndex;
            b2AddContactToGraph(world,&copy,contact);
            int moved=b2Array_RemoveSwap(set->contactSims,oldIndex);
            if(moved!=B2_NULL_INDEX)
            { world->contacts.data[set->contactSims.data[oldIndex].contactId].localIndex=oldIndex; }
        }
        else if(!touching && wasTouching)
        {
            b2EmitContactEnd(world,contact);
            b2UnlinkContact(world,contact);
            b2ContactSim copy=*b2GetContactSim(world,contact);
            b2RemoveContactFromGraph(world,bodyA->id,bodyB->id,contact->colorIndex,contact->localIndex);
            b2SolverSet* set=world->solverSets.data+contact->setIndex;
            contact->colorIndex=B2_NULL_INDEX;contact->localIndex=set->contactSims.count;
            b2Array_Push(set->contactSims,copy);
        }
    }
}


b2ContactData b2Contact_GetData( b2ContactId id )
{
    B2_ASSERT(b2Contact_IsValid(id));
    b2World* world=b2_worlds+id.world0;
    b2Contact* contact=world->contacts.data+id.index1-1;
    const b2Shape* a=world->shapes.data+contact->shapeIdA;
    const b2Shape* b=world->shapes.data+contact->shapeIdB;
    return b2ContactData{ id,
        b2ShapeId{a->id+1,id.world0,a->generation},
        b2ShapeId{b->id+1,id.world0,b->generation},
        b2GetContactSim(world,contact)->manifold };
}

static void b2CollideTask( int startIndex, int endIndex, int workerIndex, void* context )
{


	b2StepContext* stepContext = static_cast<b2StepContext *>( context );
	b2World* world = stepContext->world;
	b2TaskContext* taskContext = world->taskContexts.data + workerIndex;
	const b2ContactCollideSpan* spans = stepContext->collideSpans;
	b2Shape* shapes = world->shapes.data;
	const b2AABB* fatAABBs = world->fatAABBs.data;
	b2SolverSet* solverSets = world->solverSets.data;
	b2BodySim* awakeSims = solverSets[b2_awakeSet].bodySims.data;
	b2BodySim* staticSims = solverSets[b2_staticSet].bodySims.data;

	B2_ASSERT( startIndex < endIndex );

	int spanIndex = 0;
	while ( spans[spanIndex + 1].start <= startIndex )
	{
		spanIndex += 1;
	}

	int spanStart = spans[spanIndex].start;
	int spanEnd = spans[spanIndex + 1].start;
	b2ContactSim* spanBase = spans[spanIndex].contacts;

	float recycleDistance = world->contactRecycleDistance;
	float speculativeDistance = B2_SPECULATIVE_DISTANCE;
	float recycleDistanceNonTouching = b2MinFloat( recycleDistance, speculativeDistance );

	for ( int contactIndex = startIndex; contactIndex < endIndex; ++contactIndex )
	{
		if ( contactIndex == spanEnd )
		{
			spanIndex += 1;
			spanStart = spans[spanIndex].start;
			spanEnd = spans[spanIndex + 1].start;
			spanBase = spans[spanIndex].contacts;
		}

		b2ContactSim* contactSim = spanBase + ( contactIndex - spanStart );

		int contactId = contactSim->contactId;
		int shapeIdA = contactSim->shapeIdA;
		int shapeIdB = contactSim->shapeIdB;

		// 代理是否仍重叠？
		bool overlap = b2OverlapV( fatAABBs + shapeIdA, fatAABBs + shapeIdB );
		if ( overlap == false )
		{
			contactSim->simFlags |= b2_simDisjoint;
			contactSim->simFlags &= ~b2_simTouchingFlag;
			b2SetBit( &taskContext->contactStateBitSet, contactId );
		}
		else
		{
			b2Shape* shapeA = shapes + shapeIdA;
			b2Shape* shapeB = shapes + shapeIdB;

			bool wasTouching = ( contactSim->simFlags & b2_simTouchingFlag );

			// 按形状/刚体顺序（A,B）更新接触
			int encodedA = contactSim->encodedBodySimA;
			int encodedB = contactSim->encodedBodySimB;
			b2BodySim* bodySimA = b2GetBodySim(world,world->bodies.data+shapeA->bodyId);
			b2BodySim* bodySimB = b2GetBodySim(world,world->bodies.data+shapeB->bodyId);
			b2WorldTransform transformA = bodySimA->transform;
			b2WorldTransform transformB = bodySimB->transform;

			// 下面的相对变换检查可能不会跳过这些
			contactSim->invMassA = bodySimA->invMass;
			contactSim->invIA = bodySimA->invInertia;
			contactSim->invMassB = bodySimB->invMass;
			contactSim->invIB = bodySimB->invInertia;

			bool hadTimeOfImpact = ( ( bodySimA->flags | bodySimB->flags ) & b2_hadTimeOfImpact ) != 0;

			// 接触回收优化。若使用此优化请引用此代码。
			// 灵感来自某些物理引擎（如 PhysX）使用的持久接触流形。
			// 但此方案允许更大的相对运动，调参更少（只有一个）。
			if ( ( hadTimeOfImpact == false || wasTouching ) && recycleDistance > 0.0f &&
				 ( contactSim->simFlags & b2_simRelativeTransformValid ) &&
				 ( contactSim->simFlags & b2_contactRecycleFlag ) )
			{
				b2Rot cachedQA = contactSim->cachedRotationA;
				b2Rot cachedQB = contactSim->cachedRotationB;
				b2Transform xfc = contactSim->cachedRelativePose;
				b2Transform xf = b2InvMulWorldTransforms( transformA, transformB );

				float cosA = b2RelativeCos( transformA.q, cachedQA );
				float cosB = b2RelativeCos( transformB.q, cachedQB );
				float minCos = b2MinFloat( cosA, cosB );

				float maxExtentA = world->bodies.data[shapeA->bodyId].type==b2_staticBody ? 0.0f : bodySimA->maxExtent;
				float maxExtentB = world->bodies.data[shapeB->bodyId].type==b2_staticBody ? 0.0f : bodySimB->maxExtent;
				float maxExtent = b2MaxFloat( maxExtentA, maxExtentB );
				float distance = b2Distance( xf.p, xfc.p );
				b2Rot qr = b2InvMulRot( xf.q, xfc.q );

				// 此度量用于快速刚体和休眠。来自保守推进。
				// 注意小角度时 qr.s == sin(theta) ~= theta。非接触形状需要更紧的容差，以免漏掉接触。
				float tolerance = wasTouching ? recycleDistance : recycleDistanceNonTouching;

				if ( minCos > B2_CONTACT_RECYCLE_COS_ANGLE && distance + maxExtent * b2AbsFloat( qr.s ) < tolerance )
				{
					b2Rot dqA = b2MulRot( transformA.q, b2InvertRot( cachedQA ) );
					b2Rot dqB = b2MulRot( transformB.q, b2InvertRot( cachedQB ) );
					b2Vec2 normal = contactSim->manifold.normal;

					// 尽量减少舍入
					b2Vec2 dc = b2SubPos( bodySimB->center, bodySimA->center );

					for ( int i = 0; i < contactSim->manifold.pointCount; ++i )
					{
						// 保留锚点但更新分离，与子步相同。这可消除抖动。
						b2ManifoldPoint* mp = contactSim->manifold.points + i;
						b2Vec2 rA = b2RotateVector( dqA, mp->anchorA );
						b2Vec2 rB = b2RotateVector( dqB, mp->anchorB );
						b2Vec2 dp = b2Add( dc, b2Sub( rB, rA ) );

						mp->separation = mp->baseSeparation + b2Dot( dp, normal );
						mp->normalVelocity = 0.0f;
						mp->persisted = true;
					}

					taskContext->recycledContactCount += 1;

					// 接触被回收。这也跳过更新接触的其他方面，如材质参数。
					continue;
				}
			}

			// 接触回收的缓存。
			contactSim->cachedRotationA = transformA.q;
			contactSim->cachedRotationB = transformB.q;
			contactSim->cachedRelativePose = b2InvMulWorldTransforms( transformA, transformB );
			contactSim->simFlags |= b2_simRelativeTransformValid;

			b2Vec2 centerOffsetA = b2RotateVector( transformA.q, bodySimA->localCenter );
			b2Vec2 centerOffsetB = b2RotateVector( transformB.q, bodySimB->localCenter );

			// 这会更新实体接触
			bool touching = b2UpdateContact( world, contactSim, shapeA, transformA, centerOffsetA, shapeB, transformB,
											 centerOffsetB );

			// 影响岛连通性的状态变化。也影响接触事件。
			if ( touching == true && wasTouching == false )
			{
				contactSim->simFlags |= b2_simStartedTouching;
				b2SetBit( &taskContext->contactStateBitSet, contactId );
			}
			else if ( touching == false && wasTouching == true )
			{
				contactSim->simFlags |= b2_simStoppedTouching;
				b2SetBit( &taskContext->contactStateBitSet, contactId );
			}

			if ( touching )
			{
				for ( int i = 0; i < contactSim->manifold.pointCount; ++i )
				{
					// 缓存分离
					b2ManifoldPoint* mp = contactSim->manifold.points + i;
					mp->baseSeparation = mp->separation;
				}
			}

			// 要使这生效，撞击时刻代码需要根据刚体的 TOI 事件数调整目标距离。
			// if (touching && bodySimB->isFast)
			//{
			//	b2Manifold* manifold = &contactSim->manifold;
			//	int pointCount = manifold->pointCount;
			//	for (int i = 0; i < pointCount; ++i)
			//	{
			//		// trick the solver into pushing the fast shapes apart
			//		manifold->points[i].separation -= 0.25f * B2_SPECULATIVE_DISTANCE;
			//	}
			//}
		}
	}

}

static void b2AddNonTouchingContact( b2World* world, b2Contact* contact, b2ContactSim* contactSim )
{
	B2_ASSERT( contact->setIndex == b2_awakeSet );
	b2SolverSet* set = b2Array_Get( world->solverSets, b2_awakeSet );
	contact->colorIndex = B2_NULL_INDEX;
	contact->localIndex = set->contactSims.count;

	b2ContactSim* newContactSim = b2Array_Emplace( set->contactSims );
	memcpy( newContactSim, contactSim, sizeof( b2ContactSim ) );
}

static void b2RemoveNonTouchingContact( b2World* world, int setIndex, int localIndex )
{
	b2SolverSet* set = b2Array_Get( world->solverSets, setIndex );
	int movedIndex = b2Array_RemoveSwap( set->contactSims, localIndex );
	if ( movedIndex != B2_NULL_INDEX )
	{
		b2ContactSim* movedContactSim = set->contactSims.data + localIndex;
		b2Contact* movedContact = b2Array_Get( world->contacts, movedContactSim->contactId );
		B2_ASSERT( movedContact->setIndex == setIndex );
		B2_ASSERT( movedContact->localIndex == movedIndex );
		B2_ASSERT( movedContact->colorIndex == B2_NULL_INDEX );
		movedContact->localIndex = localIndex;
	}
}

static int b2GetCollideContactCount( b2World* world )
{
	int contactCount = 0;
	b2GraphColor* graphColors = world->constraintGraph.colors;
	for ( int i = 0; i < B2_GRAPH_COLOR_COUNT; ++i )
	{
		contactCount += graphColors[i].contactSims.count;
	}

	contactCount += world->solverSets.data[b2_awakeSet].contactSims.count;
	return contactCount;
}

void b2BeginCollide( b2StepContext* context )
{
	b2World* world = context->world;

	B2_ASSERT( world->workerCount > 0 );

	// 将接触收集到单一数组以便 parallel-for
	world->recycledContactCount=0;
	int contactCount = b2GetCollideContactCount( world );
	context->collideContactCount = contactCount;
	if ( contactCount == 0 )
	{
		return;
	}


	b2GraphColor* graphColors = world->constraintGraph.colors;
	int nonTouchingCount = world->solverSets.data[b2_awakeSet].contactSims.count;

	b2ContactCollideSpan* collideSpans = context->collideSpans;
	int spanCount = 0;
	int contactIndex = 0;
	for ( int i = 0; i < B2_GRAPH_COLOR_COUNT; ++i )
	{
		b2GraphColor* color = graphColors + i;
		int count = color->contactSims.count;
		if ( count > 0 )
		{
			collideSpans[spanCount].start = contactIndex;
			collideSpans[spanCount].contacts = color->contactSims.data;
			spanCount += 1;
			contactIndex += count;
		}
	}

	if ( nonTouchingCount > 0 )
	{
		collideSpans[spanCount].start = contactIndex;
		collideSpans[spanCount].contacts = world->solverSets.data[b2_awakeSet].contactSims.data;
		spanCount += 1;
		contactIndex += nonTouchingCount;
	}

	B2_ASSERT( contactIndex == contactCount );
	B2_ASSERT( spanCount <= B2_GRAPH_COLOR_COUNT + 1 );

	collideSpans[spanCount].start = contactCount;
	collideSpans[spanCount].contacts = NULL;

	// 接触位集基于 id，因为接触指针在接触与非接触之间移动时不稳定。
	int contactIdCapacity = b2GetIdCapacity( &world->contactIdPool );
	for ( int i = 0; i < world->workerCount; ++i )
	{
		b2SetBitCountAndClear( &world->taskContexts.data[i].contactStateBitSet, contactIdCapacity );
		world->taskContexts.data[i].recycledContactCount = 0;
	}

	// 任务在 4GHz CPU 上应至少耗时 40us（10K 周期）
	int minRange = 64;
	b2StageParallelFor( world, b2_taskCollide, &b2CollideTask, contactCount, minRange, context );

}

void b2EndCollide( b2StepContext* context )
{
	b2World* world = context->world;

	if ( context->collideContactCount == 0 )
	{
		return;
	}
	for(int i=0;i<world->workerCount;++i) { world->recycledContactCount+=world->taskContexts.data[i].recycledContactCount; }

	b2GraphColor* graphColors = world->constraintGraph.colors;

	// 串行更新接触状态


	// 对所有接触位做按位或
	b2BitSet* bitSet = &world->taskContexts.data[0].contactStateBitSet;
	for ( int i = 1; i < world->workerCount; ++i )
	{
		b2InPlaceUnion( bitSet, &world->taskContexts.data[i].contactStateBitSet );
	}

	b2SolverSet* awakeSet = b2Array_Get( world->solverSets, b2_awakeSet );

	int endEventArrayIndex = world->endEventArrayIndex;

	const b2Shape* shapes = world->shapes.data;
	uint16_t worldId = world->worldId;

	// 处理接触状态变化。遍历已置位
	for ( uint32_t k = 0; k < bitSet->blockCount; ++k )
	{
		uint64_t bits = bitSet->bits[k];
		while ( bits != 0 )
		{
			uint32_t ctz = std::countr_zero( bits );
			int contactId = (int)( 64 * k + ctz );

			b2Contact* contact = b2Array_Get( world->contacts, contactId );
			B2_ASSERT( contact->setIndex == b2_awakeSet );

			int colorIndex = contact->colorIndex;
			int localIndex = contact->localIndex;

			b2ContactSim* contactSim = NULL;
			if ( colorIndex != B2_NULL_INDEX )
			{
				// 接触位于约束图中
				B2_ASSERT( 0 <= colorIndex && colorIndex < B2_GRAPH_COLOR_COUNT );
				b2GraphColor* color = graphColors + colorIndex;
				contactSim = b2Array_Get( color->contactSims, localIndex );
			}
			else
			{
				contactSim = b2Array_Get( awakeSet->contactSims, localIndex );
			}

			uint32_t flags = contact->flags;
			uint32_t simFlags = contactSim->simFlags;

			if ( simFlags & b2_simDisjoint )
			{
				// 包围盒不再重叠
				b2DestroyContact( world, contact );
				contact = NULL;
				contactSim = NULL;
			}
			else if ( simFlags & b2_simStartedTouching )
			{
				B2_ASSERT( contact->islandId == B2_NULL_INDEX );

				if ( flags & b2_contactEnableContactEvents )
				{
					const b2Shape* shapeA = shapes + contact->shapeIdA;
					const b2Shape* shapeB = shapes + contact->shapeIdB;
					b2ShapeId shapeIdA = { shapeA->id + 1, worldId, shapeA->generation };
					b2ShapeId shapeIdB = { shapeB->id + 1, worldId, shapeB->generation };
					b2ContactId contactFullId = {
						.index1 = contactId + 1,
						.world0 = worldId,
						.padding = 0,
						.generation = contact->generation,
					};

					b2ContactBeginTouchEvent event = { shapeIdA, shapeIdB, contactFullId };
					b2Array_Push( world->contactBeginEvents, event );
				}

				B2_ASSERT( contactSim->manifold.pointCount > 0 );
				B2_ASSERT( contact->setIndex == b2_awakeSet );

				// 先链接，因为这会唤醒碰撞刚体并确保刚体仿真位于正确位置。
				contact->flags |= b2_contactTouchingFlag;
				b2LinkContact( world, contact );

				// 确保这些没有改变
				B2_ASSERT( contact->colorIndex == B2_NULL_INDEX );
				B2_ASSERT( contact->localIndex == localIndex );

				// 接触仿真指针可能因唤醒集合增长而失效，因此只需刷新它。
				contactSim = b2Array_Get( awakeSet->contactSims, localIndex );

				contactSim->simFlags &= ~b2_simStartedTouching;

				// 先添加以便 memcpy
				b2AddContactToGraph( world, contactSim, contact );

				// 这会销毁接触仿真
				b2RemoveNonTouchingContact( world, b2_awakeSet, localIndex );

				contactSim = NULL;
			}
			else if ( simFlags & b2_simStoppedTouching )
			{
				contactSim->simFlags &= ~b2_simStoppedTouching;
				contact->flags &= ~b2_contactTouchingFlag;

				if ( contact->flags & b2_contactEnableContactEvents )
				{
					const b2Shape* shapeA = shapes + contact->shapeIdA;
					const b2Shape* shapeB = shapes + contact->shapeIdB;
					b2ShapeId shapeIdA = { shapeA->id + 1, worldId, shapeA->generation };
					b2ShapeId shapeIdB = { shapeB->id + 1, worldId, shapeB->generation };
					b2ContactId contactFullId = {
						.index1 = contactId + 1,
						.world0 = worldId,
						.padding = 0,
						.generation = contact->generation,
					};

					b2ContactEndTouchEvent event = { shapeIdA, shapeIdB, contactFullId };
					b2Array_Push( world->contactEndEvents[endEventArrayIndex], event );
				}

				B2_ASSERT( contactSim->manifold.pointCount == 0 );

				b2UnlinkContact( world, contact );
				int bodyIdA = contact->edges[0].bodyId;
				int bodyIdB = contact->edges[1].bodyId;

				// 先添加以便 memcpy
				b2AddNonTouchingContact( world, contact, contactSim );
				b2RemoveContactFromGraph( world, bodyIdA, bodyIdB, colorIndex, localIndex );
				contact = NULL;
				contactSim = NULL;
			}

			// 清除最低已置位
			bits = bits & ( bits - 1 );
		}
	}




}
