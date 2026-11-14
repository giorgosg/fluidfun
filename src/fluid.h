// SPDX-License-Identifier: MIT
// What the world and the solver call on the world's fluid. The public API is in box2d/fluidfun.h.
#pragma once

#include "box2d/types.h"

struct b2fFluid;

// What the fluid's sub-step can do with the solver. These were public when the fluid was a separate library; now
// only the fluid calls them. They are defined in solver.c.
typedef struct b2SubStepContext b2SubStepContext;

// Runs part of a parallel loop started with b2SubStep_ParallelFor
typedef void b2SubStepTaskFcn( int startIndex, int endIndex, int workerIndex, void* taskContext );

// Motion of a body part way through a step, see b2SubStep_GetBodyMotion
typedef struct b2SubStepBody
{
	b2Pos center;
	b2Rot rotation;
	b2Vec2 localCenter;
	b2Vec2 linearVelocity;
	float angularVelocity;
	// Zero unless the body is awake
	float invMass;
	float invInertia;
} b2SubStepBody;

// Runs a parallel loop on the workers of the current step. The loop finishes before this returns.
void b2SubStep_ParallelFor( b2SubStepContext* context, b2SubStepTaskFcn* task, int itemCount, int minRange,
							void* taskContext );

// Gets a body's motion as of this sub-step. Returns true if the body is awake.
bool b2SubStep_GetBodyMotion( b2SubStepContext* context, b2BodyId bodyId, b2SubStepBody* body );

// Changes an awake body's velocity. Call from the sub-step itself, not from a parallel task.
void b2SubStep_ApplyImpulse( b2SubStepContext* context, b2BodyId bodyId, b2Vec2 impulse, float angularImpulse );

// Before a step, while the world is unlocked: changes particle sizes, runs the emitters for a step of this time and
// gathers the shapes near the liquid
void b2PrepareFluid( struct b2fFluid* fluid, float timeStep, int subStepCount );

// Once per sub-step, after the bodies have moved
void b2FluidSubStep( b2SubStepContext* context, int subStepIndex, int subStepCount, float h, struct b2fFluid* fluid );

// After a step, while the world is unlocked: works out what the liquid tells the caller, body states, events and
// the liquid in sensors
void b2FinishFluid( struct b2fFluid* fluid );

// Throws the liquid as b2World_Explode throws bodies, between steps. Returns how many particles it pushed.
int b2fExplode( struct b2fFluid* fluid, const b2ExplosionDef* def );

// Frees a fluid the world is done with
void b2FreeFluid( struct b2fFluid* fluid );
