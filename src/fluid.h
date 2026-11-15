// SPDX-License-Identifier: MIT
// What the world and the solver call on the world's fluid. The public API is in box2d/fluidfun.h.
#pragma once

#include "parallel_for.h"

#include "box2d/types.h"

struct b2fFluid;

// The fluid's parallel loops in a step, defined in solver.c. The fluid reads and changes bodies, shapes and contacts
// in the world's own arrays.
typedef struct b2SubStepContext b2SubStepContext;

// Runs a parallel loop on the workers of the current step: in the solver's fluid stage while the solver runs, through
// b2ParallelFor when no body is awake, and on the calling thread with no context, as outside a step. The loop finishes
// before this returns.
void b2SubStep_ParallelFor( b2SubStepContext* context, b2ParallelForCallback* task, int itemCount, int minRange,
							void* taskContext );

// Before a step, while the world is unlocked: gives spawned particles waiting for them their sizes, runs the emitters
// for a step of this time and gathers the shapes near the liquid
void b2PrepareFluid( struct b2fFluid* fluid, float timeStep, int subStepCount );

// Once per sub-step, after the bodies have moved
void b2FluidSubStep( b2SubStepContext* context, int subStepIndex, float h, struct b2fFluid* fluid );

// After a step, while the world is unlocked: wakes the sleeping bodies the liquid reached, works out what the liquid
// tells the caller, body states, events and the liquid in sensors, then changes particle sizes
void b2FinishFluid( struct b2fFluid* fluid );

// Throws the liquid as b2World_Explode throws bodies, between steps. Returns how many particles it pushed.
int b2fExplode( struct b2fFluid* fluid, const b2ExplosionDef* def );

// Frees a fluid the world is done with
void b2FreeFluid( struct b2fFluid* fluid );
