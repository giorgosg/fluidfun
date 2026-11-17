// SPDX-License-Identifier: MIT
// What the world and the solver call on the world's fluid. The public API is in box2d/fluidfun.h.
#pragma once

#include "parallel_for.h"

#include "box2d/types.h"

#include <stdbool.h>
#include <stdint.h>

struct b2fFluid;

// The fluid's parallel loops in a step, defined in solver.c. The fluid reads and changes bodies, shapes and contacts
// in the world's own arrays.
typedef struct b2SubStepContext b2SubStepContext;

// Runs a parallel loop on the workers of the current step, in the solver's fluid stage, and on the calling thread with no
// context, as outside a step. When no body is awake the solver's tasks run only the fluid (b2BeginSolve). The loop
// finishes before this returns.
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

// Snapshots and recording (fluid_snapshot.c). A world's snapshot carries a fluid section when the world has a fluid or
// has had one, and a recording's seed and keyframes always do: the serials ids are made from, and the fluid. Without
// one the image is Box2D's own.
struct b2World;
struct b2RecBuffer;
bool b2fHasFluidSection( const struct b2World* world );
void b2fSerializeFluid( const struct b2World* world, struct b2RecBuffer* buf );
// Reads the section at data[*cursor], replacing the world's fluid and serials, and advances the cursor. Returns false
// on a corrupt or foreign section, which leaves the world without a fluid.
bool b2fDeserializeFluid( struct b2World* world, const uint8_t* data, int size, int* cursor );
// For an image without a section: no fluid, and the fluid serial of a world that never had one. The slot's liquid and
// emitter serials stay as they are, only growing, so no old id fits a later liquid or emitter.
void b2fClearFluidState( struct b2World* world );
// Mixes the fluid's state into a hash: every particle, and each body's sleep. Deep adds what else persists.
uint64_t b2fHashFluid( const struct b2World* world, uint64_t hash, bool deep );
