// SPDX-License-Identifier: MIT
// Shared by the library's source files. Not part of the public API.
#pragma once

#include "box2d/fluidfun.h"

#include "box2d/math_functions.h"

#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define B2F_CHUNK_COUNT 64
#define B2F_CONTACTS_PER_PARTICLE 4
#define B2F_WALL_TABLE_SIZE 128

// The most one density pass may move a particle, as a fraction of the interaction radius. Effective stiffness
// climbs with density, so without this a pressure pulse overshoots and sends out the next pulse. Water can
// then churn indefinitely at fine spacings, even at depths it would hold if it started still.
#define B2F_MOVE_LIMIT 0.1f
#define B2F_SPAWN_SPACING 1.09f

// Pressure alone can hold up a column of about this many metres, found by experiment at stiffness 0.04, spacing
// 0.1 and 4 substeps. It is the usual explicit limit, so it scales with stiffness, spacing squared and substeps squared.
#define B2F_DEPTH_LIMIT_SCALE 4.34f

// Fields every particle carries, kept in sort order. Sorting, compaction, splits and spawning all go through
// this list, so a new field is one entry here.
#define B2F_PARTICLE_FIELDS( X )                                                                                        \
	X( b2fCoord, x ) X( b2fCoord, y ) X( b2fCoord, px ) X( b2fCoord, py ) X( float, vx ) X( float, vy ) X( float, size )            \
		X( float, speedMemory ) X( int, id )

// Working values per particle, rebuilt every substep or every frame, so sorting does not carry them
#define B2F_WORK_FIELDS( X )                                                                                            \
	X( float, dx ) X( float, dy ) X( float, rho ) X( float, rhoNear ) X( float, wallDensity ) X( float, wallNearDensity ) \
		X( int, contactCount ) X( int, cell ) X( int, permutation ) X( float, adaptDepth ) X( float, adaptFlow )        \
			X( float, adaptMass ) X( float, adaptTarget ) X( float, adaptPressure ) X( unsigned char, adaptClass )      \
				X( unsigned char, adaptUsed )

#define B2F_DECLARE_FIELD( type, name ) type* name;

typedef struct b2fShape
{
	b2ShapeType type;
	int bodyIndex;
	b2Vec2 p1, p2;
	float radius;
	b2Polygon polygon;
	// World bounds this substep, grown by the particle radius
	b2fCoord minX, minY, maxX, maxY;
} b2fShape;

// A flat contact between a body and something else, which shuts liquid out of the gap
typedef struct b2fPatch
{
	b2fCoord x[2], y[2];
	float normalX, normalY;	 // pushes this body away from what it touches
	float length;
	int bodyIndex;
} b2fPatch;

typedef struct b2fBody
{
	b2BodyId id;
	int firstPatch, patchCount;
	b2SubStepBody motion;
	// World centre of mass and body origin, in double when Box2D is built for large worlds
	b2Pos center, origin;
	bool awake;
} b2fBody;

// A shape within the interaction radius of a particle
typedef struct b2fContact
{
	int bodyIndex;
	float normalX, normalY;
	// Kernel integrals over the wall, for the pressure and near pressure parts of the move
	float pressureWeight, nearWeight;
	float move;
} b2fContact;

typedef struct b2fReaction
{
	float x, y, angular;
} b2fReaction;


struct b2fFluid
{
	b2WorldId worldId;
	b2fFluidDef def;
	float radius;	 // interaction radius h
	float cellSize;	 // h plus a skin, since moves within a substep outdate the grid
	float margin;	 // collision radius of a particle
	float mass;
	float restDensity;
	int capacity;
	int count;

	B2F_PARTICLE_FIELDS( B2F_DECLARE_FIELD )
	// The same fields in the order the next sort writes, swapped in afterwards
	struct
	{
		B2F_PARTICLE_FIELDS( B2F_DECLARE_FIELD )
	} sorted;
	B2F_WORK_FIELDS( B2F_DECLARE_FIELD )
	b2fContact* contacts;

	int sizeChanges;
	float maxRadius;
	bool gridValid;
	int nextParticleId;
	float wallTable[B2F_WALL_TABLE_SIZE + 1][4];

	int gridWidth, gridHeight, gridCapacity;
	b2fCoord gridX, gridY;
	int* cellStart;

	b2fShape* shapes;
	int shapeCount, shapeCapacity;
	b2fBody* bodies;
	int bodyCount, bodyCapacity;
	b2fReaction* reactions;
	int reactionCapacity;
	b2fPatch* patches;
	int patchCount, patchCapacity;
	float* patchImpulses;
	int patchImpulseCapacity;

	// Set per substep
	float h, gravityX, gravityY;
	float moveLimit;
	float movedMass;

	// Density box tables, sized to the liquid's bounds
	double *boxMass, *boxPx, *boxPy, *boxWeight;
	float *boxDepth, *boxFlow, *boxPressure;
	int boxCapacity;

	b2fProfile profile;
	float measureTime;
};

static inline void b2fCopyParticle( b2fFluid* f, int to, int from )
{
#define B2F_COPY_FIELD( type, name ) f->name[to] = f->name[from];
	B2F_PARTICLE_FIELDS( B2F_COPY_FIELD )
#undef B2F_COPY_FIELD
}

static inline void b2fClearParticle( b2fFluid* f, int i )
{
#define B2F_CLEAR_FIELD( type, name ) f->name[i] = 0;
	B2F_PARTICLE_FIELDS( B2F_CLEAR_FIELD )
#undef B2F_CLEAR_FIELD
}

// Every particle in the 3x3 cells around particle i, itself included
#define FOR_NEIGHBOURS( f, i, j )                                                                                                \
	for ( int row_ = -1, cell_ = f->cell[i]; row_ <= 1; ++row_ )                                                                 \
		for ( int j = f->cellStart[cell_ + row_ * f->gridWidth - 1], end_ = f->cellStart[cell_ + row_ * f->gridWidth + 2];       \
			  j < end_; ++j )

// grid.c
void b2fSortParticles( b2fFluid* f, b2SubStepContext* context );

// solver.c
void b2fPredict( b2fFluid* f, b2SubStepContext* context );
void b2fApplyViscosity( b2fFluid* f, b2SubStepContext* context );
void b2fRelaxDensity( b2fFluid* f, b2SubStepContext* context );
void b2fUpdateVelocities( b2fFluid* f, b2SubStepContext* context );

// boundary.c
void b2fBuildWallTable( b2fFluid* f );
void b2fGatherShapes( b2fFluid* f );
void b2fUpdateBodies( b2fFluid* f, b2SubStepContext* context );
void b2fFindContacts( b2fFluid* f, b2SubStepContext* context );
void b2fCollide( b2fFluid* f, b2SubStepContext* context );
bool b2fShapeDistance( const b2fShape* s, b2Vec2 p, float* distance, b2Vec2* normal );

// adapt.c
void b2fAdaptSizes( b2fFluid* f );
float b2fAdaptiveReach( const b2fFluid* f, float g, float h );
