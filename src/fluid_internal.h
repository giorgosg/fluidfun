// SPDX-License-Identifier: MIT
// Shared by the library's source files. Not part of the public API.
#pragma once

#include "box2d/fluidfun.h"

// One coordinate of a b2Pos: double when Box2D is built for large worlds
#if defined( BOX2D_DOUBLE_PRECISION )
typedef double b2fCoord;
#else
typedef float b2fCoord;
#endif

#include "box2d/math_functions.h"

#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define B2F_CHUNK_COUNT 64
#define B2F_CONTACTS_PER_PARTICLE 4

// Squeeze, how far a particle's density is past rest, rises in a line with depth in still water. Measured in
// pools: it starts below rest at the surface, and its slope goes as g h^2 / ( stiffness spacing^2 size ), with size
// over the finest. Read backwards it gives depth from the weight of what is above, which walls do not hide.
#define B2F_SQUEEZE_SURFACE -0.28f
#define B2F_SQUEEZE_SLOPE 0.11f

// Liquid at rest packs looser than the lattice that rest density is counted from: near pressure
// pushes neighbours apart until the density term's pull balances it, about 1.13 spacings apart, where a particle's
// density is 0.72 of rest (B2F_SQUEEZE_SURFACE). This is the area the liquid fills then, as a share of that lattice's.
// Particle mass, the density a wall counts as and a particle's weight all go by it, so that a volume of liquid weighs
// its mass density times that volume however tightly depth or particle size has packed it.
#define B2F_REST_FILL 0.79f
// The same as a distance: how far apart liquid at rest sits, over the spacing of that lattice. The fluid's
// particleSpacing is the rest distance, so the lattice the solver works from is this much finer.
#define B2F_REST_PITCH 1.125f
// How much the fill grows per unit of density over rest density, from lattices 1 to 1.13 spacings apart
#define B2F_FILL_SLOPE 0.75f
#define B2F_WALL_TABLE_SIZE 128

// Neighbours the density pass keeps per particle for the move pass. A particle with more keeps none, and the
// move pass scans the grid for it as before.
#define B2F_PAIRS_PER_PARTICLE 32

// The most one density pass may move a particle, as a fraction of the interaction radius. Effective stiffness
// climbs with density, so without this a pressure pulse overshoots and sends out the next pulse. Water can
// then churn indefinitely at fine spacings, even at depths it would hold if it started still.
#define B2F_MOVE_LIMIT 0.1f
#define B2F_SPAWN_SPACING 1.09f

// Peak move per substep of adhesion at 1, as a share of the interaction radius. About what cohesion at 1 gives a
// particle from three neighbours at its peak distance, so the two settings compare. Not calibrated further.
#define B2F_ADHESION_SCALE 0.19f

// Pressure alone can hold up a column of about this many metres, found by experiment at stiffness 0.04, spacing
// 0.1 and 4 substeps. It is the usual explicit limit, so it scales with stiffness, spacing squared and substeps squared.
#define B2F_DEPTH_LIMIT_SCALE 4.34f

// Fields every particle carries, kept in sort order. Sorting, compaction, splits and spawning all go through
// this list, so a new field is one entry here.
#define B2F_PARTICLE_FIELDS( X )                                                                                        \
	X( b2Pos, pos ) X( b2Pos, prev ) X( float, vx ) X( float, vy ) X( float, size )            \
		X( float, speedMemory ) X( int, id ) X( unsigned char, liquid )

// Working values per particle, rebuilt every substep or every frame, so sorting does not carry them
#define B2F_WORK_FIELDS( X )                                                                                            \
	X( float, dx ) X( float, dy ) X( float, viscX ) X( float, viscY ) X( float, rho ) X( float, rhoNear ) X( float, pressure ) X( float, nearPressure ) X( float, wallDensity ) X( float, wallNearDensity ) \
		X( int, contactCount ) X( int, pairCount ) X( int, cell ) X( int, window ) X( int, sortCell ) X( int, sortBlock ) X( int, permutation ) X( float, adaptDepth ) X( float, adaptFlow )        \
			X( float, adaptMass ) X( float, adaptTarget ) X( float, adaptPressure ) X( unsigned char, adaptClass )      \
				X( unsigned char, adaptUsed )

#define B2F_DECLARE_FIELD( type, name ) type* name;

// Springs a particle holds, each to a partner by id, since indices change with every sort. Both ends hold the same
// spring, and work it out the same way, so it stays the same at both.
#define B2F_SPRINGS_PER_PARTICLE 16

typedef struct b2fSprings
{
	int id[B2F_SPRINGS_PER_PARTICLE];
	// Where the partner was last seen, which the sort's newIndex maps on; the id check catches it when merges and
	// splits have moved it since
	int index[B2F_SPRINGS_PER_PARTICLE];
	float rest[B2F_SPRINGS_PER_PARTICLE];
	int count;
} b2fSprings;

// Where a particle id is this substep, for finding spring partners
typedef struct b2fIdSlot
{
	int id, index;
} b2fIdSlot;

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

// Something a body touches or nearly touches, closer than liquid can fill the gap between them
typedef struct b2fPatch
{
	float normalX, normalY;	 // pushes this body away from the other
	int bodyIndex, otherIndex;
} b2fPatch;

// A gap narrower than this many particle spacings counts as one liquid has not filled
#define B2F_PATCH_GAP 3.0f

// A body slower than this, with liquid all the way round, gets exactly its buoyancy from the liquid; the share falls
// to nothing at this speed, in m/s
#define B2F_STILL_SPEED 0.5f

typedef struct b2fBody
{
	b2BodyId id;
	int firstPatch, patchCount;
	b2SubStepBody motion;
	// World centre of mass and body origin, in double when Box2D is built for large worlds
	b2Pos center, origin;
	bool awake;
	// Velocity as of the end of the last substep, after the liquid's push, or at the start of the step. What Box2D
	// added since then, from gravity, contacts and joints, is the rest of what moved the body this substep.
	b2Vec2 lastVelocity;
	float lastAngularVelocity;
	b2Vec2 solverDelta;
	float solverAngularDelta;
	// The change of motion the liquid within reach shares with the body this substep, for the share pass
	b2Vec2 shareVelocity;
	float shareAngular;
	bool sharing;
	// Static and kinematic bodies pull on the liquid with adhesion; bodies moved by forces do not
	bool adheres;
	// Area of the body's shapes near the liquid, and how far they reach below and above its centre of mass, against
	// gravity, this substep
	float area, low, high;
	// Centre of that area, as its sum weighted by area in the body's frame, and where it is from the centre of mass
	b2Vec2 areaCenter, buoyancyArm;
} b2fBody;

// A shape within the interaction radius of a particle
typedef struct b2fContact
{
	int bodyIndex;
	float normalX, normalY;
	// Kernel integrals over the wall, for the pressure and near pressure parts of the move
	float pressureWeight, nearWeight;
	float move;
	// One where the particle touches the shape, falling to zero at the edge of its reach
	float share;
} b2fContact;

// A neighbour within reach, found by the density pass. Nothing moves before the move pass reads it, so the
// distance is exact there.
typedef struct b2fPair
{
	int j;
	float l;
} b2fPair;

// What the liquid gives a body in a substep, and the mass of the liquid within reach that moves with it, with its
// first and second moments about the body's centre of mass
typedef struct b2fReaction
{
	float x, y, angular;
	float mass, momentX, momentY, inertia;
	// Of the liquid within reach: its mass, that mass times the density its buoyancy goes by, and times its height
	// above the body's centre of mass
	float wetMass, wetDensity, wetHeight;
	// The length of the body's outline that liquid is against, and that length times the outline's normal, which
	// sums to nothing for a body with liquid all the way round
	float wetLength, wetNormalX, wetNormalY;
} b2fReaction;

typedef struct b2fLiquid
{
	b2fLiquidDef def;
	float mass;	 // of a particle at the finest size
} b2fLiquid;

// What two liquids use between them, kept for every ordered pair so the move pass makes one lookup per pair
typedef struct b2fLiquidPair
{
	float cohesion;
	// Share of the pair's move the first liquid takes, over the even share: twice the other's mass density over the
	// sum of both. Heavier particles move less, which keeps momentum and makes lighter liquid float.
	float weight;
	float linearViscosity, quadraticViscosity;
} b2fLiquidPair;


struct b2fFluid
{
	b2WorldId worldId;
	b2fFluidDef def;
	// Spacing of the lattice rest density is counted from, particleSpacing over B2F_REST_PITCH. Particle sizes, the
	// interaction radius and every length inside the library are in these, not in particleSpacing.
	float spacing;
	float radius;	 // interaction radius h, twice the spacing
	float cellSize;	 // smallest grid cell, h plus a skin, since moves after the sort outdate the grid
	float margin;	 // collision radius of a particle
	float mass;	 // of a finest particle of liquid 0, the unit adaptive sizes count mass in
	float restDensity;
	int capacity;
	int count;

	b2fLiquid liquids[B2F_MAX_LIQUIDS];
	int liquidCount;
	bool adaptive;	 // some liquid has adaptive sizes
	float cohesionBetween[B2F_MAX_LIQUIDS][B2F_MAX_LIQUIDS];
	b2fLiquidPair liquidPairs[B2F_MAX_LIQUIDS][B2F_MAX_LIQUIDS];

	B2F_PARTICLE_FIELDS( B2F_DECLARE_FIELD )
	// The same fields in the order the next sort writes, swapped in afterwards
	struct
	{
		B2F_PARTICLE_FIELDS( B2F_DECLARE_FIELD )
	} sorted;
	B2F_WORK_FIELDS( B2F_DECLARE_FIELD )
	b2fContact* contacts;
	b2fPair* pairs;

	// Only made once some liquid has springs, and then sorted, copied and cleared with the particle fields
	b2fSprings* springs;
	b2fSprings* sortedSprings;
	b2fIdSlot* idMap;
	int idMapMask;
	// Each particle's index after the last sort, by its index before it
	int* newIndex;
	bool springy;	 // some liquid has springs
	bool checkSprings;	 // the first substep of a step, which checks both ends hold each spring

	int sizeChanges;
	float maxRadius;
	bool gridValid;
	int nextParticleId;
	float wallTable[B2F_WALL_TABLE_SIZE + 1][4];

	int gridWidth, gridHeight, gridCapacity;
	b2fCoord gridX, gridY;
	float gridCell;	 // cellSize, doubled as often as it takes to keep the grid small
	int uniformWindow;	 // cells each side every particle searches when sizes are uniform, otherwise 0
	int gridBorder;	 // empty cells on each side, as many as the widest search reaches past its own cell
	int* cellStart;

	// Largest particle in each block of grid cells, and the largest whose reach touches each block
	float *blockSize, *blockReach;
	int blockCapacity;

	b2fShape* shapes;
	int shapeCount, shapeCapacity;
	b2fBody* bodies;
	int bodyCount, bodyCapacity;
	b2fReaction* reactions;
	int reactionCapacity;
	b2fPatch* patches;
	int patchCount, patchCapacity;

	// Set per substep. Gravity is liquid 0's, after its gravity scale.
	float h, gravityX, gravityY;
	b2Vec2 worldGravity;
	float moveLimit;
	float movedMass;

	// Density box tables, sized to the liquid's bounds
	double *boxMass, *boxPx, *boxPy, *boxWeight, *boxMixed;
	unsigned char* boxLiquid;
	float *boxDepth, *boxFlow, *boxPressure;
	int boxCapacity;

	// Particle sizes as the caller sees them, in rest spacings, filled when asked for
	float* restSizes;
	int restSizeCapacity;

	b2fProfile profile;
	float measureTime;
};

static inline void b2fCopyParticle( b2fFluid* f, int to, int from )
{
#define B2F_COPY_FIELD( type, name ) f->name[to] = f->name[from];
	B2F_PARTICLE_FIELDS( B2F_COPY_FIELD )
#undef B2F_COPY_FIELD
	if ( f->springs != NULL )
	{
		f->springs[to] = f->springs[from];
	}
}

static inline void b2fClearParticle( b2fFluid* f, int i )
{
#define B2F_CLEAR_FIELD( type, name ) f->name[i] = ( type ){ 0 };
	B2F_PARTICLE_FIELDS( B2F_CLEAR_FIELD )
#undef B2F_CLEAR_FIELD
	if ( f->springs != NULL )
	{
		f->springs[i].count = 0;
	}
}

// Every particle in the square of cells within reach cells of particle i's, itself included. Each row of the square is
// one run of particles, since they are sorted by cell along rows. The grid's border is wide enough that no square
// runs off it.
#define FOR_NEIGHBOURS_WITHIN( f, i, j, reach )                                                                                \
	for ( int r_ = ( reach ), row_ = -r_, cell_ = f->cell[i]; row_ <= r_; ++row_ )                                             \
		for ( int j = f->cellStart[cell_ + row_ * f->gridWidth - r_], end_ = f->cellStart[cell_ + row_ * f->gridWidth + r_ + 1]; \
			  j < end_; ++j )

#define FOR_NEIGHBOURS( f, i, j ) FOR_NEIGHBOURS_WITHIN( f, i, j, f->uniformWindow > 0 ? f->uniformWindow : f->window[i] )

// grid.c
void b2fSortParticles( b2fFluid* f, b2SubStepContext* context );

// solver.c
void b2fPredict( b2fFluid* f, b2SubStepContext* context );
void b2fRelaxDensity( b2fFluid* f, b2SubStepContext* context );
void b2fUpdateVelocities( b2fFluid* f, b2SubStepContext* context );
void b2fRelaxSprings( b2fFluid* f, b2SubStepContext* context );

// boundary.c
void b2fBuildWallTable( b2fFluid* f );
void b2fGatherShapes( b2fFluid* f );
void b2fUpdateBodies( b2fFluid* f, b2SubStepContext* context );
void b2fFindContacts( b2fFluid* f, b2SubStepContext* context );
void b2fCollide( b2fFluid* f, b2SubStepContext* context );
bool b2fShapeDistance( const b2fShape* s, b2Vec2 p, float* distance, b2Vec2* normal );

// adapt.c
void b2fAdaptSizes( b2fFluid* f );
float b2fAdaptiveReach( const b2fFluid* f, int liquid, float g, float h );
