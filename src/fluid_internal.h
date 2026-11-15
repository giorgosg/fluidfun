// SPDX-License-Identifier: MIT
// Shared by the library's source files. Not part of the public API.
#pragma once

#include "box2d/fluidfun.h"
#include "core.h"
#include "fluid.h"

// Inside the library the fluid is a plain struct; callers hold a b2FluidId
typedef struct b2fFluid b2fFluid;
#define B2F_MAX_LIQUIDS B2_MAX_LIQUIDS

// One coordinate of a b2Pos: double when Box2D is built for large worlds
#if defined( BOX2D_DOUBLE_PRECISION )
typedef double b2fCoord;
#else
typedef float b2fCoord;
#endif

#include "box2d/math_functions.h"

#include <float.h>
#include <limits.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// The fluid allocates through Box2D, so b2SetAllocator and b2GetByteCount see its memory. b2Free takes the size, so
// every array is freed with the count it was made with. This grows an array of oldCount items to newCount, keeping
// what it held; the new part is not cleared.
static inline void* b2fGrowArray( void* array, int oldCount, int newCount, size_t itemSize )
{
	return b2GrowAlloc( array, (size_t)oldCount * itemSize, (size_t)newCount * itemSize );
}

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

// Particle sizes are kept as the caller sees them, how far apart particles of that size rest, so particleSpacing at
// the finest. The kernel works on the lattice above instead, where two particles interact within the sum of their
// sizes: this is a size on that lattice, half the particle's interaction radius. A multiply rather than a divide,
// since the passes take it for every pair: dividing made the density pass 5 to 10% slower in the Tank, Pool and
// Layers.
static inline float b2fLatticeSize( float size )
{
	return ( 1.0f / B2F_REST_PITCH ) * size;
}

#define B2F_WALL_TABLE_SIZE 128

// Blocks of this many grid cells a side, as a power of two, carry the largest particle size near them, and the
// shapes whose bounds overlap them
#define B2F_BLOCK_SHIFT 2
#define B2F_BLOCK_CELLS ( 1 << B2F_BLOCK_SHIFT )

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

// Pressure alone can hold up a column of about this depth, found by experiment at stiffness 0.04, spacing 0.1 and 4
// substeps. It is the usual explicit limit, so it scales with stiffness, spacing squared and substeps squared. It is a
// pure number: the depth is this times stiffness spacing^2 / ( g h^2 ), which is a length in any units, so it needs no
// b2_lengthUnitsPerMeter.
#define B2F_DEPTH_LIMIT_SCALE 4.34f

// Fields every particle carries, kept in sort order. Sorting, compaction, splits and spawning all go through
// this list, so a new field is one entry here. Densities are among them since the caller reads them between steps,
// and the coarsening of spawns across the sorts between its passes.
#define B2F_PARTICLE_FIELDS( X )                                                                                        \
	X( b2Pos, pos ) X( b2Pos, prev ) X( float, vx ) X( float, vy ) X( float, size ) X( float, rho ) X( int, id )        \
		X( unsigned char, liquid )

// A particle field only adaptiveSpeed reads, made once some adaptive liquid has it (UpdateLiquids), and then
// sorted, copied and cleared with the particle fields
#define B2F_SPEED_FIELDS( X ) X( float, speedMemory )

// Working values that compaction carries along with the particle fields, though sorting does not: the size pass's own
// compaction keeps the classes and targets for the splits that follow it. Only adaptive sizes use them.
#define B2F_CARRIED_FIELDS( X )                                                                                         \
	X( unsigned char, adaptClass ) X( float, adaptTarget )

// Working values per particle, rebuilt every substep or every frame, so sorting does not carry them. marked is
// scratch for the calls that remove particles.
#define B2F_WORK_FIELDS( X )                                                                                            \
	X( float, dx ) X( float, dy ) X( float, viscX ) X( float, viscY ) X( float, rhoNear ) X( float, pressure )          \
		X( float, nearPressure ) X( float, wallDensity ) X( float, wallNearDensity ) X( int, contactCount )             \
			X( int, pairCount ) X( int, cell ) X( int, sortCell ) X( int, permutation ) X( unsigned char, marked )

// Working values only adaptive sizes use: each particle's search window and grid block, which the sort fills when
// sizes vary, and the size pass's measurements and scratch. Made with the carried fields once some liquid is
// adaptive.
#define B2F_ADAPT_FIELDS( X )                                                                                           \
	X( int, window ) X( int, sortBlock ) X( float, adaptDepth ) X( float, adaptFlow ) X( float, adaptMass )            \
		X( float, adaptPressure ) X( unsigned char, adaptUsed )

#define B2F_DECLARE_FIELD( type, name ) type* name;

// Springs a particle holds, each to a partner by id, since indices change with every sort, with removals between
// steps, and with merges and splits at the end of each. The spring pass finds each partner in an id table it builds
// every substep. Both ends hold the same spring, and work it out the same way, so it stays the same at both.
#define B2F_SPRINGS_PER_PARTICLE 16

typedef struct b2fSprings
{
	int id[B2F_SPRINGS_PER_PARTICLE];
	float rest[B2F_SPRINGS_PER_PARTICLE];
	int count;
} b2fSprings;

// Where a particle id is this substep, for finding spring partners
typedef struct b2fIdSlot
{
	int id, index;
} b2fIdSlot;

// Bounds of particle centres
typedef struct b2fBox
{
	b2fCoord minX, minY, maxX, maxY;
} b2fBox;

// The grid covers the bulk of the liquid only while at most one particle in this many is outside it (b2fFindBulk)
#define B2F_BULK_OUTSIDE 8

typedef struct b2fShape
{
	b2ShapeType type;
	int bodyIndex;
	b2Vec2 p1, p2;
	float radius;
	b2Polygon polygon;
	// A polygon thinner than a particle spacing, which a fast particle can cross within one substep
	bool thin;
	// World bounds this substep, grown by the particle radius
	b2fCoord minX, minY, maxX, maxY;
	// The shape's filter, and a bit for each liquid it collides with, by index. A shape no liquid collides with is not
	// gathered at all, so this only matters when b2fFluid::shapesFiltered says some shape leaves some liquid out.
	uint64_t categoryBits, maskBits;
	uint32_t liquids;
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
// to nothing at this speed, 0.5 m/s in the caller's length units
#define B2F_STILL_SPEED ( 0.5f * b2_lengthUnitsPerMeter )

// Liquid against a face whose outward normal is closer to up than this, as a cosine, lies on top of the body rather
// than standing beside it, so it does not count towards the level a body reports
#define B2F_TOP_FACING 0.5f

// The nearest point of a shape counts as buried in another shape held fixed against it when it is closer to that
// shape than this share of the particle's lattice size, as where two tiles of a floor meet
#define B2F_SEAM_TOLERANCE 0.1f

// A body's motion as of this substep, read from Box2D's solver set
typedef struct b2fMotion
{
	b2Pos center;
	b2Rot rotation;
	b2Vec2 localCenter;
	b2Vec2 linearVelocity;
	float angularVelocity;
	// Zero unless the body is awake
	float invMass;
	float invInertia;
} b2fMotion;

typedef struct b2fBody
{
	b2BodyId id;
	int firstPatch, patchCount;
	b2fMotion motion;
	// Box2D's velocity of the body while it is awake in a step, which the liquid's push changes, set each substep;
	// null while it sleeps and outside a step
	struct b2BodyState* state;
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
	// Static bodies never move, so their shapes keep where they are against each other, as a body's own shapes do
	bool isStatic;
	// Area of the body's shapes near the liquid, and how far they reach below and above its centre of mass, against
	// gravity, this substep
	float area, low, high;
	// Centre of that area, as its sum weighted by area in the body's frame, and where it is from the centre of mass
	b2Vec2 areaCenter, buoyancyArm;
	// What the liquid did to the body over this step, summed over substeps, for b2Fluid_GetBodyState: its impulse and
	// angular impulse once patches are applied, the share under liquid, and whether any liquid was within reach
	b2Vec2 stepImpulse;
	float stepAngular, stepSubmerged;
	bool stepWet;
	// Whether liquid was within reach of the body in some substep of this step, asleep or not. Those asleep wake once
	// the step is done (b2fWakeReached).
	bool reached;
	// Where the body is at the start of the step, and where its shapes start in b2fFluid::bodyShapes, for finding
	// patches
	b2WorldTransform transform;
	int firstShape, shapeCount;
} b2fBody;

// Shapes sorted into the squares of a grid by their bounds, so that a point or a box finds the shapes it could touch
// without testing all of them. A shape is in every square its bounds overlap, and each square lists its shapes in
// their order in b2fFluid::shapes, so a pass that walks a square's list meets them in the order a walk over every
// shape would. A point or bounds outside the grid counts as in the nearest square on its edge, which keeps every
// overlap: clamping does not change the order of coordinates.
typedef struct b2fShapeBins
{
	b2fCoord x, y;	 // lower corner of the grid
	float inverse;	 // one over the side of a square
	int width, height;
	// Where each square's list starts in shapes, width * height + 1 of them
	int* start;
	int* shapes;
	int startCapacity, shapeCapacity;
} b2fShapeBins;

// A body that has been in liquid lately, kept across steps, since the list of bodies is rebuilt every step
typedef struct b2fBodyRecord
{
	b2BodyId id;
	b2FluidBodyState state;
	// Seconds of steps in a row whose liquid within reach disagrees with state.inLiquid; it flips at B2F_WET_TIME
	float run;
	// Whether the body was near the liquid this step, and had liquid in reach
	bool seen, wet;
} b2fBodyRecord;

// A body's inLiquid changes after this many seconds of steps in a row with liquid in reach, or without it, so that a
// splash passing by does not begin and end it every step. At 60 Hz that is three steps, and at 120 Hz six.
#define B2F_WET_TIME 0.045f

// A sensor shape near the liquid this step, or one that held liquid last step
typedef struct b2fSensor
{
	b2ShapeId id;
	float area;
	bool gathered;
	// A bit for each liquid the sensor counts, by index, from the filters when last gathered
	uint32_t liquids;
} b2fSensor;

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
	// Of the liquid within reach: its mass, and that mass times the density its buoyancy goes by
	float wetMass, wetDensity;
	// The length of the body's outline that liquid is against, and that length times the outline's normal, which
	// sums to nothing for a body with liquid all the way round
	float wetLength, wetNormalX, wetNormalY;
	// How high the liquid against the body stands above its lowest point: the highest particle that is not spray, and
	// not lying on an upward face, plus half its size, or zero
	float wetTop;
} b2fReaction;

#define B2F_MAX_EMITTERS B2_MAX_EMITTERS

// A mouth stepped by the world (fluid_spawn.c)
typedef struct b2fEmitter
{
	b2EmitterDef def;
	int liquid;
	// How far the liquid has gone across the mouth since the last row, and how many rows there have been, which sets
	// each row's half-spacing shift
	float travel;
	int rows;
	// As for liquids: a destroyed emitter leaves its slot, and the next one in it has a new generation
	bool alive;
	uint16_t generation;
} b2fEmitter;

typedef struct b2fLiquid
{
	b2LiquidDef def;
	float mass;	 // of a particle at the finest size
	// A destroyed liquid leaves its slot for the next one created. The generation its ids carry is the world's
	// liquidSerial when it was made, so an old id does not fit a later liquid in the slot, in this fluid or the next.
	bool alive;
	uint16_t generation;
} b2fLiquid;

// What two liquids use between them, kept for every ordered pair so the move pass makes one lookup per pair
typedef struct b2fLiquidPair
{
	float cohesion;
	// Share of the pair's move the first liquid takes, over the even share: twice the other's mass density over the
	// sum of both. Heavier particles move less, which keeps momentum and makes lighter liquid float.
	float weight;
	float viscosity, quadraticViscosity;
} b2fLiquidPair;


struct b2fFluid
{
	b2World* world;
	b2FluidDef def;
	// Spacing of the lattice rest density is counted from, particleSpacing over B2F_REST_PITCH. The interaction
	// radius and the solver's lengths go by it; particle sizes are in particleSpacing (b2fLatticeSize).
	float spacing;
	float radius;	 // interaction radius h, twice the spacing
	float cellSize;	 // smallest grid cell, h plus a skin, since moves after the sort outdate the grid
	float fineArea;	 // the area a finest particle fills at rest, which adaptive sizes count mass in, whatever the liquid
	float restDensity;
	// Room for this many particles in every per-particle array, which b2fGrow doubles up to def.maxCapacity
	int capacity;
	int count;

	b2fEmitter emitters[B2F_MAX_EMITTERS];
	int emitterCount;

	b2fLiquid liquids[B2F_MAX_LIQUIDS];
	int liquidCount;
	bool adaptive;	 // some liquid has adaptive sizes
	float cohesionBetween[B2F_MAX_LIQUIDS][B2F_MAX_LIQUIDS];
	b2fLiquidPair liquidPairs[B2F_MAX_LIQUIDS][B2F_MAX_LIQUIDS];
	// For each liquid a bit for every liquid it pushes on, itself always included, and whether some pair of live
	// liquids passes through each other. Only then do the passes look at these, so default filters cost nothing.
	uint32_t liquidSees[B2F_MAX_LIQUIDS];
	bool liquidsFiltered;
	// A bit for each live liquid, and whether some gathered shape leaves out some live liquid this step. Shapes no
	// liquid collides with are not gathered, so with one liquid this stays false and the passes are as without filters.
	uint32_t liveLiquids;
	bool shapesFiltered;
	// Some shape near the liquid was left out this step since no liquid collides with it
	bool shapesDropped;

	B2F_PARTICLE_FIELDS( B2F_DECLARE_FIELD )
	B2F_SPEED_FIELDS( B2F_DECLARE_FIELD )
	// The same fields in the order the next sort writes, swapped in afterwards
	struct
	{
		B2F_PARTICLE_FIELDS( B2F_DECLARE_FIELD )
		B2F_SPEED_FIELDS( B2F_DECLARE_FIELD )
	} sorted;
	B2F_WORK_FIELDS( B2F_DECLARE_FIELD )
	// Only made once some liquid is adaptive, and then kept, grown and freed with the rest; NULL until then
	B2F_CARRIED_FIELDS( B2F_DECLARE_FIELD )
	B2F_ADAPT_FIELDS( B2F_DECLARE_FIELD )
	b2fContact* contacts;
	b2fPair* pairs;

	// Only made once some liquid has springs, and then sorted, copied and cleared with the particle fields
	b2fSprings* springs;
	b2fSprings* sortedSprings;
	b2fIdSlot* idMap;
	int idMapMask, idMapShift;
	bool springy;	 // some liquid has springs
	bool checkSprings;	 // the first substep of a step, which checks both ends hold each spring

	int sizeChanges;
	float maxRadius;
	int nextParticleId;
	// Particles spawned since the last step that take their sizes at the start of the next one, before they move: those
	// of each liquid with a bit in coarsenLiquids and an id from coarsenFrom of that liquid on
	uint32_t coarsenLiquids;
	int coarsenFrom[B2F_MAX_LIQUIDS];
	float wallTable[B2F_WALL_TABLE_SIZE + 1][4];

	int gridWidth, gridHeight, gridCapacity;
	b2fCoord gridX, gridY;
	float gridCell;	 // cellSize, doubled as often as it takes to keep the grid small
	int uniformWindow;	 // cells each side every particle searches when sizes are uniform, otherwise 0
	int gridBorder;	 // empty cells on each side, as many as the widest search reaches past its own cell
	int* cellStart;
	// The particles the grid is laid over, inside its border: all of them, or with gridBulkOnly the bulk of the liquid,
	// and the rest are in the cells along its edge
	b2fBox gridBulk;
	bool gridBulkOnly;

	// Largest particle in each block of grid cells, and the largest whose reach touches each block
	float *blockSize, *blockReach;
	int blockCapacity;

	b2fShape* shapes;
	int shapeCount, shapeCapacity;
	// When liquid lies outside its bulk, shapes are gathered with several queries, and this marks each shape of the
	// world, by its index there, with the gather that took it, so it is taken once
	int* gatherMark;
	int gatherMarkCapacity, gatherSerial;
	bool gatherMarking;
	b2fBody* bodies;
	int bodyCount, bodyCapacity;
	// Where each body of the world is in bodies, by its index in the world; good only where that entry is below
	// bodyCount and its body has this index, so it needs no clearing between steps
	int* bodyOfIndex;
	int bodyOfIndexCapacity;
	b2fReaction* reactions;
	int reactionCapacity;
	b2fPatch* patches;
	int patchCount, patchCapacity;
	// The shapes in the blocks of each substep's grid, and the shapes forces do not move in a grid over them, once a
	// step, for finding patches
	b2fShapeBins shapeBins, fixedBins;
	// Scratch for finding patches, one of each per shape: the shapes of each body in order, a mark per shape, and
	// the candidates of one query
	int* bodyShapes;
	int* shapeMark;
	int* candidates;
	int markCapacity;

	// Set per step and substep. Gravity is the world's; each liquid scales it by its gravityScale.
	float h;
	b2Vec2 worldGravity;

	// Density box tables, sized to the liquid's bounds
	double *boxMass, *boxPx, *boxPy, *boxWeight, *boxMixed;
	unsigned char* boxLiquid;
	float *boxDepth, *boxFlow, *boxPressure;
	int boxCapacity;

	b2FluidProfile profile;
	float measureTime;

	// What the liquid tells the caller after each step (fluid_state.c). Time and substeps the liquid ran this step.
	float stepTime;
	int stepSubSteps;
	b2fBodyRecord* records;
	int recordCount, recordCapacity;
	// The record of each body, by its index in the world, plus one; zero for none. A destroyed body's slot can point
	// at its record until the record goes, so the id in the record is checked.
	int* recordOfBody;
	int recordOfBodyCapacity;
	b2fSensor* sensors;
	int sensorCount, sensorCapacity;
	// Mass of each liquid within reach of each body in the last substep, to find which liquid a body is in
	float* bodyLiquidMass;
	int bodyLiquidCapacity;
	b2FluidBodyBeginEvent* bodyBeginEvents;
	b2FluidBodyEndEvent* bodyEndEvents;
	b2FluidSensorBeginEvent* sensorBeginEvents;
	b2FluidSensorEndEvent* sensorEndEvents;
	int bodyBeginCount, bodyEndCount, sensorBeginCount, sensorEndCount;
	int bodyBeginCapacity, bodyEndCapacity, sensorBeginCapacity, sensorEndCapacity;
};

static inline void b2fCopyParticle( b2fFluid* f, int to, int from )
{
#define B2F_COPY_FIELD( type, name ) f->name[to] = f->name[from];
	B2F_PARTICLE_FIELDS( B2F_COPY_FIELD )
	if ( f->speedMemory != NULL )
	{
		B2F_SPEED_FIELDS( B2F_COPY_FIELD )
	}
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
	if ( f->speedMemory != NULL )
	{
		B2F_SPEED_FIELDS( B2F_CLEAR_FIELD )
	}
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

// A shape's outline where it is now, and its bounds in the world, for testing whether particle centres are inside it.
// Every call that picks particles in a region tests this one outline (b2fPicks), whatever the region's type: a box
// has the type b2_shapeTypeCount and is its bounds alone, so its test is four compares, and a proxy is turned into a
// circle, capsule or polygon once per call.
typedef struct b2fOutline
{
	b2WorldTransform transform;
	b2ShapeType type;
	b2Polygon polygon;
	b2Circle circle;
	b2Capsule capsule;
	b2fCoord minX, minY, maxX, maxY;
} b2fOutline;

// fluid.c
b2fFluid* b2fGetFluid( b2FluidId fluidId );
// Makes room for at least needed particles, doubling, up to maxCapacity. Not during a substep.
// Returns false if needed is past maxCapacity.
bool b2fGrow( b2fFluid* f, int needed );
// Removes the particles with their mark set, keeping the order of the rest. The particle fields and the carried fields
// go with each particle; the grid of the last sort no longer holds. Not during a substep. Returns how many went.
int b2fRemoveMarked( b2fFluid* f );
// The liquid's index in its fluid
int b2fGetLiquid( b2LiquidId liquidId, b2fFluid** fluid );
// For the calls that act between steps: asserts the world is not locked, and returns whether it is
bool b2fWorldLocked( b2FluidId fluidId );
// The liquid a call that picks particles acts on, or -1 for every liquid when the id is null
int b2fPickLiquid( b2fFluid* f, b2LiquidId liquidId );
// The id of the liquid in a slot
b2LiquidId b2fMakeLiquidId( const b2fFluid* f, int index );
// A bit for each live liquid that collides with a shape of this filter: each one's mask includes the other's category
uint32_t b2fLiquidsCollidingWith( const b2fFluid* f, b2Filter filter );

// grid.c
// With no context, on this thread, as outside a step
void b2fSortParticles( b2fFluid* f, b2SubStepContext* context );
// The bounds of every particle, and the part of the liquid a grid of this cell and border covers: every particle, and
// false, when their bounds fit the grid's limits, or when the box found below leaves out more than one particle in
// B2F_BULK_OUTSIDE. Otherwise true, and the bulk is a box that fits and holds as many particles as it can, found along
// each axis in turn. One thread, in particle order, so it does not depend on worker count.
bool b2fFindBulk( const b2fFluid* f, float cell, int border, b2fBox* all, b2fBox* bulk );

// solver.c
void b2fPredict( b2fFluid* f, b2SubStepContext* context );
void b2fRelaxDensity( b2fFluid* f, b2SubStepContext* context );
void b2fUpdateVelocities( b2fFluid* f, b2SubStepContext* context );
void b2fRelaxSprings( b2fFluid* f, b2SubStepContext* context );

// boundary.c
void b2fBuildWallTable( b2fFluid* f );
void b2fGatherShapes( b2fFluid* f );
void b2fWakeReached( b2fFluid* f );
void b2fUpdateBodies( b2fFluid* f );
void b2fFindContacts( b2fFluid* f, b2SubStepContext* context );
void b2fCollide( b2fFluid* f, b2SubStepContext* context );
void b2fHoldBodies( b2fFluid* f );
void b2fCollideHeld( b2fFluid* f );
bool b2fShapeDistance( const b2fShape* s, b2Vec2 p, float* distance, b2Vec2* normal );

// spawn.c
// Adds the rows each enabled emitter owes for a step of this time
void b2fStepEmitters( b2fFluid* f, float timeStep );
// Ends the emitters of a liquid that is being destroyed
void b2fDestroyEmitters( b2fFluid* f, int liquid );

// state.c
// Returns false for shapes with no inside, segments and chains
bool b2fMakeOutline( b2ShapeId shapeId, b2fOutline* outline );
// The outline of a region, or false for a region with no inside: a segment or chain shape, or bad input
bool b2fMakeRegionOutline( const b2FluidRegion* region, b2fOutline* outline );
// A box between two corners, and a circle
b2fOutline b2fMakeBoxOutline( b2Pos lower, b2Pos upper );
b2fOutline b2fMakeCircleOutline( b2Pos center, float radius );
bool b2fInsideOutline( const b2fOutline* outline, b2Pos p );
// Whether a call that picks particles in a region takes particle i: one of that liquid, or of any for -1, whose centre
// is inside the outline. The outline is a region, so filters play no part.
static inline bool b2fPicks( const b2fFluid* f, int liquid, const b2fOutline* outline, int i )
{
	return ( liquid < 0 || f->liquid[i] == liquid ) && b2fInsideOutline( outline, f->pos[i] );
}
// Notes a sensor shape near the liquid this step, which counts the liquids in the mask. One that counts none is left out.
void b2fGatherSensor( b2fFluid* f, b2ShapeId shapeId, uint32_t liquids );
void b2fFinishStep( b2fFluid* f );
void b2fFreeState( b2fFluid* f );

// adapt.c
// The size pass, at the end of a step that ran substeps. Returns whether it ran.
bool b2fAdaptSizes( b2fFluid* f );
// Gives particles waiting since their spawn the sizes their depth asks for, with no movement, for a step of h a substep
void b2fCoarsenSpawned( b2fFluid* f, float h );
