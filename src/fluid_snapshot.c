// SPDX-License-Identifier: MIT
// The fluid in a world's snapshot, which seeds a recording and makes its keyframes, and in the hash a recording checks
// each frame. The section follows Box2D's own image, flagged in its header, so a world without one is Box2D's.
//
// What a step reads from the steps before it is written here; what it rebuilds is not. Everything a step reads that
// it did not write itself in that step:
// - The def and the derived lengths: def written, spacing, radius, cellSize, fineArea, restDensity and wallTable
//   rebuilt from it by b2fMakeFluid, as b2CreateFluid builds them.
// - Liquids and emitters, whole, and the cohesion between liquids; the pair table, liquidSees, liveLiquids,
//   liquidsFiltered, adaptive and springy rebuilt by b2fUpdateLiquids from them.
// - Each particle's position, velocity, size, density, id and liquid, its sleep time, smoothed velocity (or density
//   to wake by) and whether it is asleep, its speedMemory and springFormed where those arrays have been made, in the
//   fluid's order, which the next sort keeps within a cell.
// - The spring table, in its order, springs to particles that have gone included: the next spring pass removes them. prev is set at the start of every substep before anything
//   reads it, so it is not written.
// - maxRadius, which the last sort of a step set from sizes that the size pass has changed since, and which the next
//   step's gather reads before it sorts.
// - The shapes and bodies the last step gathered, as its last substep left them: a spawn's coarsening at the start of
//   the next step reads them before that step gathers its own (ChangeSizes in fluid_adapt.c).
// - Body records and sensors, which the next step's begin and end events compare against, and recordOfBody.
// - nextParticleId, coarsenLiquids and coarsenFrom, capacity, and h and worldGravity, which every step sets before it
//   reads them but cost nothing to keep.
// - The world's fluidSerial, liquidSerial and emitterSerial, which the next fluid, liquid and emitter take as ids.
// The rest is the working state of a substep or a step, rebuilt before it is read. Events are cleared, as Box2D clears
// its own on restore.
#include "fluid_internal.h"

#include "body.h"
#include "physics_world.h"
#include "recording.h"

#define B2F_SNAP_MAGIC 0x444C4646u // 'FFLD'
#define B2F_SNAP_VERSION 4u

// The structs written whole. A change to any of them changes this, and an image of another layout is refused.
static uint32_t FluidLayoutHash( void )
{
	uint32_t h = 2166136261u;
#define MIX( x )                                                                                                       \
	h ^= (uint32_t)( x );                                                                                              \
	h *= 16777619u;
	MIX( sizeof( b2FluidDef ) )
	MIX( sizeof( b2LiquidDef ) )
	MIX( sizeof( b2EmitterDef ) )
	MIX( sizeof( b2fLiquid ) )
	MIX( sizeof( b2fEmitter ) )
	MIX( sizeof( b2fSpring ) )
	MIX( sizeof( b2fShape ) )
	MIX( sizeof( b2fBody ) )
	MIX( sizeof( b2fBodyRecord ) )
	MIX( sizeof( b2fSensor ) )
	MIX( sizeof( b2Pos ) )
	MIX( B2F_MAX_LIQUIDS )
	MIX( B2F_MAX_EMITTERS )
#undef MIX
	return h;
}

bool b2fHasFluidSection( const b2World* world )
{
	return world->fluid != NULL || world->fluidSerial != 0;
}

static void W( b2RecBuffer* buf, const void* data, size_t size )
{
	b2RecBufAppend( buf, data, (int)size );
}

static void WI( b2RecBuffer* buf, int v )
{
	int32_t w = v;
	b2RecBufAppend( buf, &w, 4 );
}

void b2fSerializeFluid( const b2World* world, b2RecBuffer* buf )
{
	uint32_t header[3] = { B2F_SNAP_MAGIC, B2F_SNAP_VERSION, FluidLayoutHash() };
	W( buf, header, sizeof( header ) );
	WI( buf, world->fluidSerial );
	W( buf, &world->liquidSerial, sizeof( uint16_t ) );
	W( buf, &world->emitterSerial, sizeof( uint16_t ) );

	const b2fFluid* f = world->fluid;
	uint8_t has = f != NULL ? 1 : 0;
	W( buf, &has, 1 );
	if ( f == NULL )
	{
		return;
	}

	W( buf, &f->def, sizeof( f->def ) );
	WI( buf, f->capacity );
	WI( buf, f->count );
	WI( buf, f->nextParticleId );
	W( buf, &f->maxRadius, sizeof( float ) );
	W( buf, &f->h, sizeof( float ) );
	W( buf, &f->worldGravity, sizeof( b2Vec2 ) );
	W( buf, &f->coarsenLiquids, sizeof( uint32_t ) );
	W( buf, f->coarsenFrom, sizeof( f->coarsenFrom ) );

	WI( buf, f->liquidCount );
	W( buf, f->liquids, sizeof( f->liquids ) );
	W( buf, f->cohesionBetween, sizeof( f->cohesionBetween ) );
	WI( buf, f->emitterCount );
	W( buf, f->emitters, sizeof( f->emitters ) );

	// Which of the optional arrays have been made: they stay once made, and a made spring array is carried by every
	// sort, so the restored fluid makes the same ones
	uint8_t made[3] = { f->adaptMass != NULL, f->speedMemory != NULL, f->springFormed != NULL };
	W( buf, made, sizeof( made ) );

	size_t n = (size_t)f->count;
	W( buf, f->pos, n * sizeof( b2Pos ) );
	W( buf, f->vx, n * sizeof( float ) );
	W( buf, f->vy, n * sizeof( float ) );
	W( buf, f->size, n * sizeof( float ) );
	W( buf, f->rho, n * sizeof( float ) );
	W( buf, f->id, n * sizeof( int ) );
	W( buf, f->liquid, n );
	W( buf, f->sleepTime, n * sizeof( float ) );
	W( buf, f->calm, n * sizeof( b2Vec2 ) );
	W( buf, f->asleep, n );
	if ( f->speedMemory != NULL )
	{
		W( buf, f->speedMemory, n * sizeof( float ) );
	}
	if ( f->springFormed != NULL )
	{
		W( buf, f->springFormed, n * sizeof( bool ) );
		WI( buf, f->springs.count );
		W( buf, f->springs.data, (size_t)f->springs.count * sizeof( b2fSpring ) );
	}

	// Gathered by the last step, read by the next step's coarsening before it gathers. The body state pointer is only
	// good inside a step.
	WI( buf, f->shapes.count );
	W( buf, f->shapes.data, (size_t)f->shapes.count * sizeof( b2fShape ) );
	WI( buf, f->bodies.count );
	for ( int i = 0; i < f->bodies.count; ++i )
	{
		b2fBody body = f->bodies.data[i];
		body.state = NULL;
		W( buf, &body, sizeof( body ) );
	}
	uint8_t flags[2] = { f->shapesFiltered, f->shapesDropped };
	W( buf, flags, sizeof( flags ) );

	WI( buf, f->records.count );
	W( buf, f->records.data, (size_t)f->records.count * sizeof( b2fBodyRecord ) );
	int lookup = f->recordOfBody.count;
	while ( lookup > 0 && f->recordOfBody.data[lookup - 1] == 0 )
	{
		lookup -= 1;
	}
	WI( buf, lookup );
	W( buf, f->recordOfBody.data, (size_t)lookup * sizeof( int ) );
	WI( buf, f->sensors.count );
	W( buf, f->sensors.data, (size_t)f->sensors.count * sizeof( b2fSensor ) );
}

typedef struct Reader
{
	const uint8_t* data;
	int cursor, size;
	bool ok;
} Reader;

static void R( Reader* r, void* out, size_t size )
{
	if ( size == 0 )
	{
		return;
	}
	if ( r->ok == false || size > (size_t)( r->size - r->cursor ) )
	{
		r->ok = false;
		memset( out, 0, size );
		return;
	}
	memcpy( out, r->data + r->cursor, size );
	r->cursor += (int)size;
}

static int RI( Reader* r )
{
	int32_t v = 0;
	R( r, &v, 4 );
	return v;
}

// A count from the image that must lie in [0, limit] and have at least bytes per item left in the image
static int RCount( Reader* r, int limit, size_t bytes )
{
	int count = RI( r );
	if ( count < 0 || count > limit || (size_t)count * bytes > (size_t)( r->size - r->cursor ) )
	{
		r->ok = false;
		return 0;
	}
	return count;
}

void b2fClearFluidState( b2World* world )
{
	if ( world->fluid != NULL )
	{
		b2FreeFluid( world->fluid );
		world->fluid = NULL;
	}
	world->fluidSerial = 0;
}

bool b2fDeserializeFluid( b2World* world, const uint8_t* data, int size, int* cursor )
{
	b2fClearFluidState( world );
	world->liquidSerial = 0;
	world->emitterSerial = 0;
	Reader reader = { data, *cursor, size, true };
	Reader* r = &reader;

	uint32_t header[3];
	R( r, header, sizeof( header ) );
	if ( r->ok == false || header[0] != B2F_SNAP_MAGIC || header[1] != B2F_SNAP_VERSION || header[2] != FluidLayoutHash() )
	{
		b2Log( "snapshot fluid section mismatch\n" );
		return false;
	}

	world->fluidSerial = RI( r );
	R( r, &world->liquidSerial, sizeof( uint16_t ) );
	R( r, &world->emitterSerial, sizeof( uint16_t ) );
	uint8_t has = 0;
	R( r, &has, 1 );
	if ( r->ok == false || has == 0 )
	{
		*cursor = r->cursor;
		return r->ok;
	}

	b2FluidDef def;
	R( r, &def, sizeof( def ) );
	int capacity = RI( r );
	if ( r->ok == false || capacity < 0 || capacity > def.maxCapacity || b2IsValidFloat( def.particleSpacing ) == false ||
		 def.particleSpacing <= 0.0f )
	{
		return false;
	}
	b2fFluid* f = b2fMakeFluid( world, &def, capacity );
	world->fluid = f;

	int count = RCount( r, capacity, sizeof( b2Pos ) );
	f->nextParticleId = RI( r );
	R( r, &f->maxRadius, sizeof( float ) );
	R( r, &f->h, sizeof( float ) );
	R( r, &f->worldGravity, sizeof( b2Vec2 ) );
	R( r, &f->coarsenLiquids, sizeof( uint32_t ) );
	R( r, f->coarsenFrom, sizeof( f->coarsenFrom ) );
	f->liquidCount = RCount( r, B2F_MAX_LIQUIDS, 0 );
	R( r, f->liquids, sizeof( f->liquids ) );
	R( r, f->cohesionBetween, sizeof( f->cohesionBetween ) );
	f->emitterCount = RCount( r, B2F_MAX_EMITTERS, 0 );
	R( r, f->emitters, sizeof( f->emitters ) );
	uint8_t made[3];
	R( r, made, sizeof( made ) );
	if ( r->ok == false )
	{
		b2fClearFluidState( world );
		return false;
	}

	// A body an emitter rides is in this world, wherever the image came from
	uint16_t world0 = world->worldId;
	for ( int e = 0; e < B2F_MAX_EMITTERS; ++e )
	{
		r->ok = r->ok && ( f->emitters[e].alive == false || ( 0 <= f->emitters[e].liquid && f->emitters[e].liquid < f->liquidCount ) );
		if ( B2_IS_NON_NULL( f->emitters[e].def.bodyId ) )
		{
			f->emitters[e].def.bodyId.world0 = world0;
		}
	}

	if ( r->ok == false )
	{
		b2fClearFluidState( world );
		return false;
	}
	b2fUpdateLiquids( f );
	b2fAllocateOptional( f, made[0] != 0, made[1] != 0, made[2] != 0 );

	size_t n = (size_t)count;
	f->count = count;
	// The particles are filed again, awake and asleep, before the next substep, as they were filed before this one
	f->activeCount = count;
	f->settledCount = 0;
	f->gridStale = true;
	R( r, f->pos, n * sizeof( b2Pos ) );
	R( r, f->vx, n * sizeof( float ) );
	R( r, f->vy, n * sizeof( float ) );
	R( r, f->size, n * sizeof( float ) );
	R( r, f->rho, n * sizeof( float ) );
	R( r, f->id, n * sizeof( int ) );
	R( r, f->liquid, n );
	R( r, f->sleepTime, n * sizeof( float ) );
	R( r, f->calm, n * sizeof( b2Vec2 ) );
	R( r, f->asleep, n );
	memcpy( f->prev, f->pos, n * sizeof( b2Pos ) );
	for ( int i = 0; i < count; ++i )
	{
		r->ok = r->ok && f->liquid[i] < f->liquidCount && f->asleep[i] <= ( B2F_ASLEEP | B2F_ASLEEP_SHAPE | B2F_ASLEEP_NEW );
	}
	b2fCountAsleep( f );
	if ( made[1] )
	{
		R( r, f->speedMemory, n * sizeof( float ) );
	}
	if ( made[2] )
	{
		R( r, f->springFormed, n * sizeof( bool ) );
		for ( int i = 0; i < count; ++i )
		{
			r->ok = r->ok && ( (const unsigned char*)f->springFormed )[i] <= 1u;
		}
		int springCount = RCount( r, INT_MAX / (int)sizeof( b2fSpring ), sizeof( b2fSpring ) );
		b2Array_Resize( f->springs, springCount );
		R( r, f->springs.data, (size_t)springCount * sizeof( b2fSpring ) );
	}

	int shapeCount = RCount( r, INT_MAX / (int)sizeof( b2fShape ), sizeof( b2fShape ) );
	b2Array_Resize( f->shapes, shapeCount );
	R( r, f->shapes.data, (size_t)shapeCount * sizeof( b2fShape ) );
	int bodyCount = RCount( r, INT_MAX / (int)sizeof( b2fBody ), sizeof( b2fBody ) );
	b2Array_Resize( f->bodies, bodyCount );
	R( r, f->bodies.data, (size_t)bodyCount * sizeof( b2fBody ) );
	for ( int i = 0; i < f->bodies.count; ++i )
	{
		f->bodies.data[i].id.world0 = world0;
	}
	for ( int i = 0; i < f->shapes.count; ++i )
	{
		r->ok = r->ok && 0 <= f->shapes.data[i].bodyIndex && f->shapes.data[i].bodyIndex < f->bodies.count;
	}
	uint8_t flags[2];
	R( r, flags, sizeof( flags ) );
	f->shapesFiltered = flags[0] != 0;
	f->shapesDropped = flags[1] != 0;

	int recordCount = RCount( r, INT_MAX / (int)sizeof( b2fBodyRecord ), sizeof( b2fBodyRecord ) );
	b2Array_Resize( f->records, recordCount );
	R( r, f->records.data, (size_t)recordCount * sizeof( b2fBodyRecord ) );
	for ( int i = 0; i < f->records.count; ++i )
	{
		b2fBodyRecord* record = f->records.data + i;
		record->id.world0 = world0;
		if ( B2_IS_NON_NULL( record->state.liquidId ) )
		{
			record->state.liquidId.world0 = world0;
		}
	}
	int lookup = RCount( r, INT_MAX / (int)sizeof( int ), sizeof( int ) );
	b2Array_Resize( f->recordOfBody, lookup );
	R( r, f->recordOfBody.data, (size_t)lookup * sizeof( int ) );
	for ( int i = 0; i < lookup; ++i )
	{
		r->ok = r->ok && 0 <= f->recordOfBody.data[i] && f->recordOfBody.data[i] <= f->records.count;
	}
	int sensorCount = RCount( r, INT_MAX / (int)sizeof( b2fSensor ), sizeof( b2fSensor ) );
	b2Array_Resize( f->sensors, sensorCount );
	R( r, f->sensors.data, (size_t)sensorCount * sizeof( b2fSensor ) );
	for ( int i = 0; i < f->sensors.count; ++i )
	{
		f->sensors.data[i].id.world0 = world0;
	}

	if ( r->ok == false )
	{
		b2Log( "snapshot fluid section corrupt\n" );
		b2fClearFluidState( world );
		return false;
	}
	*cursor = r->cursor;
	return true;
}

static uint64_t MixBits( uint64_t hash, const void* data, size_t size )
{
	// Four bytes at a time, as b2HashWorldState mixes floats
	const uint8_t* p = data;
	for ( size_t k = 0; k + 4 <= size; k += 4 )
	{
		uint32_t bits;
		memcpy( &bits, p + k, 4 );
		hash = ( hash ^ (uint64_t)bits ) * B2_SNAP_FNV_PRIME;
	}
	for ( size_t k = size & ~(size_t)3; k < size; ++k )
	{
		hash = ( hash ^ (uint64_t)p[k] ) * B2_SNAP_FNV_PRIME;
	}
	return hash;
}

uint64_t b2fHashFluid( const b2World* world, uint64_t hash, bool deep )
{
	const b2fFluid* f = world->fluid;
	if ( f == NULL )
	{
		return hash;
	}

	hash = MixBits( hash, &f->count, sizeof( int ) );
	hash = MixBits( hash, &f->nextParticleId, sizeof( int ) );
	for ( int i = 0; i < f->count; ++i )
	{
		hash = b2FnvMixPosition( hash, f->pos[i] );
		hash = MixBits( hash, f->vx + i, sizeof( float ) );
		hash = MixBits( hash, f->vy + i, sizeof( float ) );
		hash = MixBits( hash, f->size + i, sizeof( float ) );
		hash = MixBits( hash, f->rho + i, sizeof( float ) );
		hash = MixBits( hash, f->id + i, sizeof( int ) );
		hash = ( hash ^ (uint64_t)f->liquid[i] ) * B2_SNAP_FNV_PRIME;
		hash = MixBits( hash, f->sleepTime + i, sizeof( float ) );
		hash = ( hash ^ (uint64_t)f->asleep[i] ) * B2_SNAP_FNV_PRIME;
	}

	// The liquid keeps bodies awake and wakes the ones it reaches, through sleepTime and the solver sets
	for ( int i = 0; i < world->bodies.count; ++i )
	{
		const b2Body* body = world->bodies.data + i;
		if ( body->id != i )
		{
			continue;
		}
		hash = MixBits( hash, &body->setIndex, sizeof( int ) );
		hash = MixBits( hash, &body->sleepTime, sizeof( float ) );
	}

	if ( deep == false )
	{
		return hash;
	}

	hash = MixBits( hash, f->calm, (size_t)f->count * sizeof( b2Vec2 ) );
	if ( f->speedMemory != NULL )
	{
		hash = MixBits( hash, f->speedMemory, (size_t)f->count * sizeof( float ) );
	}
	if ( f->springFormed != NULL )
	{
		hash = MixBits( hash, f->springFormed, (size_t)f->count * sizeof( bool ) );
		hash = MixBits( hash, &f->springs.count, sizeof( int ) );
		hash = MixBits( hash, f->springs.data, (size_t)f->springs.count * sizeof( b2fSpring ) );
	}
	for ( int l = 0; l < f->liquidCount; ++l )
	{
		hash = ( hash ^ (uint64_t)f->liquids[l].alive ) * B2_SNAP_FNV_PRIME;
		hash = ( hash ^ (uint64_t)f->liquids[l].generation ) * B2_SNAP_FNV_PRIME;
	}
	for ( int e = 0; e < f->emitterCount; ++e )
	{
		const b2fEmitter* emitter = f->emitters + e;
		hash = ( hash ^ (uint64_t)emitter->alive ) * B2_SNAP_FNV_PRIME;
		hash = MixBits( hash, &emitter->travel, sizeof( float ) );
		hash = MixBits( hash, &emitter->rows, sizeof( int ) );
	}
	hash = MixBits( hash, &f->coarsenLiquids, sizeof( uint32_t ) );
	for ( int r = 0; r < f->records.count; ++r )
	{
		const b2fBodyRecord* record = f->records.data + r;
		hash = MixBits( hash, &record->id.index1, sizeof( int ) );
		hash = ( hash ^ (uint64_t)record->state.inLiquid ) * B2_SNAP_FNV_PRIME;
		hash = MixBits( hash, &record->run, sizeof( float ) );
	}
	for ( int s = 0; s < f->sensors.count; ++s )
	{
		hash = MixBits( hash, &f->sensors.data[s].id.index1, sizeof( int ) );
		hash = MixBits( hash, &f->sensors.data[s].area, sizeof( float ) );
	}
	return hash;
}
