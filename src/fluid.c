// SPDX-License-Identifier: MIT
// Creating a liquid, its settings, what it reports, and the order of passes in each substep.
#include "fluid_internal.h"

#include "core.h"
#include "fluid.h"
#include "physics_world.h"

b2LiquidDef b2DefaultLiquidDef( void )
{
	b2LiquidDef def = { 0 };
	def.density = 1.0f;
	def.tuning.stiffness = 0.04f;
	def.tuning.nearStiffness = 0.1f;
	def.tuning.tensilePressure = 0.5f;
	def.cohesion = 0.04f;
	def.adhesion = 0.025f;
	def.viscosity = 0.5f;
	def.friction = 0.1f;
	def.gravityScale = 1.0f;
	def.tuning.buoyancyAssist = 1.0f;
	def.tuning.addedMass = 1.0f;
	def.springStiffness = 0.0f;
	def.plasticity = 0.5f;
	def.yieldRatio = 0.1f;
	def.tuning.adaptiveRatio = 10.0f;
	def.tuning.adaptiveSpeed = 0.0f;
	def.tuning.adaptiveDepthLimit = false;
	def.tuning.adaptiveTolerance = 0.3f;
	def.tuning.adaptiveLimit = 3.0f;
	def.tuning.adaptiveRate = 3.0f;
	def.categoryBits = B2_DEFAULT_CATEGORY_BITS;
	def.maskBits = B2_DEFAULT_MASK_BITS;
	return def;
}

// The values the sample scenes use for these liquids
b2LiquidDef b2MakeOilDef( void )
{
	b2LiquidDef def = b2DefaultLiquidDef();
	def.density = 0.6f;
	def.cohesion = 0.1f;
	return def;
}

b2LiquidDef b2MakeHoneyDef( void )
{
	b2LiquidDef def = b2DefaultLiquidDef();
	def.density = 1.4f;
	def.cohesion = 0.2f;
	def.friction = 0.3f;
	def.viscosity = 20.0f;
	return def;
}

b2LiquidDef b2MakeMercuryDef( void )
{
	b2LiquidDef def = b2DefaultLiquidDef();
	def.density = 13.6f;
	def.cohesion = 0.1f;
	return def;
}

b2LiquidDef b2MakeSlimeDef( void )
{
	b2LiquidDef def = b2DefaultLiquidDef();
	def.cohesion = 0.25f;
	def.viscosity = 10.0f;
	def.friction = 0.5f;
	return def;
}

b2LiquidDef b2MakeJellyDef( void )
{
	b2LiquidDef def = b2DefaultLiquidDef();
	def.springStiffness = 10000.0f;
	def.plasticity = 0.0f;
	return def;
}

b2LiquidDef b2MakeDoughDef( void )
{
	b2LiquidDef def = b2DefaultLiquidDef();
	def.springStiffness = 10000.0f;
	def.plasticity = 1.0f;
	def.yieldRatio = 0.1f;
	return def;
}

b2FluidDef b2DefaultFluidDef( void )
{
	b2FluidDef def = { 0 };
	def.particleSpacing = 0.08f;
	def.adaptiveDepth = 12.0f;
	def.adaptiveFullBox = 0.9f;
	def.capacity = 100000;
	def.maxCapacity = 1000000;
	return def;
}

// Rebuilds what the passes read from the liquids' settings. A destroyed liquid's slot keeps its last settings, which
// nothing reads since no particle is of it.
static void UpdateLiquids( b2fFluid* f )
{
	float spacing = f->spacing;
	f->adaptive = false;
	f->springy = false;
	for ( int a = 0; a < f->liquidCount; ++a )
	{
		b2fLiquid* liquid = f->liquids + a;
		liquid->mass = liquid->def.density * 0.8660254f * spacing * spacing / B2F_REST_FILL;
		f->cohesionBetween[a][a] = liquid->def.cohesion;
		if ( liquid->alive )
		{
			f->adaptive = f->adaptive || liquid->def.tuning.adaptiveRatio > 1.0f;
			f->springy = f->springy || liquid->def.springStiffness > 0.0f;
		}
	}

	// Adaptive sizes count mass in finest particles of the first liquid, as a measure of area. Without one, any
	// mass does.
	f->mass = f->liquidCount > 0 && f->liquids[0].alive ? f->liquids[0].mass : 0.8660254f * spacing * spacing / B2F_REST_FILL;

	if ( f->springy && f->springs == NULL )
	{
		size_t n = (size_t)f->capacity + 1, slots = 1;
		while ( slots < 2 * n )
		{
			slots *= 2;
		}
		f->springs = calloc( n, sizeof( b2fSprings ) );
		f->sortedSprings = calloc( n, sizeof( b2fSprings ) );
		f->idMap = malloc( slots * sizeof( b2fIdSlot ) );
		f->newIndex = calloc( n, sizeof( int ) );
	}

	for ( int a = 0; a < f->liquidCount; ++a )
	{
		const b2LiquidDef* da = &f->liquids[a].def;
		for ( int b = 0; b < f->liquidCount; ++b )
		{
			const b2LiquidDef* db = &f->liquids[b].def;
			float sum = da->density + db->density;
			f->liquidPairs[a][b] = ( b2fLiquidPair ){
				.cohesion = f->cohesionBetween[a][b],
				.weight = sum > 0.0f ? 2.0f * db->density / sum : 1.0f,
				.viscosity = 0.5f * ( da->viscosity + db->viscosity ),
				.quadraticViscosity = 0.5f * ( da->tuning.quadraticViscosity + db->tuning.quadraticViscosity ),
			};
		}
	}

	// Which liquids push on which. A destroyed slot sees everything, which nothing reads since no particle is of it.
	f->liveLiquids = 0;
	f->liquidsFiltered = false;
	for ( int a = 0; a < B2F_MAX_LIQUIDS; ++a )
	{
		f->liquidSees[a] = UINT32_MAX;
		if ( a < f->liquidCount && f->liquids[a].alive )
		{
			f->liveLiquids |= 1u << a;
		}
	}
	for ( int a = 0; a < f->liquidCount; ++a )
	{
		const b2LiquidDef* da = &f->liquids[a].def;
		for ( int b = 0; b < f->liquidCount; ++b )
		{
			const b2LiquidDef* db = &f->liquids[b].def;
			bool live = f->liquids[a].alive && f->liquids[b].alive;
			if ( live && a != b && ( ( da->maskBits & db->categoryBits ) == 0 || ( da->categoryBits & db->maskBits ) == 0 ) )
			{
				f->liquidSees[a] &= ~( 1u << b );
				f->liquidsFiltered = true;
			}
		}
	}
}

uint32_t b2fLiquidsCollidingWith( const b2fFluid* f, b2Filter filter )
{
	uint32_t liquids = 0;
	for ( int l = 0; l < f->liquidCount; ++l )
	{
		const b2LiquidDef* def = &f->liquids[l].def;
		if ( f->liquids[l].alive && ( def->maskBits & filter.categoryBits ) != 0 && ( def->categoryBits & filter.maskBits ) != 0 )
		{
			liquids |= 1u << l;
		}
	}
	return liquids;
}

static float LatticeDensity( float spacing, float radius )
{
	float sum = 0.0f;
	float rowSpacing = 0.8660254f * spacing;
	for ( int row = -32; row <= 32; ++row )
	{
		for ( int column = -32; column <= 32; ++column )
		{
			float dx = ( (float)column + ( row & 1 ? 0.5f : 0.0f ) ) * spacing;
			float dy = (float)row * rowSpacing;
			float d = sqrtf( dx * dx + dy * dy );
			if ( d > 0.0f && d < radius )
			{
				float q = 1.0f - d / radius;
				sum += q * q;
			}
		}
	}
	return sum;
}

// The world holds its one fluid. An id is good while the world is and until the fluid is destroyed.
b2fFluid* b2fGetFluid( b2FluidId fluidId )
{
	b2World* world = b2GetWorld( fluidId.world0 );
	B2_ASSERT( world->fluid != NULL && fluidId.index1 == world->fluidSerial && fluidId.generation == world->generation );
	return world->fluid;
}

b2FluidId b2CreateFluid( b2WorldId worldId, const b2FluidDef* def )
{
	b2World* world = b2GetWorldFromId( worldId );
	B2_ASSERT( world->locked == false );
	if ( world->locked || world->fluid != NULL )
	{
		return b2_nullFluidId;
	}

	b2fFluid* f = calloc( 1, sizeof( b2fFluid ) );
	f->worldId = worldId;
	f->def = *def;
	f->capacity = def->capacity > 0 ? def->capacity : 0;
	f->maxCapacity = def->maxCapacity > f->capacity ? def->maxCapacity : f->capacity;
	f->spacing = def->particleSpacing / B2F_REST_PITCH;
	f->radius = 2.0f * f->spacing;
	f->cellSize = 1.1f * f->radius;
	UpdateLiquids( f );
	f->restDensity = LatticeDensity( f->spacing, f->radius );
	f->maxRadius = f->radius;
	f->moveLimit = B2F_MOVE_LIMIT;

	size_t n = (size_t)f->capacity + 1;
#define B2F_ALLOC_FIELD( type, name ) f->name = calloc( n, sizeof( type ) );
#define B2F_ALLOC_SORTED( type, name ) f->sorted.name = calloc( n, sizeof( type ) );
	B2F_PARTICLE_FIELDS( B2F_ALLOC_FIELD )
	B2F_PARTICLE_FIELDS( B2F_ALLOC_SORTED )
	B2F_WORK_FIELDS( B2F_ALLOC_FIELD )
#undef B2F_ALLOC_FIELD
#undef B2F_ALLOC_SORTED
	f->contacts = calloc( n, B2F_CONTACTS_PER_PARTICLE * sizeof( b2fContact ) );
	f->pairs = malloc( n * B2F_PAIRS_PER_PARTICLE * sizeof( b2fPair ) );
	f->restSizes = calloc( n, sizeof( float ) );
	b2fBuildWallTable( f );

	world->fluid = f;
	world->fluidSerial += 1;
	return ( b2FluidId ){ world->fluidSerial, world->worldId, world->generation };
}

void b2DestroyFluid( b2FluidId fluidId )
{
	b2World* world = b2GetWorld( fluidId.world0 );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return;
	}

	b2FreeFluid( b2fGetFluid( fluidId ) );
	world->fluid = NULL;
}

void b2FreeFluid( b2fFluid* f )
{
#define B2F_FREE_FIELD( type, name ) free( f->name );
#define B2F_FREE_SORTED( type, name ) free( f->sorted.name );
	B2F_PARTICLE_FIELDS( B2F_FREE_FIELD )
	B2F_PARTICLE_FIELDS( B2F_FREE_SORTED )
	B2F_WORK_FIELDS( B2F_FREE_FIELD )
#undef B2F_FREE_FIELD
#undef B2F_FREE_SORTED
	free( f->contacts );
	free( f->restSizes );
	free( f->pairs );
	free( f->springs );
	free( f->sortedSprings );
	free( f->idMap );
	free( f->newIndex );
	free( f->cellStart );
	free( f->blockSize );
	free( f->blockReach );
	free( f->shapes );
	free( f->bodies );
	free( f->reactions );
	free( f->patches );
	free( f->boxMass );
	free( f->boxPx );
	free( f->boxPy );
	free( f->boxWeight );
	free( f->boxMixed );
	free( f->boxLiquid );
	free( f->boxPressure );
	free( f->boxDepth );
	free( f->boxFlow );
	b2fFreeState( f );
	free( f );
}

void b2Fluid_SetDef( b2FluidId fluidId, const b2FluidDef* def )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	float spacing = f->def.particleSpacing;
	int capacity = f->def.capacity;
	f->def = *def;
	f->def.particleSpacing = spacing;
	f->def.capacity = capacity;
	f->maxCapacity = def->maxCapacity > f->capacity ? def->maxCapacity : f->capacity;
}

int b2Fluid_GetCapacity( b2FluidId fluidId )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	return f->capacity;
}

// Resizes an array of count + 1 values to newCount + 1, zeroing the new part when the array started zeroed
static void* Resize( void* array, int count, int newCount, size_t size, bool zero )
{
	char* p = realloc( array, ( (size_t)newCount + 1 ) * size );
	B2_ASSERT( p != NULL );
	if ( zero )
	{
		memset( p + ( (size_t)count + 1 ) * size, 0, (size_t)( newCount - count ) * size );
	}
	return p;
}

// Every per-particle array is carried at the new size with what it holds, and the new room is as b2CreateFluid
// leaves it, so a fluid that grew is the same as one made that large. Nothing reads capacity in the passes, so the
// liquid moves the same.
bool b2fGrow( b2fFluid* f, int needed )
{
	if ( needed <= f->capacity )
	{
		return true;
	}
	if ( needed > f->maxCapacity )
	{
		return false;
	}

	int64_t grown = f->capacity > 0 ? f->capacity : 64;
	while ( grown < needed )
	{
		grown *= 2;
	}
	int old = f->capacity, capacity = grown < f->maxCapacity ? (int)grown : f->maxCapacity;

#define B2F_GROW_FIELD( type, name ) f->name = Resize( f->name, old, capacity, sizeof( type ), true );
#define B2F_GROW_SORTED( type, name ) f->sorted.name = Resize( f->sorted.name, old, capacity, sizeof( type ), true );
	B2F_PARTICLE_FIELDS( B2F_GROW_FIELD )
	B2F_PARTICLE_FIELDS( B2F_GROW_SORTED )
	B2F_WORK_FIELDS( B2F_GROW_FIELD )
#undef B2F_GROW_FIELD
#undef B2F_GROW_SORTED
	f->contacts = Resize( f->contacts, old, capacity, B2F_CONTACTS_PER_PARTICLE * sizeof( b2fContact ), true );
	f->pairs = Resize( f->pairs, old, capacity, B2F_PAIRS_PER_PARTICLE * sizeof( b2fPair ), false );
	f->restSizes = Resize( f->restSizes, old, capacity, sizeof( float ), true );

	// The spring arrays, made once some liquid has springs, and the id table, which is cleared before each use
	if ( f->springs != NULL )
	{
		f->springs = Resize( f->springs, old, capacity, sizeof( b2fSprings ), true );
		f->sortedSprings = Resize( f->sortedSprings, old, capacity, sizeof( b2fSprings ), true );
		f->newIndex = Resize( f->newIndex, old, capacity, sizeof( int ), true );
		size_t n = (size_t)capacity + 1, slots = 1;
		while ( slots < 2 * n )
		{
			slots *= 2;
		}
		free( f->idMap );
		f->idMap = malloc( slots * sizeof( b2fIdSlot ) );
	}

	f->capacity = capacity;
	return true;
}

// Compaction keeps the order, so each cell's run of particles stays a run and the grid from the last sort holds with
// its starts moved down. The size pass then still runs at the next step, which reads the grid, the cells and search
// windows, and the densities of the last substep, so those go with each particle here.
int b2fRemoveMarked( b2fFluid* f )
{
	int n = f->count, kept = 0;
	bool keepGrid = f->gridValid && n > 0;
	int* before = keepGrid ? malloc( ( (size_t)n + 1 ) * sizeof( int ) ) : NULL;
	for ( int i = 0; i < n; ++i )
	{
		if ( keepGrid )
		{
			before[i] = kept;
		}
		if ( f->adaptUsed[i] )
		{
			continue;
		}
		if ( kept != i )
		{
			b2fCopyParticle( f, kept, i );
			f->rho[kept] = f->rho[i];
			f->cell[kept] = f->cell[i];
			f->window[kept] = f->window[i];
		}
		kept += 1;
	}

	int removed = n - kept;
	if ( removed > 0 )
	{
		for ( int i = kept; i < n; ++i )
		{
			b2fClearParticle( f, i );
		}
		f->count = kept;
		f->restSizesStale = true;

		if ( keepGrid )
		{
			before[n] = kept;
			int entries = f->gridWidth * f->gridHeight + 2;
			for ( int c = 0; c < entries; ++c )
			{
				f->cellStart[c] = before[f->cellStart[c]];
			}
		}
	}
	free( before );
	return removed;
}

bool b2fWorldLocked( b2FluidId fluidId )
{
	b2World* world = b2GetWorld( fluidId.world0 );
	B2_ASSERT( world->locked == false );
	return world->locked;
}

int b2fPickLiquid( b2fFluid* f, b2LiquidId liquidId )
{
	if ( B2_IS_NULL( liquidId ) )
	{
		return -1;
	}
	b2fFluid* other;
	int liquid = b2fGetLiquid( liquidId, &other );
	B2_ASSERT( other == f );
	(void)f;
	return liquid;
}

int b2fMarkInside( b2fFluid* f, int liquid, const b2fOutline* outline, int maxCount )
{
	int marked = 0;
	for ( int i = 0; i < f->count; ++i )
	{
		bool inside = marked < maxCount && ( liquid < 0 || f->liquid[i] == liquid ) && b2fInsideOutline( outline, f->pos[i] );
		f->adaptUsed[i] = inside ? 1 : 0;
		marked += inside ? 1 : 0;
	}
	return marked;
}

int b2Fluid_RemoveInBox( b2FluidId fluidId, b2LiquidId liquidId, b2Pos lower, b2Pos upper, int maxCount )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	if ( b2fWorldLocked( fluidId ) )
	{
		return 0;
	}

	b2fOutline box = b2fMakeBoxOutline( lower, upper );
	b2fMarkInside( f, b2fPickLiquid( f, liquidId ), &box, maxCount );
	return b2fRemoveMarked( f );
}

int b2Fluid_RemoveInShape( b2FluidId fluidId, b2LiquidId liquidId, b2ShapeId shapeId, int maxCount )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	if ( b2fWorldLocked( fluidId ) )
	{
		return 0;
	}

	int liquid = b2fPickLiquid( f, liquidId );
	b2fOutline outline;
	if ( b2fMakeOutline( shapeId, &outline ) == false )
	{
		return 0;
	}

	b2fMarkInside( f, liquid, &outline, maxCount );
	return b2fRemoveMarked( f );
}

b2LiquidId b2fMakeLiquidId( const b2fFluid* f, int index )
{
	return ( b2LiquidId ){ index + 1, (uint16_t)( f->worldId.index1 - 1 ), f->liquids[index].generation };
}

int b2fGetLiquid( b2LiquidId liquidId, b2fFluid** fluid )
{
	b2World* world = b2GetWorld( liquidId.world0 );
	b2fFluid* f = world->fluid;
	int index = liquidId.index1 - 1;
	B2_ASSERT( f != NULL && 0 <= index && index < f->liquidCount );
	B2_ASSERT( f->liquids[index].alive && liquidId.generation == f->liquids[index].generation );
	*fluid = f;
	return index;
}

bool b2Liquid_IsValid( b2LiquidId liquidId )
{
	const b2World* world = b2FindWorld( liquidId.world0 );
	if ( world == NULL || world->fluid == NULL )
	{
		return false;
	}

	const b2fFluid* f = world->fluid;
	int index = liquidId.index1 - 1;
	return 0 <= index && index < f->liquidCount && f->liquids[index].alive &&
		   liquidId.generation == f->liquids[index].generation;
}

b2LiquidId b2CreateLiquid( b2FluidId fluidId, const b2LiquidDef* def )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	b2World* world = b2GetWorld( fluidId.world0 );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return b2_nullLiquidId;
	}

	// The lowest free slot, so that the first liquid of a fluid is the one the single-liquid passes read
	int index = 0;
	while ( index < f->liquidCount && f->liquids[index].alive )
	{
		index += 1;
	}
	if ( index == B2F_MAX_LIQUIDS )
	{
		return b2_nullLiquidId;
	}

	f->liquidCount = index < f->liquidCount ? f->liquidCount : index + 1;
	f->liquids[index].def = *def;
	f->liquids[index].alive = true;
	world->liquidSerial += 1;
	f->liquids[index].generation = world->liquidSerial;
	for ( int other = 0; other < B2F_MAX_LIQUIDS; ++other )
	{
		f->cohesionBetween[index][other] = 0.0f;
		f->cohesionBetween[other][index] = 0.0f;
	}
	UpdateLiquids( f );
	return ( b2LiquidId ){ index + 1, fluidId.world0, f->liquids[index].generation };
}

void b2DestroyLiquid( b2LiquidId liquidId )
{
	b2fFluid* f;
	int index = b2fGetLiquid( liquidId, &f );
	b2World* world = b2GetWorld( liquidId.world0 );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return;
	}

	for ( int i = 0; i < f->count; ++i )
	{
		f->adaptUsed[i] = f->liquid[i] == index ? 1 : 0;
	}
	b2fRemoveMarked( f );
	b2fDestroyEmitters( f, index );

	f->liquids[index].alive = false;
	while ( f->liquidCount > 0 && f->liquids[f->liquidCount - 1].alive == false )
	{
		f->liquidCount -= 1;
	}
	UpdateLiquids( f );
}

void b2Liquid_SetDef( b2LiquidId liquidId, const b2LiquidDef* def )
{
	b2fFluid* f;
	int index = b2fGetLiquid( liquidId, &f );
	f->liquids[index].def = *def;
	UpdateLiquids( f );
}

b2LiquidDef b2Liquid_GetDef( b2LiquidId liquidId )
{
	b2fFluid* f;
	int index = b2fGetLiquid( liquidId, &f );
	return f->liquids[index].def;
}

int b2Liquid_GetIndex( b2LiquidId liquidId )
{
	b2fFluid* f;
	return b2fGetLiquid( liquidId, &f );
}

int b2Fluid_GetLiquidCount( b2FluidId fluidId )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	int count = 0;
	for ( int i = 0; i < f->liquidCount; ++i )
	{
		count += f->liquids[i].alive ? 1 : 0;
	}
	return count;
}

void b2Liquid_SetCohesionBetween( b2LiquidId liquidA, b2LiquidId liquidB, float cohesion )
{
	b2fFluid *f, *other;
	int a = b2fGetLiquid( liquidA, &f ), b = b2fGetLiquid( liquidB, &other );
	B2_ASSERT( f == other );
	if ( f == other && a != b )
	{
		f->cohesionBetween[a][b] = cohesion;
		f->cohesionBetween[b][a] = cohesion;
		UpdateLiquids( f );
	}
}

int b2Fluid_GetParticleCount( b2FluidId fluidId )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	return f->count;
}

b2FluidParticles b2Fluid_GetParticles( b2FluidId fluidId )
{
	b2fFluid* f = b2fGetFluid( fluidId );

	// Inside, a size is a spacing of the solver's lattice; the caller gets how far apart such particles rest. This is
	// worked out once after each change rather than on every call.
	if ( f->restSizesStale )
	{
		for ( int i = 0; i < f->count; ++i )
		{
			f->restSizes[i] = B2F_REST_PITCH * f->size[i];
		}
		f->restSizesStale = false;
	}

	return ( b2FluidParticles ){
		.count = f->count,
		.positions = f->pos,
		.velocitiesX = f->vx,
		.velocitiesY = f->vy,
		.sizes = f->restSizes,
		.liquids = f->liquid,
		.densities = f->rho,
		.ids = f->id,
		.restDensity = f->restDensity,
	};
}

b2FluidProfile b2Fluid_GetProfile( b2FluidId fluidId )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	return f->profile;
}

void b2PrepareFluid( b2fFluid* f, float timeStep )
{
	uint64_t ticks = b2GetTicks();

	// This reads where shapes and bodies were last substep, so it has to come before they are gathered again
	b2fAdaptSizes( f );
	// After the sizes, which need every particle to have a grid cell from the last sort, as new ones have not
	b2fStepEmitters( f, timeStep );
	f->restSizesStale = true;
	b2fGatherShapes( f );

	f->stepTime = 0.0f;
	f->stepSubSteps = 0;
	memset( &f->profile, 0, sizeof( f->profile ) );
	f->profile.sizeChanges = f->sizeChanges;
	f->profile.prepare = b2GetMilliseconds( ticks );
	f->profile.measure = f->measureTime;
}

void b2FinishFluid( b2fFluid* f )
{
	b2fFinishStep( f );
}

void b2FluidSubStep( b2SubStepContext* context, int subStepIndex, int subStepCount, float h, b2fFluid* f )
{
	(void)subStepCount;
	f->checkSprings = subStepIndex == 0;
	// The sort below reorders particles
	f->restSizesStale = true;
	f->stepTime += h;
	f->stepSubSteps += 1;
	if ( f->count == 0 )
	{
		return;
	}

	uint64_t start = b2GetTicks();
	uint64_t ticks = start;

	b2Vec2 gravity = b2World_GetGravity( f->worldId );
	f->h = h;
	f->worldGravity = gravity;

	b2fUpdateBodies( f, context );
	b2fPredict( f, context );
	b2fSortParticles( f, context );
	f->profile.sort += b2GetMillisecondsAndReset( &ticks );

	b2fFindContacts( f, context );
	if ( f->springy )
	{
		b2fRelaxSprings( f, context );
	}
	b2fRelaxDensity( f, context );
	f->profile.density += b2GetMillisecondsAndReset( &ticks );

	b2fCollide( f, context );
	b2fUpdateVelocities( f, context );
	f->profile.collide += b2GetMillisecondsAndReset( &ticks );
	f->profile.total += b2GetMilliseconds( start );
}
