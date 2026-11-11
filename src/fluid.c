// SPDX-License-Identifier: MIT
// Creating a liquid, its settings, what it reports, and the order of passes in each substep.
#include "fluid_internal.h"

#include "core.h"
#include "fluid.h"
#include "physics_world.h"

b2LiquidDef b2DefaultLiquidDef( void )
{
	b2LiquidDef def = { 0 };
	def.massDensity = 1.0f;
	def.stiffness = 0.04f;
	def.nearStiffness = 0.1f;
	def.tensilePressure = 0.5f;
	def.cohesion = 0.04f;
	def.adhesion = 0.025f;
	def.linearViscosity = 0.5f;
	def.friction = 0.1f;
	def.gravityScale = 1.0f;
	def.contactPressure = 1.0f;
	def.addedMass = 1.0f;
	def.springStiffness = 0.0f;
	def.plasticity = 0.5f;
	def.yieldRatio = 0.1f;
	def.adaptiveRatio = 10.0f;
	def.adaptiveSpeed = 0.0f;
	def.adaptiveDepthLimit = false;
	def.adaptiveTolerance = 0.3f;
	def.adaptiveLimit = 3.0f;
	def.adaptiveRate = 0.05f;
	return def;
}

b2FluidDef b2DefaultFluidDef( void )
{
	b2FluidDef def = { 0 };
	def.particleSpacing = 0.08f;
	def.adaptiveDepth = 12.0f;
	def.adaptiveFullBox = 0.9f;
	def.capacity = 100000;
	def.liquid = b2DefaultLiquidDef();
	return def;
}

// Rebuilds what the passes read from the liquids' settings
static void UpdateLiquids( b2fFluid* f )
{
	float spacing = f->spacing;
	f->adaptive = false;
	for ( int a = 0; a < f->liquidCount; ++a )
	{
		b2fLiquid* liquid = f->liquids + a;
		liquid->mass = liquid->def.massDensity * 0.8660254f * spacing * spacing / B2F_REST_FILL;
		f->adaptive = f->adaptive || liquid->def.adaptiveRatio > 1.0f;
		f->cohesionBetween[a][a] = liquid->def.cohesion;
	}
	f->mass = f->liquids[0].mass;

	f->springy = false;
	for ( int a = 0; a < f->liquidCount; ++a )
	{
		f->springy = f->springy || f->liquids[a].def.springStiffness > 0.0f;
	}
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
	f->def.liquid = f->liquids[0].def;

	for ( int a = 0; a < f->liquidCount; ++a )
	{
		const b2LiquidDef* da = &f->liquids[a].def;
		for ( int b = 0; b < f->liquidCount; ++b )
		{
			const b2LiquidDef* db = &f->liquids[b].def;
			float sum = da->massDensity + db->massDensity;
			f->liquidPairs[a][b] = ( b2fLiquidPair ){
				.cohesion = f->cohesionBetween[a][b],
				.weight = sum > 0.0f ? 2.0f * db->massDensity / sum : 1.0f,
				.linearViscosity = 0.5f * ( da->linearViscosity + db->linearViscosity ),
				.quadraticViscosity = 0.5f * ( da->quadraticViscosity + db->quadraticViscosity ),
			};
		}
	}
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
	B2_ASSERT( world->fluid == NULL );
	if ( world->locked || world->fluid != NULL )
	{
		return b2_nullFluidId;
	}

	b2fFluid* f = calloc( 1, sizeof( b2fFluid ) );
	f->worldId = worldId;
	f->def = *def;
	f->capacity = def->capacity;
	f->spacing = def->particleSpacing / B2F_REST_PITCH;
	f->radius = 2.0f * f->spacing;
	f->cellSize = 1.1f * f->radius;
	f->margin = 0.5f * f->spacing;
	f->liquids[0].def = def->liquid;
	f->liquidCount = 1;
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
	f->liquids[0].def = def->liquid;
	UpdateLiquids( f );
}

int b2Fluid_CreateLiquid( b2FluidId fluidId, const b2LiquidDef* def )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	if ( f->liquidCount == B2F_MAX_LIQUIDS )
	{
		return -1;
	}
	int index = f->liquidCount++;
	f->liquids[index].def = *def;
	UpdateLiquids( f );
	return index;
}

void b2Fluid_SetLiquid( b2FluidId fluidId, int liquid, const b2LiquidDef* def )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	if ( 0 <= liquid && liquid < f->liquidCount )
	{
		f->liquids[liquid].def = *def;
		UpdateLiquids( f );
	}
}

int b2Fluid_GetLiquidCount( b2FluidId fluidId )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	return f->liquidCount;
}

void b2Fluid_SetCohesionBetween( b2FluidId fluidId, int a, int b, float cohesion )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	if ( 0 <= a && a < f->liquidCount && 0 <= b && b < f->liquidCount && a != b )
	{
		f->cohesionBetween[a][b] = cohesion;
		f->cohesionBetween[b][a] = cohesion;
		UpdateLiquids( f );
	}
}

float b2Fluid_GetMovedMass( b2FluidId fluidId )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	return f->movedMass;
}

int b2Fluid_GetParticleCount( b2FluidId fluidId )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	return f->count;
}
const b2Pos* b2Fluid_GetPositions( b2FluidId fluidId )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	return f->pos;
}
const float* b2Fluid_GetVelocitiesX( b2FluidId fluidId )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	return f->vx;
}
const float* b2Fluid_GetVelocitiesY( b2FluidId fluidId )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	return f->vy;
}
const float* b2Fluid_GetSizes( b2FluidId fluidId )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	// Inside, a size is a spacing of the solver's lattice; the caller gets how far apart such particles rest
	if ( f->restSizeCapacity < f->count )
	{
		f->restSizeCapacity = f->capacity;
		f->restSizes = realloc( f->restSizes, (size_t)f->restSizeCapacity * sizeof( float ) );
	}
	for ( int i = 0; i < f->count; ++i )
	{
		f->restSizes[i] = B2F_REST_PITCH * f->size[i];
	}
	return f->restSizes;
}
const unsigned char* b2Fluid_GetLiquids( b2FluidId fluidId )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	return f->liquid;
}
const float* b2Fluid_GetDensities( b2FluidId fluidId )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	return f->rho;
}
float b2Fluid_GetRestDensity( b2FluidId fluidId )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	return f->restDensity;
}
b2FluidProfile b2Fluid_GetProfile( b2FluidId fluidId )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	return f->profile;
}

void b2PrepareFluid( b2fFluid* f )
{
	uint64_t ticks = b2GetTicks();

	// This reads where shapes and bodies were last substep, so it has to come before they are gathered again
	b2fAdaptSizes( f );
	b2fGatherShapes( f );

	memset( &f->profile, 0, sizeof( f->profile ) );
	f->profile.sizeChanges = f->sizeChanges;
	f->profile.prepare = b2GetMilliseconds( ticks );
	f->profile.measure = f->measureTime;
}

void b2FluidSubStep( b2SubStepContext* context, int subStepIndex, int subStepCount, float h, b2fFluid* f )
{
	(void)subStepCount;
	f->checkSprings = subStepIndex == 0;
	if ( f->count == 0 )
	{
		return;
	}

	uint64_t start = b2GetTicks();
	uint64_t ticks = start;

	b2Vec2 gravity = b2World_GetGravity( f->worldId );
	f->h = h;
	f->worldGravity = gravity;
	f->gravityX = f->liquids[0].def.gravityScale * gravity.x;
	f->gravityY = f->liquids[0].def.gravityScale * gravity.y;

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
