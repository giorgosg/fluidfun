// SPDX-License-Identifier: MIT
// Creating a liquid, its settings, what it reports, and the order of passes in each substep.
#include "fluid_internal.h"

b2fFluidDef b2fDefaultFluidDef( void )
{
	b2fFluidDef def = { 0 };
	def.particleSpacing = 0.07f;
	def.massDensity = 1.0f;
	def.stiffness = 0.04f;
	def.nearStiffness = 0.1f;
	def.tensilePressure = 0.0f;
	def.cohesion = 0.1f;
	def.linearViscosity = 0.5f;
	def.friction = 0.1f;
	def.gravityScale = 1.0f;
	def.contactPressure = 2.0f;
	def.adaptiveRatio = 10.0f;
	def.adaptiveDepth = 12.0f;
	def.adaptiveSpeed = 0.0f;
	def.adaptiveDepthLimit = false;
	def.adaptiveFullBox = 0.9f;
	def.adaptiveTolerance = 0.3f;
	def.adaptiveLimit = 3.0f;
	def.adaptiveRate = 0.05f;
	def.capacity = 100000;
	return def;
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

static void Substep( b2SubStepContext* context, int subStepIndex, int subStepCount, float h, void* userContext );

b2fFluid* b2fCreateFluid( b2WorldId worldId, const b2fFluidDef* def )
{
	b2fFluid* f = calloc( 1, sizeof( b2fFluid ) );
	f->worldId = worldId;
	f->def = *def;
	f->capacity = def->capacity;
	f->radius = 2.0f * def->particleSpacing;
	f->cellSize = 1.1f * f->radius;
	f->margin = 0.5f * def->particleSpacing;
	f->mass = def->massDensity * 0.8660254f * def->particleSpacing * def->particleSpacing;
	f->restDensity = LatticeDensity( def->particleSpacing, f->radius );
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

	b2World_SetSubStepCallback( worldId, Substep, f );
	return f;
}

void b2fDestroyFluid( b2fFluid* f )
{
	if ( b2World_IsValid( f->worldId ) )
	{
		b2World_SetSubStepCallback( f->worldId, NULL, NULL );
	}

#define B2F_FREE_FIELD( type, name ) free( f->name );
#define B2F_FREE_SORTED( type, name ) free( f->sorted.name );
	B2F_PARTICLE_FIELDS( B2F_FREE_FIELD )
	B2F_PARTICLE_FIELDS( B2F_FREE_SORTED )
	B2F_WORK_FIELDS( B2F_FREE_FIELD )
#undef B2F_FREE_FIELD
#undef B2F_FREE_SORTED
	free( f->contacts );
	free( f->pairs );
	free( f->cellStart );
	free( f->blockSize );
	free( f->blockReach );
	free( f->shapes );
	free( f->bodies );
	free( f->reactions );
	free( f->patches );
	free( f->patchImpulses );
	free( f->boxMass );
	free( f->boxPx );
	free( f->boxPy );
	free( f->boxWeight );
	free( f->boxPressure );
	free( f->boxDepth );
	free( f->boxFlow );
	free( f );
}

void b2fFluid_SetDef( b2fFluid* f, const b2fFluidDef* def )
{
	float spacing = f->def.particleSpacing;
	int capacity = f->def.capacity;
	f->def = *def;
	f->def.particleSpacing = spacing;
	f->def.capacity = capacity;
}

float b2fFluid_GetMovedMass( const b2fFluid* f )
{
	return f->movedMass;
}

int b2fFluid_GetParticleCount( const b2fFluid* f )
{
	return f->count;
}
const b2Pos* b2fFluid_GetPositions( const b2fFluid* f )
{
	return f->pos;
}
const float* b2fFluid_GetVelocitiesX( const b2fFluid* f )
{
	return f->vx;
}
const float* b2fFluid_GetVelocitiesY( const b2fFluid* f )
{
	return f->vy;
}
const float* b2fFluid_GetSizes( const b2fFluid* f )
{
	return f->size;
}
const float* b2fFluid_GetDensities( const b2fFluid* f )
{
	return f->rho;
}
float b2fFluid_GetRestDensity( const b2fFluid* f )
{
	return f->restDensity;
}
b2fProfile b2fFluid_GetProfile( const b2fFluid* f )
{
	return f->profile;
}

void b2fFluid_Prepare( b2fFluid* f )
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

static void Substep( b2SubStepContext* context, int subStepIndex, int subStepCount, float h, void* userContext )
{
	(void)subStepIndex;
	(void)subStepCount;
	b2fFluid* f = userContext;
	if ( f->count == 0 )
	{
		return;
	}

	uint64_t start = b2GetTicks();
	uint64_t ticks = start;

	b2Vec2 gravity = b2World_GetGravity( f->worldId );
	f->h = h;
	f->gravityX = f->def.gravityScale * gravity.x;
	f->gravityY = f->def.gravityScale * gravity.y;

	b2fUpdateBodies( f, context );
	b2fPredict( f, context );
	b2fSortParticles( f, context );
	f->profile.sort += b2GetMillisecondsAndReset( &ticks );

	b2fFindContacts( f, context );
	b2fRelaxDensity( f, context );
	f->profile.density += b2GetMillisecondsAndReset( &ticks );

	b2fCollide( f, context );
	b2fUpdateVelocities( f, context );
	f->profile.collide += b2GetMillisecondsAndReset( &ticks );
	f->profile.total += b2GetMilliseconds( start );
}
