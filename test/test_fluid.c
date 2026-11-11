// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box2d/box2d.h"
#include "box2d/fluidfun.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

// The world owns its fluid: one at a time, stepped by b2World_Step, gone with the world
static int FluidLifetime( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2BodyId groundId = b2CreateBody( worldId, &bodyDef );
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2Polygon floor = b2MakeOffsetBox( 10.0f, 1.0f, ( b2Vec2 ){ 0.0f, -1.0f }, b2Rot_identity );
	b2CreatePolygonShape( groundId, &shapeDef, &floor );

	ENSURE( b2Fluid_IsValid( b2_nullFluidId ) == false );

	b2FluidDef def = b2DefaultFluidDef();
	def.capacity = 1000;
	b2FluidId fluidId = b2CreateFluid( worldId, &def );
	ENSURE( b2Fluid_IsValid( fluidId ) );

	// A fluid starts with no liquids, and stepping it then does nothing
	ENSURE( b2Fluid_GetLiquidCount( fluidId ) == 0 );
	b2World_Step( worldId, 1.0f / 60.0f, 4 );

	b2LiquidDef water = b2DefaultLiquidDef();
	b2LiquidId waterId = b2CreateLiquid( fluidId, &water );
	ENSURE( b2Liquid_IsValid( waterId ) );
	ENSURE( b2Liquid_GetIndex( waterId ) == 0 );
	int added = b2Liquid_CreateParticlesInBox( waterId, ( b2Pos ){ -1.0f, 1.0f }, ( b2Pos ){ 1.0f, 2.0f } );
	ENSURE( added > 0 );
	ENSURE( b2Fluid_GetParticleCount( fluidId ) == added );

	// Stepping the world moves the liquid, with no call on the fluid
	float startY = (float)b2Fluid_GetParticles( fluidId ).positions[0].y;
	for ( int i = 0; i < 30; ++i )
	{
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
	}
	const b2Pos* positions = b2Fluid_GetParticles( fluidId ).positions;
	float lowest = (float)positions[0].y;
	for ( int i = 1; i < added; ++i )
	{
		lowest = positions[i].y < lowest ? (float)positions[i].y : lowest;
	}
	ENSURE( lowest < startY );
	ENSURE( lowest > 0.0f );

	// A second liquid, then the first destroyed: its particles go, the other's stay, and its slot is used again
	b2LiquidDef oil = b2MakeOilDef();
	b2LiquidId oilId = b2CreateLiquid( fluidId, &oil );
	ENSURE( b2Liquid_GetIndex( oilId ) == 1 );
	int oilAdded = b2Liquid_CreateParticlesInBox( oilId, ( b2Pos ){ -1.0f, 3.0f }, ( b2Pos ){ 1.0f, 3.5f } );
	ENSURE( oilAdded > 0 );
	ENSURE( b2Fluid_GetLiquidCount( fluidId ) == 2 );
	b2World_Step( worldId, 1.0f / 60.0f, 4 );

	// Sizes adapt as the liquid steps, so the oil is counted now, not taken from what was added
	int oilCount = 0;
	const unsigned char* liquids = b2Fluid_GetParticles( fluidId ).liquids;
	for ( int i = 0; i < b2Fluid_GetParticleCount( fluidId ); ++i )
	{
		oilCount += liquids[i] == 1 ? 1 : 0;
	}
	ENSURE( oilCount > 0 && oilCount < b2Fluid_GetParticleCount( fluidId ) );

	b2DestroyLiquid( waterId );
	ENSURE( b2Liquid_IsValid( waterId ) == false );
	ENSURE( b2Liquid_IsValid( oilId ) );
	ENSURE( b2Fluid_GetLiquidCount( fluidId ) == 1 );
	ENSURE( b2Fluid_GetParticleCount( fluidId ) == oilCount );
	liquids = b2Fluid_GetParticles( fluidId ).liquids;
	for ( int i = 0; i < oilCount; ++i )
	{
		ENSURE( liquids[i] == 1 );
	}
	for ( int i = 0; i < 10; ++i )
	{
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b2LiquidDef honey = b2MakeHoneyDef();
	b2LiquidId honeyId = b2CreateLiquid( fluidId, &honey );
	ENSURE( b2Liquid_GetIndex( honeyId ) == 0 );
	ENSURE( b2Liquid_IsValid( waterId ) == false );
	ENSURE( b2Liquid_GetDef( honeyId ).density == honey.density );
	b2Liquid_SetCohesionBetween( honeyId, oilId, 0.05f );
	b2Liquid_CreateParticlesInBox( honeyId, ( b2Pos ){ -1.0f, 5.0f }, ( b2Pos ){ 1.0f, 5.5f } );
	for ( int i = 0; i < 10; ++i )
	{
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	// With the oil gone only the honey is left, in the first slot
	b2DestroyLiquid( oilId );
	ENSURE( b2Fluid_GetLiquidCount( fluidId ) == 1 );
	b2World_Step( worldId, 1.0f / 60.0f, 4 );

	// Destroying the fluid ends its id and its liquids', and the world can then have another
	b2DestroyFluid( fluidId );
	ENSURE( b2Fluid_IsValid( fluidId ) == false );
	ENSURE( b2Liquid_IsValid( honeyId ) == false );
	b2World_Step( worldId, 1.0f / 60.0f, 4 );

	b2FluidId secondId = b2CreateFluid( worldId, &def );
	ENSURE( b2Fluid_IsValid( secondId ) );
	ENSURE( b2Fluid_IsValid( fluidId ) == false );
	ENSURE( b2Fluid_GetParticleCount( secondId ) == 0 );

	// Destroying the world frees the fluid it still has
	b2DestroyWorld( worldId );
	ENSURE( b2Fluid_IsValid( secondId ) == false );

	// A new world in the same slot does not bring the old id back
	b2WorldId nextId = b2CreateWorld( &worldDef );
	ENSURE( b2Fluid_IsValid( secondId ) == false );
	b2FluidId thirdId = b2CreateFluid( nextId, &def );
	ENSURE( b2Fluid_IsValid( thirdId ) );
	ENSURE( b2Fluid_IsValid( secondId ) == false );
	b2DestroyWorld( nextId );

	return 0;
}

static int CompareInts( const void* a, const void* b )
{
	int x = *(const int*)a, y = *(const int*)b;
	return ( x > y ) - ( x < y );
}

// Whether no two particles share an id, which sorting a copy of the ids shows
static bool IdsAreUnique( b2FluidParticles particles )
{
	int* sorted = malloc( (size_t)particles.count * sizeof( int ) );
	memcpy( sorted, particles.ids, (size_t)particles.count * sizeof( int ) );
	qsort( sorted, (size_t)particles.count, sizeof( int ), CompareInts );
	bool unique = particles.count == 0 || sorted[0] >= 0;
	for ( int i = 1; i < particles.count; ++i )
	{
		unique = unique && sorted[i] != sorted[i - 1];
	}
	free( sorted );
	return unique;
}

// b2Fluid_GetParticles: one particle keeps its id as the order changes, and sizes come in particleSpacing
static int FluidParticles( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );
	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2BodyId groundId = b2CreateBody( worldId, &bodyDef );
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2Polygon floor = b2MakeOffsetBox( 10.0f, 1.0f, ( b2Vec2 ){ 0.0f, -1.0f }, b2Rot_identity );
	b2CreatePolygonShape( groundId, &shapeDef, &floor );

	b2FluidDef def = b2DefaultFluidDef();
	def.capacity = 4000;
	b2FluidId fluidId = b2CreateFluid( worldId, &def );

	// Uniform sizes, so no particle splits or merges and the same particles are there throughout
	b2LiquidDef water = b2DefaultLiquidDef();
	water.tuning.adaptiveRatio = 1.0f;
	b2LiquidId waterId = b2CreateLiquid( fluidId, &water );
	b2Liquid_CreateParticlesInBox( waterId, ( b2Pos ){ -1.0f, 0.05f }, ( b2Pos ){ 1.0f, 0.6f } );

	b2FluidParticles particles = b2Fluid_GetParticles( fluidId );
	int count = particles.count;
	ENSURE( count > 0 && count == b2Fluid_GetParticleCount( fluidId ) );
	ENSURE( IdsAreUnique( particles ) );
	ENSURE( particles.restDensity > 0.0f );

	// Where each particle is and how fast it moves, by id. Ids start at zero, so they index an array as long as the
	// count.
	b2Pos* last = malloc( (size_t)count * sizeof( b2Pos ) );
	b2Vec2* lastVelocity = malloc( (size_t)count * sizeof( b2Vec2 ) );
	int* firstOrder = malloc( (size_t)count * sizeof( int ) );
	for ( int i = 0; i < count; ++i )
	{
		int id = particles.ids[i];
		ENSURE( 0 <= id && id < count );
		last[id] = particles.positions[i];
		lastVelocity[id] = ( b2Vec2 ){ particles.velocitiesX[i], particles.velocitiesY[i] };
		firstOrder[i] = id;
	}

	// Each frame every id is still there, and its particle went about where its velocities at the start and end of
	// the frame take it. Neighbours are a spacing apart, so an id passed to one would be off by that much.
	float timeStep = 1.0f / 60.0f, largestMiss = 0.0f;
	bool reordered = false;
	for ( int frame = 0; frame < 60; ++frame )
	{
		b2World_Step( worldId, timeStep, 4 );
		particles = b2Fluid_GetParticles( fluidId );
		ENSURE( particles.count == count && particles.count == b2Fluid_GetParticleCount( fluidId ) );
		ENSURE( IdsAreUnique( particles ) );
		for ( int i = 0; i < count; ++i )
		{
			int id = particles.ids[i];
			ENSURE( 0 <= id && id < count );
			b2Vec2 velocity = { particles.velocitiesX[i], particles.velocitiesY[i] };
			float dx = (float)( particles.positions[i].x - last[id].x ) - 0.5f * timeStep * ( lastVelocity[id].x + velocity.x );
			float dy = (float)( particles.positions[i].y - last[id].y ) - 0.5f * timeStep * ( lastVelocity[id].y + velocity.y );
			float miss = sqrtf( dx * dx + dy * dy );
			largestMiss = miss > largestMiss ? miss : largestMiss;
			last[id] = particles.positions[i];
			lastVelocity[id] = velocity;
			reordered = reordered || id != firstOrder[i];

			// Every particle is at the finest size, which is the fluid's spacing
			ENSURE( fabsf( particles.sizes[i] - def.particleSpacing ) <= 1e-6f * def.particleSpacing );
			ENSURE( particles.liquids[i] == b2Liquid_GetIndex( waterId ) );
		}
	}
	ENSURE( largestMiss < 0.25f * def.particleSpacing );
	// The sort changed the order, so the ids were followed through it rather than left in place
	ENSURE( reordered );
	free( last );
	free( lastVelocity );
	free( firstOrder );
	b2DestroyLiquid( waterId );
	ENSURE( b2Fluid_GetParticles( fluidId ).count == 0 );

	// With adaptive sizes particles split and merge, and new particles still get ids no other has
	b2LiquidDef adaptive = b2DefaultLiquidDef();
	b2LiquidId adaptiveId = b2CreateLiquid( fluidId, &adaptive );
	b2Liquid_CreateParticlesInBox( adaptiveId, ( b2Pos ){ -2.0f, 0.05f }, ( b2Pos ){ 2.0f, 2.0f } );
	int sizeChanges = 0;
	for ( int frame = 0; frame < 90; ++frame )
	{
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
		sizeChanges += b2Fluid_GetProfile( fluidId ).sizeChanges;
	}
	particles = b2Fluid_GetParticles( fluidId );
	ENSURE( sizeChanges > 0 );
	ENSURE( particles.count == b2Fluid_GetParticleCount( fluidId ) );
	ENSURE( IdsAreUnique( particles ) );

	b2DestroyWorld( worldId );
	return 0;
}

int FluidTest( void )
{
	RUN_SUBTEST( FluidLifetime );
	RUN_SUBTEST( FluidParticles );
	return 0;
}
