// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box2d/box2d.h"
#include "box2d/fluidfun.h"

#include <float.h>
#include <limits.h>
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
	int added = b2Liquid_SpawnBox( waterId, ( b2Pos ){ -1.0f, 1.0f }, ( b2Pos ){ 1.0f, 2.0f }, b2Vec2_zero );
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
	int oilAdded = b2Liquid_SpawnBox( oilId, ( b2Pos ){ -1.0f, 3.0f }, ( b2Pos ){ 1.0f, 3.5f }, b2Vec2_zero );
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
	b2Liquid_SpawnBox( honeyId, ( b2Pos ){ -1.0f, 5.0f }, ( b2Pos ){ 1.0f, 5.5f }, b2Vec2_zero );
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
	b2Liquid_SpawnBox( waterId, ( b2Pos ){ -1.0f, 0.05f }, ( b2Pos ){ 1.0f, 0.6f }, b2Vec2_zero );

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
	b2Liquid_SpawnBox( adaptiveId, ( b2Pos ){ -2.0f, 0.05f }, ( b2Pos ){ 2.0f, 2.0f }, b2Vec2_zero );
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

// Body state, events and queries: a crate dropped into a tank of water, and a sensor under the surface
static int FluidState( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );
	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2BodyId groundId = b2CreateBody( worldId, &bodyDef );
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	float halfWidth = 1.5f;
	b2Polygon floor = b2MakeOffsetBox( halfWidth + 0.2f, 0.1f, ( b2Vec2 ){ 0.0f, -0.1f }, b2Rot_identity );
	b2Polygon left = b2MakeOffsetBox( 0.1f, 1.0f, ( b2Vec2 ){ -halfWidth - 0.1f, 1.0f }, b2Rot_identity );
	b2Polygon right = b2MakeOffsetBox( 0.1f, 1.0f, ( b2Vec2 ){ halfWidth + 0.1f, 1.0f }, b2Rot_identity );
	b2ShapeId floorId = b2CreatePolygonShape( groundId, &shapeDef, &floor );
	b2CreatePolygonShape( groundId, &shapeDef, &left );
	b2CreatePolygonShape( groundId, &shapeDef, &right );

	// A sensor box low in the water on the left, away from the crate
	b2ShapeDef sensorDef = b2DefaultShapeDef();
	sensorDef.isSensor = true;
	b2Polygon sensorBox = b2MakeOffsetBox( 0.4f, 0.15f, ( b2Vec2 ){ -0.8f, 0.4f }, b2Rot_identity );
	b2ShapeId sensorId = b2CreatePolygonShape( groundId, &sensorDef, &sensorBox );
	float sensorArea = 0.8f * 0.3f;

	b2FluidDef def = b2DefaultFluidDef();
	def.capacity = 4000;
	b2FluidId fluidId = b2CreateFluid( worldId, &def );
	b2LiquidDef water = b2DefaultLiquidDef();
	water.tuning.adaptiveRatio = 1.0f;
	b2LiquidId waterId = b2CreateLiquid( fluidId, &water );
	b2Liquid_SpawnBox( waterId, ( b2Pos ){ -halfWidth + 0.05f, 0.05f }, ( b2Pos ){ halfWidth - 0.05f, 0.75f }, b2Vec2_zero );

	// A crate of half the water's density, dropped from above on the right
	b2BodyDef crateDef = b2DefaultBodyDef();
	crateDef.type = b2_dynamicBody;
	crateDef.position = ( b2Pos ){ 0.7f, 1.8f };
	b2BodyId crateId = b2CreateBody( worldId, &crateDef );
	b2ShapeDef crateShape = b2DefaultShapeDef();
	crateShape.density = 0.5f * water.density;
	b2Polygon crateBox = b2MakeBox( 0.3f, 0.3f );
	b2CreatePolygonShape( crateId, &crateShape, &crateBox );
	float weight = b2Body_GetMass( crateId ) * b2Length( b2World_GetGravity( worldId ) );

	// Not near liquid yet, and not an error to ask, nor for a static body
	b2FluidBodyState state = b2Fluid_GetBodyState( fluidId, crateId );
	ENSURE( state.inLiquid == false && state.force.y == 0.0f && B2_IS_NULL( state.liquidId ) );

	int crateBegins = 0, crateEnds = 0, sensorBegins = 0, sensorEnds = 0;
	float forceSum = 0.0f, submergedSum = 0.0f, levelSum = 0.0f, areaSum = 0.0f;
	int settled = 0;
	for ( int frame = 0; frame < 240; ++frame )
	{
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
		ENSURE( b2Fluid_GetBodyState( fluidId, groundId ).inLiquid == false );

		// Falling, well clear of the water
		if ( b2Body_GetPosition( crateId ).y > 1.4f )
		{
			state = b2Fluid_GetBodyState( fluidId, crateId );
			ENSURE( state.inLiquid == false && state.force.y == 0.0f );
		}

		b2FluidEvents events = b2Fluid_GetEvents( fluidId );
		for ( int i = 0; i < events.bodyBeginCount; ++i )
		{
			ENSURE( B2_ID_EQUALS( events.bodyBeginEvents[i].bodyId, crateId ) );
			ENSURE( B2_ID_EQUALS( events.bodyBeginEvents[i].liquidId, waterId ) );
			crateBegins += 1;
		}
		crateEnds += events.bodyEndCount;
		for ( int i = 0; i < events.sensorBeginCount; ++i )
		{
			ENSURE( B2_ID_EQUALS( events.sensorBeginEvents[i].sensorShapeId, sensorId ) );
			sensorBegins += 1;
		}
		sensorEnds += events.sensorEndCount;

		// The last second, once the crate floats still
		if ( frame >= 180 )
		{
			state = b2Fluid_GetBodyState( fluidId, crateId );
			ENSURE( state.inLiquid && B2_ID_EQUALS( state.liquidId, waterId ) );
			ENSURE( 0.0f <= state.submerged && state.submerged < 1.0f );
			forceSum += state.force.y;
			submergedSum += state.submerged;

			// The crate's level, from the surface just beside it
			b2Pos at = b2Body_GetPosition( crateId ), surface = { 0.0f, 0.0f };
			ENSURE( b2Fluid_GetSurfaceHeight( fluidId, ( b2Pos ){ at.x + 0.3f + 2.0f * def.particleSpacing, at.y }, &surface ) );
			levelSum += (float)( surface.y - ( at.y - 0.3f ) ) / 0.6f;
			areaSum += b2Fluid_GetSensorArea( fluidId, sensorId );
			settled += 1;
		}
	}
	ENSURE( crateBegins == 1 && crateEnds == 0 );
	ENSURE( sensorBegins == 1 && sensorEnds == 0 );

	// Afloat, the liquid holds the crate up with its weight, and the share under is its level
	float force = forceSum / (float)settled, submerged = submergedSum / (float)settled, level = levelSum / (float)settled;
	ENSURE( fabsf( force - weight ) < 0.25f * weight );
	ENSURE( fabsf( submerged - level ) < 0.15f );

	// The sensor is under the surface, full of water. Particles cross its edges as the water sloshes, so this is the
	// mean over the last second. Liquid packs about as it was spawned, 0.97 particleSpacing apart, so the particles'
	// area in the sensor is about 1 / 0.97^2 of its own; it reads 1.04 of that.
	float held = areaSum / (float)settled, packed = sensorArea / ( 0.97f * 0.97f );
	ENSURE( fabsf( held - packed ) < 0.1f * packed );
	ENSURE( b2Fluid_GetSensorArea( fluidId, floorId ) == 0.0f );

	// Everything is in the tank, and the surface stands as high as the water's area, and what the crate displaces,
	// spread over the tank's width
	b2FluidAmount amount = b2Fluid_MeasureBox( fluidId, ( b2Pos ){ -halfWidth, -0.1f }, ( b2Pos ){ halfWidth, 2.0f } );
	ENSURE( amount.count == b2Fluid_GetParticleCount( fluidId ) );
	ENSURE( fabsf( amount.mass - amount.area * water.density ) < 1e-3f * amount.mass );
	float depth = ( amount.area + b2Body_GetMass( crateId ) / water.density ) / ( 2.0f * halfWidth );
	b2Pos surface = { 0.0f, 0.0f };
	ENSURE( b2Fluid_GetSurfaceHeight( fluidId, ( b2Pos ){ -0.5f, 0.1f }, &surface ) );
	ENSURE( fabsf( (float)surface.y - depth ) < def.particleSpacing );
	ENSURE( surface.x == -0.5f );
	ENSURE( b2Fluid_GetSurfaceHeight( fluidId, ( b2Pos ){ 3.0f, 0.1f }, &surface ) == false );

	// A ball in the water for one step, as a passing splash would be, does not begin; one held there for three does
	b2BodyDef ballDef = b2DefaultBodyDef();
	ballDef.type = b2_dynamicBody;
	ballDef.gravityScale = 0.0f;
	ballDef.position = ( b2Pos ){ -0.5f, 3.0f };
	b2BodyId ballId = b2CreateBody( worldId, &ballDef );
	b2Circle ball = { { 0.0f, 0.0f }, 0.1f };
	b2CreateCircleShape( ballId, &crateShape, &ball );
	int ballBegins = 0, ballEnds = 0;
	for ( int frame = 0; frame < 6; ++frame )
	{
		bool inWater = frame == 0 || frame >= 3;
		b2Body_SetTransform( ballId, ( b2Pos ){ -0.5f, inWater ? 0.4f : 3.0f }, b2Rot_identity );
		b2Body_SetLinearVelocity( ballId, b2Vec2_zero );
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
		b2FluidEvents events = b2Fluid_GetEvents( fluidId );
		ballBegins += events.bodyBeginCount;
		ballEnds += events.bodyEndCount;
		ENSURE( b2Fluid_GetBodyState( fluidId, ballId ).inLiquid == ( frame == 5 ) );
	}
	ENSURE( ballBegins == 1 && ballEnds == 0 );

	// Destroyed in the liquid, the crate ends once at the next step, and its id gives nothing after
	b2DestroyBody( crateId );
	b2World_Step( worldId, 1.0f / 60.0f, 4 );
	b2FluidEvents events = b2Fluid_GetEvents( fluidId );
	ENSURE( events.bodyEndCount == 1 && B2_ID_EQUALS( events.bodyEndEvents[0].bodyId, crateId ) );
	ENSURE( events.bodyBeginCount == 0 );
	ENSURE( b2Fluid_GetBodyState( fluidId, crateId ).inLiquid == false );
	b2World_Step( worldId, 1.0f / 60.0f, 4 );
	ENSURE( b2Fluid_GetEvents( fluidId ).bodyEndCount == 0 );

	// So does a sensor
	b2DestroyShape( sensorId, false );
	b2World_Step( worldId, 1.0f / 60.0f, 4 );
	events = b2Fluid_GetEvents( fluidId );
	ENSURE( events.sensorEndCount == 1 && B2_ID_EQUALS( events.sensorEndEvents[0].sensorShapeId, sensorId ) );
	ENSURE( b2Fluid_GetSensorArea( fluidId, sensorId ) == 0.0f );

	b2DestroyWorld( worldId );
	return 0;
}

// The particles added since first, with the velocity asked for, all passing an inside test
typedef bool InsideFcn( b2Pos p, const void* shape );

static bool CheckSpawned( b2FluidId fluidId, int first, b2Vec2 velocity, InsideFcn* inside, const void* shape )
{
	b2FluidParticles particles = b2Fluid_GetParticles( fluidId );
	bool good = true;
	for ( int i = first; i < particles.count; ++i )
	{
		good = good && particles.velocitiesX[i] == velocity.x && particles.velocitiesY[i] == velocity.y;
		good = good && inside( particles.positions[i], shape );
	}
	return good;
}

// A small allowance for rounding at the edge
#define SPAWN_EDGE 1e-4f

static bool InsideCircle( b2Pos p, const void* shape )
{
	const b2Circle* c = shape;
	return b2Length( b2Sub( b2ToVec2( p ), c->center ) ) <= c->radius + SPAWN_EDGE;
}

static bool InsideCapsule( b2Pos p, const void* shape )
{
	const b2Capsule* c = shape;
	b2Vec2 d = b2Sub( c->center2, c->center1 ), q = b2Sub( b2ToVec2( p ), c->center1 );
	float t = b2ClampFloat( b2Dot( q, d ) / b2Dot( d, d ), 0.0f, 1.0f );
	return b2Length( b2Sub( q, b2MulSV( t, d ) ) ) <= c->radius + SPAWN_EDGE;
}

typedef struct SpawnBox
{
	b2Vec2 center, half;
	b2Rot rotation;
} SpawnBox;

static bool InsideBox( b2Pos p, const void* shape )
{
	const SpawnBox* box = shape;
	b2Vec2 local = b2InvRotateVector( box->rotation, b2Sub( b2ToVec2( p ), box->center ) );
	return fabsf( local.x ) <= box->half.x + SPAWN_EDGE && fabsf( local.y ) <= box->half.y + SPAWN_EDGE;
}

// Spawning in shapes, removing by box, shape and liquid, emitters, and capacity that grows
static int FluidSpawnAndRemove( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.gravity = b2Vec2_zero;
	b2WorldId worldId = b2CreateWorld( &worldDef );

	// Small to start with, so every spawn below makes it grow
	b2FluidDef def = b2DefaultFluidDef();
	def.capacity = 64;
	b2FluidId fluidId = b2CreateFluid( worldId, &def );
	ENSURE( b2Fluid_GetCapacity( fluidId ) == 64 );

	b2LiquidDef water = b2DefaultLiquidDef();
	water.tuning.adaptiveRatio = 1.0f;
	b2LiquidId waterId = b2CreateLiquid( fluidId, &water );

	// A particle spawned fills a triangle-lattice cell 0.97 particleSpacing across, 1.09 of the solver's spacing
	float spawnSpacing = 1.09f * def.particleSpacing / 1.125f, cellArea = 0.8660254f * spawnSpacing * spawnSpacing;

	b2Vec2 velocity = { 1.0f, -2.0f };
	b2Circle circle = { { 0.0f, 0.0f }, 0.5f };
	int count = b2Liquid_SpawnCircle( waterId, b2ToPos( circle.center ), circle.radius, velocity );
	ENSURE( fabsf( (float)count - B2_PI * circle.radius * circle.radius / cellArea ) < 0.1f * (float)count );
	ENSURE( CheckSpawned( fluidId, 0, velocity, InsideCircle, &circle ) );
	ENSURE( b2Fluid_GetCapacity( fluidId ) >= count );

	int first = b2Fluid_GetParticleCount( fluidId );
	b2Capsule capsule = { { 1.5f, -0.2f }, { 2.5f, 0.3f }, 0.3f };
	count = b2Liquid_SpawnCapsule( waterId, b2ToPos( capsule.center1 ), b2ToPos( capsule.center2 ), capsule.radius, velocity );
	float capsuleArea = 2.0f * capsule.radius * b2Distance( capsule.center1, capsule.center2 ) + B2_PI * capsule.radius * capsule.radius;
	ENSURE( fabsf( (float)count - capsuleArea / cellArea ) < 0.1f * (float)count );
	ENSURE( CheckSpawned( fluidId, first, velocity, InsideCapsule, &capsule ) );

	first = b2Fluid_GetParticleCount( fluidId );
	SpawnBox box = { { 0.0f, 2.0f }, { 0.6f, 0.4f }, b2MakeRot( 0.3f ) };
	b2Polygon polygon = b2MakeBox( box.half.x, box.half.y );
	count = b2Liquid_SpawnPolygon( waterId, b2ToPos( box.center ), box.rotation, &polygon, velocity );
	ENSURE( fabsf( (float)count - 4.0f * box.half.x * box.half.y / cellArea ) < 0.1f * (float)count );
	ENSURE( CheckSpawned( fluidId, first, velocity, InsideBox, &box ) );

	// An upright box, filled as the polygon was
	first = b2Fluid_GetParticleCount( fluidId );
	SpawnBox upright = { { -3.0f, 2.0f }, { 1.5f, 1.0f }, b2Rot_identity };
	count = b2Liquid_SpawnBox( waterId, b2ToPos( b2Sub( upright.center, upright.half ) ), b2ToPos( b2Add( upright.center, upright.half ) ), velocity );
	ENSURE( fabsf( (float)count - 4.0f * upright.half.x * upright.half.y / cellArea ) < 0.1f * (float)count );
	ENSURE( CheckSpawned( fluidId, first, velocity, InsideBox, &upright ) );
	int total = b2Fluid_GetParticleCount( fluidId );
	ENSURE( b2Fluid_GetCapacity( fluidId ) >= total );

	// A step, so the particles are sorted and the removals below run on a grid that holds
	b2World_Step( worldId, 1.0f / 60.0f, 4 );

	// Removing the right half of the circle takes exactly the particles in it, and the rest keep their ids
	b2FluidParticles particles = b2Fluid_GetParticles( fluidId );
	ENSURE( particles.count == total );
	bool* outside = calloc( (size_t)total, sizeof( bool ) );
	b2Pos lower = { 0.0f, -1.0f }, upper = { 1.0f, 1.0f };
	int inBox = 0;
	for ( int i = 0; i < particles.count; ++i )
	{
		b2Pos p = particles.positions[i];
		bool in = p.x >= lower.x && p.x <= upper.x && p.y >= lower.y && p.y <= upper.y;
		inBox += in ? 1 : 0;
		ENSURE( 0 <= particles.ids[i] && particles.ids[i] < total );
		outside[particles.ids[i]] = in == false;
	}
	ENSURE( inBox > 0 );
	ENSURE( b2Fluid_RemoveInBox( fluidId, b2_nullLiquidId, lower, upper, INT_MAX ) == inBox );
	particles = b2Fluid_GetParticles( fluidId );
	ENSURE( particles.count == total - inBox && b2Fluid_GetParticleCount( fluidId ) == particles.count );
	ENSURE( IdsAreUnique( particles ) );
	for ( int i = 0; i < particles.count; ++i )
	{
		ENSURE( outside[particles.ids[i]] );
	}
	free( outside );
	ENSURE( b2Fluid_MeasureBox( fluidId, lower, upper ).count == 0 );

	// A capped removal takes that many, from the bottom up: the upright box's lowest row and a bit, so every one that
	// goes is within a grid row, under 0.2 m, of the box's lowest particle
	{
		b2Pos boxLower = { -4.6f, 0.9f }, boxUpper = { -1.4f, 3.1f };
		particles = b2Fluid_GetParticles( fluidId );
		int startCount = particles.count, maxId = 0;
		float lowest = FLT_MAX;
		for ( int i = 0; i < particles.count; ++i )
		{
			maxId = particles.ids[i] > maxId ? particles.ids[i] : maxId;
			bool in = particles.positions[i].x >= boxLower.x && particles.positions[i].x <= boxUpper.x &&
					  particles.positions[i].y >= boxLower.y && particles.positions[i].y <= boxUpper.y;
			lowest = in && (float)particles.positions[i].y < lowest ? (float)particles.positions[i].y : lowest;
		}
		float* heightOf = calloc( (size_t)maxId + 1, sizeof( float ) );
		bool* gone = calloc( (size_t)maxId + 1, sizeof( bool ) );
		for ( int i = 0; i < particles.count; ++i )
		{
			heightOf[particles.ids[i]] = (float)particles.positions[i].y;
			gone[particles.ids[i]] = true;
		}
		ENSURE( b2Fluid_RemoveInBox( fluidId, b2_nullLiquidId, boxLower, boxUpper, 10 ) == 10 );
		particles = b2Fluid_GetParticles( fluidId );
		ENSURE( particles.count == startCount - 10 );
		for ( int i = 0; i < particles.count; ++i )
		{
			gone[particles.ids[i]] = false;
		}
		int goneCount = 0;
		for ( int id = 0; id <= maxId; ++id )
		{
			if ( gone[id] )
			{
				ENSURE( heightOf[id] < lowest + 0.2f );
				goneCount += 1;
			}
		}
		ENSURE( goneCount == 10 );
		ENSURE( b2Fluid_RemoveInBox( fluidId, b2_nullLiquidId, boxLower, boxUpper, 0 ) == 0 );
		free( heightOf );
		free( gone );
	}

	// Removing one liquid's particles leaves the other's, by box and by a sensor shape
	b2LiquidDef oil = b2MakeOilDef();
	oil.tuning.adaptiveRatio = 1.0f;
	b2LiquidId oilId = b2CreateLiquid( fluidId, &oil );
	int oilCount = b2Liquid_SpawnCircle( oilId, ( b2Pos ){ -0.3f, 0.0f }, 0.3f, b2Vec2_zero );
	int before = b2Fluid_GetParticleCount( fluidId );
	ENSURE( b2Fluid_RemoveInBox( fluidId, oilId, ( b2Pos ){ -1.0f, -1.0f }, ( b2Pos ){ 0.0f, 1.0f }, INT_MAX ) == oilCount );
	ENSURE( b2Fluid_GetParticleCount( fluidId ) == before - oilCount );
	ENSURE( b2Fluid_MeasureBox( fluidId, ( b2Pos ){ -0.5f, -0.5f }, ( b2Pos ){ -0.1f, 0.5f } ).count > 0 );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2BodyId groundId = b2CreateBody( worldId, &bodyDef );
	b2ShapeDef sensorDef = b2DefaultShapeDef();
	sensorDef.isSensor = true;
	b2Circle drain = { { -3.0f, 2.0f }, 0.5f };
	b2ShapeId drainId = b2CreateCircleShape( groundId, &sensorDef, &drain );
	ENSURE( b2Fluid_RemoveInShape( fluidId, oilId, drainId, INT_MAX ) == 0 );
	ENSURE( b2Fluid_RemoveInShape( fluidId, waterId, drainId, INT_MAX ) > 0 );
	particles = b2Fluid_GetParticles( fluidId );
	for ( int i = 0; i < particles.count; ++i )
	{
		ENSURE( b2Length( b2Sub( b2ToVec2( particles.positions[i] ), drain.center ) ) > drain.radius - SPAWN_EDGE );
	}

	// Removal leaves the liquid stepping as before
	for ( int i = 0; i < 3; ++i )
	{
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
	}
	ENSURE( IdsAreUnique( b2Fluid_GetParticles( fluidId ) ) );
	b2DestroyLiquid( waterId );
	ENSURE( b2Fluid_GetParticleCount( fluidId ) == 0 );

	// An emitter adds speed times mouth length times time, as area at rest, which is how far apart particles rest
	float speed = 2.0f, mouth = 0.5f, restArea = 0.8660254f * def.particleSpacing * def.particleSpacing;
	b2EmitterDef tapDef = b2DefaultEmitterDef();
	tapDef.point1 = ( b2Pos ){ 10.0f, 0.0f };
	tapDef.point2 = ( b2Pos ){ 10.0f + mouth, 0.0f };
	tapDef.velocity = ( b2Vec2 ){ 0.3f, -speed };
	b2EmitterId tapId = b2CreateEmitter( oilId, &tapDef );
	ENSURE( b2Emitter_IsValid( tapId ) );
	ENSURE( b2Emitter_GetDef( tapId ).velocity.y == -speed );
	for ( int i = 0; i < 60; ++i )
	{
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
	}
	int emitted = b2Fluid_GetParticleCount( fluidId );
	float expected = speed * mouth * 1.0f / restArea;
	ENSURE( fabsf( (float)emitted - expected ) < 0.1f * expected );

	// Disabled, it adds nothing; destroyed, its id is gone; and destroying its liquid ends another
	tapDef.enabled = false;
	b2Emitter_SetDef( tapId, &tapDef );
	for ( int i = 0; i < 5; ++i )
	{
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
	}
	ENSURE( b2Fluid_GetParticleCount( fluidId ) == emitted );
	b2DestroyEmitter( tapId );
	ENSURE( b2Emitter_IsValid( tapId ) == false );
	tapDef.enabled = true;
	b2EmitterId secondId = b2CreateEmitter( oilId, &tapDef );
	b2World_Step( worldId, 1.0f / 60.0f, 4 );
	b2DestroyLiquid( oilId );
	ENSURE( b2Emitter_IsValid( secondId ) == false );
	b2World_Step( worldId, 1.0f / 60.0f, 4 );
	ENSURE( b2Fluid_GetParticleCount( fluidId ) == 0 );
	b2DestroyFluid( fluidId );

	// A fluid with springs grows from 64 while they are in use, and stops at its most
	def.capacity = 64;
	def.maxCapacity = 500;
	fluidId = b2CreateFluid( worldId, &def );
	b2LiquidDef jelly = b2MakeJellyDef();
	jelly.tuning.adaptiveRatio = 1.0f;
	b2LiquidId jellyId = b2CreateLiquid( fluidId, &jelly );
	count = b2Liquid_SpawnCircle( jellyId, ( b2Pos ){ 0.0f, 0.0f }, 0.5f, b2Vec2_zero );
	ENSURE( count > 64 && count < def.maxCapacity );
	for ( int i = 0; i < 3; ++i )
	{
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
	}
	int more = b2Liquid_SpawnCircle( jellyId, ( b2Pos ){ 2.0f, 0.0f }, 0.8f, b2Vec2_zero );
	ENSURE( count + more == def.maxCapacity );
	ENSURE( b2Fluid_GetParticleCount( fluidId ) == def.maxCapacity && b2Fluid_GetCapacity( fluidId ) == def.maxCapacity );
	b2EmitterId fullId = b2CreateEmitter( jellyId, &tapDef );
	for ( int i = 0; i < 3; ++i )
	{
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
	}
	ENSURE( b2Emitter_IsValid( fullId ) );
	ENSURE( b2Fluid_GetParticleCount( fluidId ) == def.maxCapacity );
	ENSURE( IdsAreUnique( b2Fluid_GetParticles( fluidId ) ) );

	b2DestroyWorld( worldId );
	ENSURE( b2Emitter_IsValid( fullId ) == false );
	return 0;
}

// Each particle's velocity, by id, in an array as long as the count; ids start at zero and none is removed here
static void VelocitiesById( b2FluidId fluidId, b2Vec2* velocities )
{
	b2FluidParticles particles = b2Fluid_GetParticles( fluidId );
	for ( int i = 0; i < particles.count; ++i )
	{
		velocities[particles.ids[i]] = ( b2Vec2 ){ particles.velocitiesX[i], particles.velocitiesY[i] };
	}
}

static b2Vec2 TotalVelocity( b2FluidId fluidId )
{
	b2FluidParticles particles = b2Fluid_GetParticles( fluidId );
	b2Vec2 sum = b2Vec2_zero;
	for ( int i = 0; i < particles.count; ++i )
	{
		sum = b2Add( sum, ( b2Vec2 ){ particles.velocitiesX[i], particles.velocitiesY[i] } );
	}
	return sum;
}

// The mean speed away from the centre, after one step, of the particles an explosion in the middle of a still blob
// pushed, at a particle spacing
static float BlastSpeed( float spacing, const b2ExplosionDef* blast )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.gravity = b2Vec2_zero;
	b2WorldId worldId = b2CreateWorld( &worldDef );
	b2FluidDef def = b2DefaultFluidDef();
	def.particleSpacing = spacing;
	def.capacity = 1000;
	b2FluidId fluidId = b2CreateFluid( worldId, &def );
	b2LiquidDef water = b2DefaultLiquidDef();
	water.tuning.adaptiveRatio = 1.0f;
	b2LiquidId waterId = b2CreateLiquid( fluidId, &water );
	int count = b2Liquid_SpawnCircle( waterId, ( b2Pos ){ 0.0f, 0.0f }, 0.6f, b2Vec2_zero );
	bool* pushed = calloc( (size_t)count, sizeof( bool ) );
	b2FluidParticles particles = b2Fluid_GetParticles( fluidId );
	for ( int i = 0; i < count; ++i )
	{
		float d = b2Distance( b2ToVec2( particles.positions[i] ), b2ToVec2( blast->position ) );
		pushed[particles.ids[i]] = d < blast->radius + blast->falloff;
	}
	b2World_Explode( worldId, blast );
	b2World_Step( worldId, 1.0f / 60.0f, 4 );

	float sum = 0.0f;
	int n = 0;
	particles = b2Fluid_GetParticles( fluidId );
	for ( int i = 0; i < particles.count; ++i )
	{
		b2Vec2 offset = b2Sub( b2ToVec2( particles.positions[i] ), b2ToVec2( blast->position ) );
		if ( pushed[particles.ids[i]] && b2Length( offset ) > 0.0f )
		{
			sum += b2Dot( ( b2Vec2 ){ particles.velocitiesX[i], particles.velocitiesY[i] }, b2Normalize( offset ) );
			n += 1;
		}
	}
	free( pushed );
	b2DestroyWorld( worldId );
	return n > 0 ? sum / (float)n : 0.0f;
}

// Changing velocity in a box or a shape, blending it toward a velocity, and explosions, in a world without gravity
static int FluidPush( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.gravity = b2Vec2_zero;
	b2WorldId worldId = b2CreateWorld( &worldDef );
	b2FluidDef def = b2DefaultFluidDef();
	def.capacity = 1000;
	b2FluidId fluidId = b2CreateFluid( worldId, &def );

	// Uniform sizes, so every particle has the same mass and the total velocity stands for momentum
	b2LiquidDef water = b2DefaultLiquidDef();
	water.tuning.adaptiveRatio = 1.0f;
	b2LiquidId waterId = b2CreateLiquid( fluidId, &water );
	int count = b2Liquid_SpawnBox( waterId, ( b2Pos ){ -1.0f, -0.5f }, ( b2Pos ){ 1.0f, 0.5f }, b2Vec2_zero );
	b2World_Step( worldId, 1.0f / 60.0f, 4 );
	ENSURE( b2Fluid_GetParticleCount( fluidId ) == count );

	b2Vec2* before = malloc( (size_t)count * sizeof( b2Vec2 ) );
	b2Vec2* after = malloc( (size_t)count * sizeof( b2Vec2 ) );
	bool* inside = malloc( (size_t)count * sizeof( bool ) );

	// The right of the blob, with one edge through it, which no particle is on
	float left = 0.0123f, right = 2.0f, bottom = -2.0f, top = 2.0f;
	b2Pos lower = { left, bottom }, upper = { right, top };
	b2FluidParticles particles = b2Fluid_GetParticles( fluidId );
	int inBox = 0;
	for ( int i = 0; i < count; ++i )
	{
		b2Pos p = particles.positions[i];
		ENSURE( fabsf( (float)p.x - left ) > 1e-4f );
		inside[particles.ids[i]] = p.x >= left;
		inBox += p.x >= left ? 1 : 0;
	}
	ENSURE( 0 < inBox && inBox < count );

	// Exactly the particles inside change, by exactly the change
	b2Vec2 change = { 0.5f, 0.25f };
	VelocitiesById( fluidId, before );
	ENSURE( b2Fluid_AccelerateInBox( fluidId, b2_nullLiquidId, lower, upper, change ) == inBox );
	VelocitiesById( fluidId, after );
	for ( int id = 0; id < count; ++id )
	{
		b2Vec2 expected = inside[id] ? b2Add( before[id], change ) : before[id];
		ENSURE( after[id].x == expected.x && after[id].y == expected.y );
	}

	// A sensor box over the same region takes the same particles: taking the change off again leaves the velocities
	// as they were, to rounding
	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2BodyId groundId = b2CreateBody( worldId, &bodyDef );
	b2ShapeDef sensorDef = b2DefaultShapeDef();
	sensorDef.isSensor = true;
	b2Polygon region = b2MakeOffsetBox( 0.5f * ( right - left ), 0.5f * ( top - bottom ),
										( b2Vec2 ){ 0.5f * ( left + right ), 0.5f * ( bottom + top ) }, b2Rot_identity );
	b2ShapeId regionId = b2CreatePolygonShape( groundId, &sensorDef, &region );
	ENSURE( b2Fluid_AccelerateInShape( fluidId, b2_nullLiquidId, regionId, b2Neg( change ) ) == inBox );
	VelocitiesById( fluidId, after );
	for ( int id = 0; id < count; ++id )
	{
		ENSURE( b2Distance( after[id], before[id] ) < 1e-6f );
	}

	// Naming a liquid picks only its particles
	b2LiquidDef oil = b2MakeOilDef();
	b2LiquidId oilId = b2CreateLiquid( fluidId, &oil );
	ENSURE( b2Fluid_AccelerateInBox( fluidId, oilId, lower, upper, change ) == 0 );
	ENSURE( b2Fluid_AccelerateInBox( fluidId, waterId, lower, upper, b2Vec2_zero ) == inBox );
	b2DestroyLiquid( oilId );

	// A share of 1 sets the velocity, and a half halves the difference
	b2Vec2 target = { 1.0f, 0.0f };
	VelocitiesById( fluidId, before );
	ENSURE( b2Fluid_BlendVelocityInBox( fluidId, b2_nullLiquidId, lower, upper, ( b2Vec2 ){ 0.0f, 2.0f }, 0.5f ) == inBox );
	VelocitiesById( fluidId, after );
	for ( int id = 0; id < count; ++id )
	{
		b2Vec2 expected = inside[id] ? b2Lerp( before[id], ( b2Vec2 ){ 0.0f, 2.0f }, 0.5f ) : before[id];
		ENSURE( b2Distance( after[id], expected ) < 1e-6f );
	}
	ENSURE( b2Fluid_BlendVelocityInShape( fluidId, b2_nullLiquidId, regionId, target, 1.0f ) == inBox );
	VelocitiesById( fluidId, after );
	for ( int id = 0; id < count; ++id )
	{
		ENSURE( inside[id] == false || ( after[id].x == target.x && after[id].y == target.y ) );
	}

	// The next step moves them the way they were pushed
	b2Pos* start = malloc( (size_t)count * sizeof( b2Pos ) );
	particles = b2Fluid_GetParticles( fluidId );
	for ( int i = 0; i < count; ++i )
	{
		start[particles.ids[i]] = particles.positions[i];
	}
	float timeStep = 1.0f / 60.0f, pushedMove = 0.0f, otherMove = 0.0f;
	b2World_Step( worldId, timeStep, 4 );
	particles = b2Fluid_GetParticles( fluidId );
	for ( int i = 0; i < count; ++i )
	{
		float dx = (float)( particles.positions[i].x - start[particles.ids[i]].x );
		pushedMove += inside[particles.ids[i]] ? dx : 0.0f;
		otherMove += inside[particles.ids[i]] ? 0.0f : dx;
	}
	ENSURE( pushedMove / (float)inBox > 0.5f * target.x * timeStep );
	ENSURE( fabsf( otherMove / (float)( count - inBox ) ) < 0.5f * pushedMove / (float)inBox );

	// Momentum given to the blob is kept as the liquid works it through, to a few percent
	b2Fluid_BlendVelocityInBox( fluidId, b2_nullLiquidId, ( b2Pos ){ -2.0f, -2.0f }, ( b2Pos ){ 2.0f, 2.0f }, b2Vec2_zero, 1.0f );
	ENSURE( b2Length( TotalVelocity( fluidId ) ) == 0.0f );
	b2Fluid_AccelerateInBox( fluidId, b2_nullLiquidId, ( b2Pos ){ -2.0f, -2.0f }, ( b2Pos ){ 0.0f, 2.0f }, ( b2Vec2 ){ 1.0f, 0.0f } );
	b2Vec2 given = TotalVelocity( fluidId );
	ENSURE( given.x > 0.25f * (float)count );
	for ( int i = 0; i < 10; ++i )
	{
		b2World_Step( worldId, timeStep, 4 );
	}
	b2Vec2 kept = TotalVelocity( fluidId );
	ENSURE( fabsf( kept.x - given.x ) < 0.03f * given.x && fabsf( kept.y ) < 0.03f * given.x );
	free( start );
	free( before );
	free( after );
	free( inside );

	// An explosion in the middle of a still blob throws particles out from the centre, by impulsePerLength over
	// density times radius + falloff / 2 within the radius, less through the falloff, and none past it. A new fluid,
	// so ids start at zero again.
	b2DestroyFluid( fluidId );
	fluidId = b2CreateFluid( worldId, &def );
	waterId = b2CreateLiquid( fluidId, &water );
	count = b2Liquid_SpawnCircle( waterId, ( b2Pos ){ 0.0f, 0.0f }, 0.6f, b2Vec2_zero );
	before = malloc( (size_t)count * sizeof( b2Vec2 ) );
	after = malloc( (size_t)count * sizeof( b2Vec2 ) );
	b2ExplosionDef blast = b2DefaultExplosionDef();
	blast.position = ( b2Pos ){ 0.01f, 0.02f };
	blast.radius = 0.2f;
	blast.falloff = 0.2f;
	blast.impulsePerLength = 0.5f;
	float reach = blast.radius + blast.falloff, length = blast.radius + 0.5f * blast.falloff;
	int expectedCount = 0;
	particles = b2Fluid_GetParticles( fluidId );
	for ( int i = 0; i < count; ++i )
	{
		expectedCount += b2Distance( b2ToVec2( particles.positions[i] ), b2ToVec2( blast.position ) ) < reach ? 1 : 0;
	}
	ENSURE( b2Fluid_Explode( fluidId, &blast ) == expectedCount );

	float nearSpeed = 0.0f, farSpeed = 0.0f;
	int nearCount = 0, farCount = 0;
	particles = b2Fluid_GetParticles( fluidId );
	for ( int i = 0; i < count; ++i )
	{
		b2Vec2 offset = b2Sub( b2ToVec2( particles.positions[i] ), b2ToVec2( blast.position ) );
		b2Vec2 velocity = { particles.velocitiesX[i], particles.velocitiesY[i] };
		float d = b2Length( offset );
		float scale = d <= blast.radius ? 1.0f : b2MaxFloat( ( reach - d ) / blast.falloff, 0.0f );
		float expected = blast.impulsePerLength * scale / ( water.density * length );
		ENSURE( fabsf( b2Length( velocity ) - expected ) < 1e-3f * expected + 1e-6f );
		ENSURE( d >= reach ? b2Length( velocity ) == 0.0f : b2Dot( velocity, offset ) > 0.999f * b2Length( velocity ) * d );
		nearSpeed += d < blast.radius ? b2Length( velocity ) : 0.0f;
		nearCount += d < blast.radius ? 1 : 0;
		farSpeed += d > blast.radius && d < reach ? b2Length( velocity ) : 0.0f;
		farCount += d > blast.radius && d < reach ? 1 : 0;
	}
	ENSURE( nearCount > 0 && farCount > 0 && nearSpeed / (float)nearCount > 1.5f * farSpeed / (float)farCount );

	// b2World_Explode throws the liquid the same way, adding the same again
	VelocitiesById( fluidId, before );
	b2World_Explode( worldId, &blast );
	VelocitiesById( fluidId, after );
	particles = b2Fluid_GetParticles( fluidId );
	for ( int i = 0; i < count; ++i )
	{
		int id = particles.ids[i];
		float d = b2Distance( b2ToVec2( particles.positions[i] ), b2ToVec2( blast.position ) );
		b2Vec2 expected = d < reach ? b2MulSV( 2.0f, before[id] ) : before[id];
		ENSURE( b2Distance( after[id], expected ) < 1e-5f );
	}

	// And a step carries the particles out from the centre
	float spread = 0.0f, spreadBefore = 0.0f;
	particles = b2Fluid_GetParticles( fluidId );
	for ( int i = 0; i < count; ++i )
	{
		spreadBefore += b2Distance( b2ToVec2( particles.positions[i] ), b2ToVec2( blast.position ) );
	}
	b2World_Step( worldId, timeStep, 4 );
	particles = b2Fluid_GetParticles( fluidId );
	for ( int i = 0; i < count; ++i )
	{
		spread += b2Distance( b2ToVec2( particles.positions[i] ), b2ToVec2( blast.position ) );
	}
	ENSURE( spread > spreadBefore );

	free( before );
	free( after );
	b2DestroyWorld( worldId );

	// The same explosion throws the same blob as hard at any particle spacing
	float fine = BlastSpeed( 0.08f, &blast ), coarse = BlastSpeed( 0.12f, &blast );
	ENSURE( fine > 0.0f && fabsf( coarse - fine ) < 0.05f * fine );
	return 0;
}

// The highest particle of a liquid, or of every liquid for -1, and how many particles of it there are
static float TopOfLiquid( b2FluidId fluidId, int liquid, int* count )
{
	b2FluidParticles particles = b2Fluid_GetParticles( fluidId );
	float top = -FLT_MAX;
	*count = 0;
	for ( int i = 0; i < particles.count; ++i )
	{
		if ( liquid < 0 || particles.liquids[i] == liquid )
		{
			top = (float)particles.positions[i].y > top ? (float)particles.positions[i].y : top;
			*count += 1;
		}
	}
	return top;
}

// Two liquids in one box over a floor, which pass through each other or not
static b2FluidId PoolOfTwo( b2WorldId worldId, bool apart, b2LiquidId* liquids )
{
	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2BodyId groundId = b2CreateBody( worldId, &bodyDef );
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2Polygon floor = b2MakeOffsetBox( 0.7f, 0.1f, ( b2Vec2 ){ 0.0f, -0.1f }, b2Rot_identity );
	b2Polygon left = b2MakeOffsetBox( 0.1f, 0.8f, ( b2Vec2 ){ -0.6f, 0.8f }, b2Rot_identity );
	b2Polygon right = b2MakeOffsetBox( 0.1f, 0.8f, ( b2Vec2 ){ 0.6f, 0.8f }, b2Rot_identity );
	b2CreatePolygonShape( groundId, &shapeDef, &floor );
	b2CreatePolygonShape( groundId, &shapeDef, &left );
	b2CreatePolygonShape( groundId, &shapeDef, &right );

	b2FluidDef def = b2DefaultFluidDef();
	def.capacity = 400;
	b2FluidId fluidId = b2CreateFluid( worldId, &def );
	b2LiquidDef first = b2DefaultLiquidDef(), second = b2DefaultLiquidDef();
	first.tuning.adaptiveRatio = second.tuning.adaptiveRatio = 1.0f;
	first.categoryBits = 0x2;
	second.categoryBits = 0x4;
	if ( apart )
	{
		first.maskBits = ~(uint64_t)0x4;
		second.maskBits = ~(uint64_t)0x2;
	}
	liquids[0] = b2CreateLiquid( fluidId, &first );
	liquids[1] = b2CreateLiquid( fluidId, &second );

	// Liquids that push on each other cannot start in one place, so those go one above the other. Those that pass
	// through each other share the box, the second shifted a little so that no two particles start at one point.
	float shift = apart ? 0.0f : 0.3f;
	b2Liquid_SpawnBox( liquids[0], ( b2Pos ){ -0.45f, 0.05f }, ( b2Pos ){ 0.45f, 0.35f }, b2Vec2_zero );
	b2Liquid_SpawnBox( liquids[1], ( b2Pos ){ -0.43f, 0.06f + shift }, ( b2Pos ){ 0.47f, 0.36f + shift }, b2Vec2_zero );
	return fluidId;
}

// The final state of a scene that uses every filter at once, run with some number of workers: water and oil with
// adaptive sizes that pass through each other, and honey that pushes on both, so the several-liquid passes and three
// sets of depth boxes run; a shelf the oil passes through above the floor; and a crate the water passes through,
// dropped into the liquid. Particles 0.05 apart make 889 at the start and 937 after a second, so each pass that reads
// neighbours (512 particles a block at the least) is split in two. Returns the particle count and copies of positions
// and velocities, which the caller frees.
typedef struct FilterRun
{
	int count;
	b2Pos* positions;
	float *velocitiesX, *velocitiesY;
	b2Pos crate;
	b2Rot crateRotation;
} FilterRun;

static FilterRun RunFilterScene( int workerCount, int frames )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.workerCount = workerCount;
	b2WorldId worldId = b2CreateWorld( &worldDef );
	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2BodyId groundId = b2CreateBody( worldId, &bodyDef );
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2Polygon floor = b2MakeOffsetBox( 1.7f, 0.1f, ( b2Vec2 ){ 0.0f, -0.1f }, b2Rot_identity );
	b2Polygon left = b2MakeOffsetBox( 0.1f, 1.5f, ( b2Vec2 ){ -1.6f, 1.5f }, b2Rot_identity );
	b2Polygon right = b2MakeOffsetBox( 0.1f, 1.5f, ( b2Vec2 ){ 1.6f, 1.5f }, b2Rot_identity );
	b2CreatePolygonShape( groundId, &shapeDef, &floor );
	b2CreatePolygonShape( groundId, &shapeDef, &left );
	b2CreatePolygonShape( groundId, &shapeDef, &right );
	b2ShapeDef shelfDef = b2DefaultShapeDef();
	shelfDef.filter.categoryBits = 0x10;
	shelfDef.filter.maskBits = ~(uint64_t)0x4;
	b2Polygon shelf = b2MakeOffsetBox( 1.5f, 0.05f, ( b2Vec2 ){ 0.0f, 0.8f }, b2Rot_identity );
	b2CreatePolygonShape( groundId, &shelfDef, &shelf );

	b2BodyDef crateDef = b2DefaultBodyDef();
	crateDef.type = b2_dynamicBody;
	crateDef.position = ( b2Pos ){ 0.7f, 1.9f };
	crateDef.linearVelocity = ( b2Vec2 ){ -1.0f, -2.0f };
	b2BodyId crateId = b2CreateBody( worldId, &crateDef );
	b2ShapeDef crateShape = b2DefaultShapeDef();
	crateShape.density = 0.5f;
	crateShape.filter.maskBits = ~(uint64_t)( 0x2 | 0x10 );
	b2Polygon crateBox = b2MakeBox( 0.15f, 0.15f );
	b2CreatePolygonShape( crateId, &crateShape, &crateBox );

	b2FluidDef def = b2DefaultFluidDef();
	def.particleSpacing = 0.05f;
	def.capacity = 4000;
	b2FluidId fluidId = b2CreateFluid( worldId, &def );
	b2LiquidDef water = b2DefaultLiquidDef(), oil = b2MakeOilDef(), honey = b2MakeHoneyDef();
	water.categoryBits = 0x2;
	water.maskBits = ~(uint64_t)0x4;
	oil.categoryBits = 0x4;
	oil.maskBits = ~(uint64_t)0x2;
	honey.categoryBits = 0x8;
	b2LiquidId waterId = b2CreateLiquid( fluidId, &water );
	b2LiquidId oilId = b2CreateLiquid( fluidId, &oil );
	b2LiquidId honeyId = b2CreateLiquid( fluidId, &honey );
	b2Liquid_SpawnBox( honeyId, ( b2Pos ){ -1.45f, 0.05f }, ( b2Pos ){ 1.45f, 0.35f }, b2Vec2_zero );
	b2Liquid_SpawnBox( waterId, ( b2Pos ){ -1.45f, 0.9f }, ( b2Pos ){ 0.0f, 1.5f }, b2Vec2_zero );
	b2Liquid_SpawnBox( oilId, ( b2Pos ){ -1.4f, 0.95f }, ( b2Pos ){ 0.3f, 1.45f }, b2Vec2_zero );

	for ( int frame = 0; frame < frames; ++frame )
	{
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
	}

	b2FluidParticles particles = b2Fluid_GetParticles( fluidId );
	FilterRun run = { 0 };
	run.count = particles.count;
	run.positions = malloc( (size_t)run.count * sizeof( b2Pos ) );
	run.velocitiesX = malloc( (size_t)run.count * sizeof( float ) );
	run.velocitiesY = malloc( (size_t)run.count * sizeof( float ) );
	memcpy( run.positions, particles.positions, (size_t)run.count * sizeof( b2Pos ) );
	memcpy( run.velocitiesX, particles.velocitiesX, (size_t)run.count * sizeof( float ) );
	memcpy( run.velocitiesY, particles.velocitiesY, (size_t)run.count * sizeof( float ) );
	run.crate = b2Body_GetPosition( crateId );
	run.crateRotation = b2Body_GetRotation( crateId );
	b2DestroyWorld( worldId );
	return run;
}

static void FreeFilterRun( FilterRun* run )
{
	free( run->positions );
	free( run->velocitiesX );
	free( run->velocitiesY );
}

// What the liquid collides with: a grate it falls through, a crate it ignores and then lifts, two liquids that pass
// through each other, a sensor that counts one of them, and an explosion that throws only one
static int FluidFilter( void )
{
	b2WorldDef worldDef = b2DefaultWorldDef();
	b2WorldId worldId = b2CreateWorld( &worldDef );
	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2BodyId groundId = b2CreateBody( worldId, &bodyDef );
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2Polygon floor = b2MakeOffsetBox( 1.0f, 0.1f, ( b2Vec2 ){ 0.0f, -0.1f }, b2Rot_identity );
	b2Polygon left = b2MakeOffsetBox( 0.1f, 1.0f, ( b2Vec2 ){ -0.9f, 1.0f }, b2Rot_identity );
	b2Polygon right = b2MakeOffsetBox( 0.1f, 1.0f, ( b2Vec2 ){ 0.9f, 1.0f }, b2Rot_identity );
	b2CreatePolygonShape( groundId, &shapeDef, &floor );
	b2CreatePolygonShape( groundId, &shapeDef, &left );
	b2CreatePolygonShape( groundId, &shapeDef, &right );

	// A grate across the tank that the liquid, category 2, passes through, and that the crate passes through too
	float grateY = 0.7f;
	b2ShapeDef grateDef = b2DefaultShapeDef();
	grateDef.filter.categoryBits = 0x8;
	grateDef.filter.maskBits = ~(uint64_t)0x2;
	b2Polygon grate = b2MakeOffsetBox( 0.8f, 0.05f, ( b2Vec2 ){ 0.0f, grateY }, b2Rot_identity );
	b2CreatePolygonShape( groundId, &grateDef, &grate );

	// A crate of half the liquid's density on the floor, which at first ignores the liquid
	b2BodyDef crateDef = b2DefaultBodyDef();
	crateDef.type = b2_dynamicBody;
	crateDef.position = ( b2Pos ){ 0.0f, 0.1f };
	b2BodyId crateId = b2CreateBody( worldId, &crateDef );
	b2ShapeDef crateShape = b2DefaultShapeDef();
	crateShape.density = 0.5f;
	crateShape.filter.categoryBits = 0x4;
	crateShape.filter.maskBits = ~(uint64_t)( 0x2 | 0x8 );
	b2Polygon crateBox = b2MakeBox( 0.1f, 0.1f );
	b2ShapeId crateShapeId = b2CreatePolygonShape( crateId, &crateShape, &crateBox );

	b2FluidDef def = b2DefaultFluidDef();
	def.capacity = 400;
	b2FluidId fluidId = b2CreateFluid( worldId, &def );
	b2LiquidDef water = b2DefaultLiquidDef();
	water.tuning.adaptiveRatio = 1.0f;
	ENSURE( water.categoryBits == B2_DEFAULT_CATEGORY_BITS && water.maskBits == B2_DEFAULT_MASK_BITS );
	water.categoryBits = 0x2;
	b2LiquidId waterId = b2CreateLiquid( fluidId, &water );
	b2Liquid_SpawnBox( waterId, ( b2Pos ){ -0.75f, grateY + 0.1f }, ( b2Pos ){ 0.75f, grateY + 0.4f }, b2Vec2_zero );

	// The liquid falls through the grate onto the floor and over the crate, which feels nothing of it
	for ( int frame = 0; frame < 90; ++frame )
	{
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
		b2FluidBodyState state = b2Fluid_GetBodyState( fluidId, crateId );
		ENSURE( state.inLiquid == false && state.force.x == 0.0f && state.force.y == 0.0f );
	}
	int count;
	float top = TopOfLiquid( fluidId, -1, &count );
	ENSURE( top < grateY - 0.1f );
	b2FluidAmount amount = b2Fluid_MeasureBox( fluidId, ( b2Pos ){ -0.8f, 0.0f }, ( b2Pos ){ 0.8f, grateY } );
	ENSURE( amount.count == count );
	ENSURE( b2Body_GetPosition( crateId ).y < 0.12f );

	// Let it collide with the liquid, still not with the grate, and it rises
	b2Filter filter = crateShape.filter;
	filter.maskBits = ~(uint64_t)0x8;
	b2Shape_SetFilter( crateShapeId, filter );
	// The liquid inside the crate is thrown out when it starts colliding, so the crate can be in the air at any one
	// frame; it has to have been in the liquid
	float lift = 0.0f;
	bool wet = false;
	for ( int frame = 0; frame < 60; ++frame )
	{
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
		b2FluidBodyState state = b2Fluid_GetBodyState( fluidId, crateId );
		lift = state.force.y > lift ? state.force.y : lift;
		wet = wet || state.inLiquid;
	}
	ENSURE( wet );
	ENSURE( lift > 0.0f );
	ENSURE( b2Body_GetPosition( crateId ).y > 0.2f );
	b2DestroyWorld( worldId );

	// Two liquids of the same density, apart and together. Apart, each settles to the depth it would have alone and
	// they overlap; together they stack to about twice that.
	float tops[2];
	for ( int apart = 0; apart < 2; ++apart )
	{
		worldId = b2CreateWorld( &worldDef );
		b2LiquidId liquids[2];
		fluidId = PoolOfTwo( worldId, apart, liquids );
		for ( int frame = 0; frame < 90; ++frame )
		{
			b2World_Step( worldId, 1.0f / 60.0f, 4 );
		}
		int counts[2];
		float first = TopOfLiquid( fluidId, 0, counts ), second = TopOfLiquid( fluidId, 1, counts + 1 );
		tops[apart] = first > second ? first : second;
		if ( apart )
		{
			ENSURE( fabsf( first - second ) < 2.0f * def.particleSpacing );

			// A sensor over the pool that only the second liquid collides with holds just that
			b2BodyId sensorBody = b2CreateBody( worldId, &bodyDef );
			b2ShapeDef sensorDef = b2DefaultShapeDef();
			sensorDef.isSensor = true;
			sensorDef.filter.maskBits = ~(uint64_t)0x2;
			b2Polygon sensorBox = b2MakeOffsetBox( 0.5f, 0.5f, ( b2Vec2 ){ 0.0f, 0.5f }, b2Rot_identity );
			b2ShapeId sensorId = b2CreatePolygonShape( sensorBody, &sensorDef, &sensorBox );
			b2World_Step( worldId, 1.0f / 60.0f, 4 );
			b2FluidParticles particles = b2Fluid_GetParticles( fluidId );
			float area = 0.0f;
			for ( int i = 0; i < particles.count; ++i )
			{
				area += particles.liquids[i] == 1 ? 0.8660254f * particles.sizes[i] * particles.sizes[i] : 0.0f;
			}
			ENSURE( fabsf( b2Fluid_GetSensorArea( fluidId, sensorId ) - area ) < 1e-3f * area );

			// An explosion whose mask leaves out the first liquid throws only the second
			b2Vec2* before = malloc( (size_t)particles.count * sizeof( b2Vec2 ) );
			b2Vec2* after = malloc( (size_t)particles.count * sizeof( b2Vec2 ) );
			VelocitiesById( fluidId, before );
			b2ExplosionDef blast = b2DefaultExplosionDef();
			blast.position = ( b2Pos ){ 0.0f, 0.1f };
			blast.radius = 0.15f;
			blast.falloff = 0.1f;
			blast.impulsePerLength = 1.0f;
			blast.maskBits = 0x4;
			int pushed = b2Fluid_Explode( fluidId, &blast );
			VelocitiesById( fluidId, after );
			particles = b2Fluid_GetParticles( fluidId );
			int moved = 0;
			for ( int i = 0; i < particles.count; ++i )
			{
				int id = particles.ids[i];
				bool changed = before[id].x != after[id].x || before[id].y != after[id].y;
				ENSURE( particles.liquids[i] == 1 || changed == false );
				moved += changed ? 1 : 0;
			}
			ENSURE( pushed > 0 && moved == pushed && pushed < counts[1] );
			blast.maskBits = 0x10;
			ENSURE( b2Fluid_Explode( fluidId, &blast ) == 0 );
			free( before );
			free( after );
		}
		b2DestroyWorld( worldId );
	}

	// The liquid alone would stand 0.3 deep
	ENSURE( tops[1] < 0.42f );
	ENSURE( tops[0] > 0.52f );

	// With every filter in use, one worker and four give the same liquid and crate, bit for bit
	FilterRun one = RunFilterScene( 1, 60 ), four = RunFilterScene( 4, 60 );
	ENSURE( one.count == four.count );
	ENSURE( memcmp( one.positions, four.positions, (size_t)one.count * sizeof( b2Pos ) ) == 0 );
	ENSURE( memcmp( one.velocitiesX, four.velocitiesX, (size_t)one.count * sizeof( float ) ) == 0 );
	ENSURE( memcmp( one.velocitiesY, four.velocitiesY, (size_t)one.count * sizeof( float ) ) == 0 );
	ENSURE( memcmp( &one.crate, &four.crate, sizeof( b2Pos ) ) == 0 );
	ENSURE( memcmp( &one.crateRotation, &four.crateRotation, sizeof( b2Rot ) ) == 0 );
	FreeFilterRun( &one );
	FreeFilterRun( &four );
	return 0;
}

int FluidTest( void )
{
	RUN_SUBTEST( FluidLifetime );
	RUN_SUBTEST( FluidParticles );
	RUN_SUBTEST( FluidState );
	RUN_SUBTEST( FluidSpawnAndRemove );
	RUN_SUBTEST( FluidPush );
	RUN_SUBTEST( FluidFilter );
	return 0;
}
