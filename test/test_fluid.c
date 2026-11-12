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
	// mean over the last second.
	float held = areaSum / (float)settled;
	ENSURE( fabsf( held - sensorArea ) < 0.1f * sensorArea );
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

	// A box puts particles on its edges, so it is close to its area only once it is many particles across
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
	ENSURE( b2Fluid_RemoveInBox( fluidId, b2_nullLiquidId, lower, upper ) == inBox );
	particles = b2Fluid_GetParticles( fluidId );
	ENSURE( particles.count == total - inBox && b2Fluid_GetParticleCount( fluidId ) == particles.count );
	ENSURE( IdsAreUnique( particles ) );
	for ( int i = 0; i < particles.count; ++i )
	{
		ENSURE( outside[particles.ids[i]] );
	}
	free( outside );
	ENSURE( b2Fluid_MeasureBox( fluidId, lower, upper ).count == 0 );

	// Removing one liquid's particles leaves the other's, by box and by a sensor shape
	b2LiquidDef oil = b2MakeOilDef();
	oil.tuning.adaptiveRatio = 1.0f;
	b2LiquidId oilId = b2CreateLiquid( fluidId, &oil );
	int oilCount = b2Liquid_SpawnCircle( oilId, ( b2Pos ){ -0.3f, 0.0f }, 0.3f, b2Vec2_zero );
	int before = b2Fluid_GetParticleCount( fluidId );
	ENSURE( b2Fluid_RemoveInBox( fluidId, oilId, ( b2Pos ){ -1.0f, -1.0f }, ( b2Pos ){ 0.0f, 1.0f } ) == oilCount );
	ENSURE( b2Fluid_GetParticleCount( fluidId ) == before - oilCount );
	ENSURE( b2Fluid_MeasureBox( fluidId, ( b2Pos ){ -0.5f, -0.5f }, ( b2Pos ){ -0.1f, 0.5f } ).count > 0 );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2BodyId groundId = b2CreateBody( worldId, &bodyDef );
	b2ShapeDef sensorDef = b2DefaultShapeDef();
	sensorDef.isSensor = true;
	b2Circle drain = { { -3.0f, 2.0f }, 0.5f };
	b2ShapeId drainId = b2CreateCircleShape( groundId, &sensorDef, &drain );
	ENSURE( b2Fluid_RemoveInShape( fluidId, oilId, drainId ) == 0 );
	ENSURE( b2Fluid_RemoveInShape( fluidId, waterId, drainId ) > 0 );
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

int FluidTest( void )
{
	RUN_SUBTEST( FluidLifetime );
	RUN_SUBTEST( FluidParticles );
	RUN_SUBTEST( FluidState );
	RUN_SUBTEST( FluidSpawnAndRemove );
	return 0;
}
