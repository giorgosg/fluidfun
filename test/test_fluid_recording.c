// SPDX-License-Identifier: MIT
// Recording, replay and snapshots of a world with a fluid: every frame of a replay must hash as the recorded run did,
// a snapshot taken mid-run must continue as the run does, and a world without a fluid must be as Box2D has it.

#include "test_macros.h"

#include "fluid_internal.h"
#include "physics_world.h"
#include "recording.h"
#include "snapshot.h"

#include "box2d/box2d.h"
#include "box2d/fluidfun.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define FRAMES 120
#define DT ( 1.0f / 60.0f )
#define SUBSTEPS 4
// The frame the snapshot is taken before: a spawn, which coarsens reading what the last step gathered, while the honey
// is asleep
#define SNAPSHOT_FRAME 90

typedef struct Scene
{
	b2FluidId fluid;
	b2LiquidId water, oil, honey, jelly;
	b2EmitterId tap, hose;
	b2BodyId cart;
	b2ShapeId drain;
	int events, mostAsleep;
} Scene;

static b2WorldId MakeWorld( int workerCount, Scene* s )
{
	memset( s, 0, sizeof( *s ) );
	b2WorldDef worldDef = b2DefaultWorldDef();
	worldDef.workerCount = workerCount;
	b2WorldId worldId = b2CreateWorld( &worldDef );

	b2BodyDef bodyDef = b2DefaultBodyDef();
	b2BodyId ground = b2CreateBody( worldId, &bodyDef );
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2Polygon box = b2MakeOffsetBox( 2.0f, 0.2f, ( b2Vec2 ){ 0.0f, -0.2f }, b2Rot_identity );
	b2CreatePolygonShape( ground, &shapeDef, &box );
	box = b2MakeOffsetBox( 0.2f, 1.0f, ( b2Vec2 ){ -2.2f, 1.0f }, b2Rot_identity );
	b2CreatePolygonShape( ground, &shapeDef, &box );
	box = b2MakeOffsetBox( 0.2f, 1.0f, ( b2Vec2 ){ 2.2f, 1.0f }, b2Rot_identity );
	b2CreatePolygonShape( ground, &shapeDef, &box );
	// A wall that keeps the water from the jelly, so the jelly can fall asleep while the water moves
	box = b2MakeOffsetBox( 0.05f, 0.5f, ( b2Vec2 ){ 0.6f, 0.5f }, b2Rot_identity );
	b2CreatePolygonShape( ground, &shapeDef, &box );

	// A drain the script empties
	b2ShapeDef sensorDef = b2DefaultShapeDef();
	sensorDef.isSensor = true;
	sensorDef.enableSensorEvents = true;
	box = b2MakeOffsetBox( 0.3f, 0.15f, ( b2Vec2 ){ -1.6f, 0.15f }, b2Rot_identity );
	s->drain = b2CreatePolygonShape( ground, &sensorDef, &box );

	bodyDef.type = b2_dynamicBody;
	box = b2MakeBox( 0.15f, 0.15f );
	bodyDef.position = ( b2Pos ){ -0.6f, 0.7f };
	b2BodyId light = b2CreateBody( worldId, &bodyDef );
	shapeDef.density = 0.5f;
	b2CreatePolygonShape( light, &shapeDef, &box );
	bodyDef.position = ( b2Pos ){ -1.0f, 0.7f };
	b2BodyId heavy = b2CreateBody( worldId, &bodyDef );
	shapeDef.density = 2.0f;
	b2CreatePolygonShape( heavy, &shapeDef, &box );

	// A kinematic cart an emitter rides
	bodyDef.type = b2_kinematicBody;
	bodyDef.position = ( b2Pos ){ -0.5f, 1.4f };
	bodyDef.linearVelocity = ( b2Vec2 ){ 0.3f, 0.0f };
	bodyDef.angularVelocity = 0.5f;
	s->cart = b2CreateBody( worldId, &bodyDef );
	box = b2MakeBox( 0.2f, 0.05f );
	b2CreatePolygonShape( s->cart, &shapeDef, &box );
	return worldId;
}

// The fluid's calls the run makes before the step of each frame. Those that change settings wake every particle, so
// they come first, and the honey then falls asleep by SNAPSHOT_FRAME.
static void Script( b2WorldId worldId, Scene* s, int frame )
{
	if ( frame == 0 )
	{
		b2FluidDef def = b2DefaultFluidDef();
		def.capacity = 64; // grows several times
		s->fluid = b2CreateFluid( worldId, &def );
		b2LiquidDef water = b2DefaultLiquidDef();
		water.tuning.adaptiveSpeed = 1.0f; // makes speedMemory
		s->water = b2CreateLiquid( s->fluid, &water );
		b2Liquid_SpawnBox( s->water, ( b2Pos ){ -1.95f, 0.05f }, ( b2Pos ){ 0.5f, 0.4f }, b2Vec2_zero );
		// A thin layer of honey behind the wall, which falls asleep while the water still moves
		b2LiquidDef honey = b2MakeHoneyDef();
		honey.tuning.adaptiveRatio = 1.0f;
		s->honey = b2CreateLiquid( s->fluid, &honey );
		b2Liquid_SpawnBox( s->honey, ( b2Pos ){ 0.7f, 0.05f }, ( b2Pos ){ 1.95f, 0.2f }, b2Vec2_zero );
	}
	if ( frame == 2 )
	{
		b2LiquidDef oil = b2MakeOilDef();
		s->oil = b2CreateLiquid( s->fluid, &oil );
		b2Liquid_SetCohesionBetween( s->water, s->oil, 0.02f );
		b2Liquid_SpawnCircle( s->oil, ( b2Pos ){ -0.8f, 0.8f }, 0.2f, ( b2Vec2 ){ 0.0f, -1.0f } );
		b2LiquidDef water = b2Liquid_GetDef( s->water );
		water.viscosity = 1.0f;
		b2Liquid_SetDef( s->water, &water );
		b2Fluid_SetAdaptiveDepth( s->fluid, 6.0f );
		b2Fluid_SetAdaptiveFullBox( s->fluid, 0.85f );
		b2Fluid_SetMaxCapacity( s->fluid, 100000 );
	}
	if ( frame == 3 )
	{
		b2EmitterDef tap = b2DefaultEmitterDef();
		tap.point1 = ( b2Pos ){ 0.0f, 1.2f };
		tap.point2 = ( b2Pos ){ 0.2f, 1.2f };
		tap.velocity = ( b2Vec2 ){ 0.0f, -2.0f };
		s->tap = b2CreateEmitter( s->water, &tap );
		b2EmitterDef hose = b2DefaultEmitterDef();
		hose.bodyId = s->cart;
		hose.point1 = ( b2Pos ){ -0.1f, -0.06f };
		hose.point2 = ( b2Pos ){ 0.1f, -0.06f };
		hose.velocity = ( b2Vec2 ){ 0.0f, -1.5f };
		s->hose = b2CreateEmitter( s->oil, &hose );
	}
	if ( 4 <= frame && frame < 10 )
	{
		b2Fluid_Accelerate( s->fluid, s->water, b2MakeBoxRegion( ( b2Pos ){ -2.0f, 0.0f }, ( b2Pos ){ -1.0f, 1.0f } ),
							( b2Vec2 ){ 0.05f, 0.0f } );
		b2Fluid_Remove( s->fluid, b2_nullLiquidId, b2MakeShapeRegion( s->drain ), 10 );
	}
	if ( frame == 6 )
	{
		b2Polygon box = b2MakeOffsetBox( 0.4f, 0.2f, ( b2Vec2 ){ -0.6f, 0.3f }, b2MakeRot( 0.3f ) );
		b2ShapeProxy proxy = b2MakeProxy( box.vertices, box.count, 0.0f );
		b2Fluid_BlendVelocity( s->fluid, b2_nullLiquidId, b2MakeProxyRegion( b2Pos_zero, &proxy ), ( b2Vec2 ){ 0.0f, 1.0f },
							   0.5f );
		b2EmitterDef tap = b2Emitter_GetDef( s->tap );
		tap.velocity = ( b2Vec2 ){ -0.5f, -2.5f };
		b2Emitter_SetDef( s->tap, &tap );
	}
	if ( frame == 8 )
	{
		b2ExplosionDef blast = b2DefaultExplosionDef();
		blast.position = ( b2Pos ){ -1.2f, 0.3f };
		blast.radius = 0.3f;
		blast.falloff = 0.2f;
		blast.impulsePerLength = 0.5f;
		b2Fluid_Explode( s->fluid, &blast );
		blast.position = ( b2Pos ){ -0.4f, 0.3f };
		b2World_Explode( worldId, &blast );
	}
	if ( frame == 12 )
	{
		// The hose ends with its liquid, and jelly takes the oil's slot
		b2DestroyLiquid( s->oil );
		b2DestroyEmitter( s->tap );
		b2LiquidDef jelly = b2MakeJellyDef();
		s->jelly = b2CreateLiquid( s->fluid, &jelly );
		b2Liquid_SpawnCapsule( s->jelly, ( b2Pos ){ -0.3f, 0.6f }, ( b2Pos ){ 0.1f, 0.7f }, 0.1f, b2Vec2_zero );
		b2Polygon box = b2MakeBox( 0.2f, 0.1f );
		b2Liquid_SpawnPolygon( s->water, ( b2Pos ){ -1.0f, 0.9f }, b2MakeRot( 0.4f ), &box, b2Vec2_zero );
	}
	if ( frame == SNAPSHOT_FRAME )
	{
		b2Liquid_SpawnCapsule( s->water, ( b2Pos ){ -1.2f, 0.8f }, ( b2Pos ){ -0.8f, 0.9f }, 0.1f, b2Vec2_zero );
	}
	if ( frame == SNAPSHOT_FRAME + 15 )
	{
		// Wakes everything, and lets it fall asleep again
		b2Fluid_EnableSleep( s->fluid, false );
		b2Fluid_EnableSleep( s->fluid, true );
	}

	b2FluidEvents events = b2Fluid_GetEvents( s->fluid );
	s->events += events.bodyBeginCount + events.bodyEndCount + events.sensorBeginCount + events.sensorEndCount;
}

static int CountAsleep( b2FluidId fluidId )
{
	b2FluidParticles p = b2Fluid_GetParticles( fluidId );
	int asleep = 0;
	for ( int i = 0; i < p.count; ++i )
	{
		asleep += p.asleep[i] != 0 ? 1 : 0;
	}
	return asleep;
}

static void Run( b2WorldId worldId, Scene* s, int from, int to )
{
	for ( int frame = from; frame < to; ++frame )
	{
		Script( worldId, s, frame );
		b2World_Step( worldId, DT, SUBSTEPS );
		int asleep = CountAsleep( s->fluid );
		s->mostAsleep = asleep > s->mostAsleep ? asleep : s->mostAsleep;
	}
}

// The particles and bodies two worlds hold, field by field
static int SameFluid( b2WorldId a, b2WorldId b )
{
	b2World* wa = b2GetWorldFromId( a );
	b2World* wb = b2GetWorldFromId( b );
	b2fFluid* fa = wa->fluid;
	b2fFluid* fb = wb->fluid;
	ENSURE( fa != NULL && fb != NULL );
	ENSURE( fa->count == fb->count );
	size_t n = (size_t)fa->count;
	ENSURE( memcmp( fa->pos, fb->pos, n * sizeof( b2Pos ) ) == 0 );
	ENSURE( memcmp( fa->vx, fb->vx, n * sizeof( float ) ) == 0 );
	ENSURE( memcmp( fa->vy, fb->vy, n * sizeof( float ) ) == 0 );
	ENSURE( memcmp( fa->size, fb->size, n * sizeof( float ) ) == 0 );
	ENSURE( memcmp( fa->id, fb->id, n * sizeof( int ) ) == 0 );
	ENSURE( memcmp( fa->asleep, fb->asleep, n ) == 0 );
	ENSURE( b2HashWorldStateDeep( wa ) == b2HashWorldStateDeep( wb ) );
	return 0;
}

// Recorded from the first call and replayed at another worker count: every frame's hash must match, the replay must end
// where the run did, and a backward seek, which restores a keyframe and steps on from it, must land on what the run had
// at that frame
static int RecordAndSeek( void )
{
	Scene s;
	b2WorldId worldId = MakeWorld( 1, &s );
	b2Recording* rec = b2CreateRecording( 0 );
	b2World_StartRecording( worldId, rec );
	uint64_t hashes[FRAMES + 1];
	hashes[0] = b2HashWorldStateDeep( b2GetWorldFromId( worldId ) );
	for ( int frame = 0; frame < FRAMES; ++frame )
	{
		Run( worldId, &s, frame, frame + 1 );
		hashes[frame + 1] = b2HashWorldStateDeep( b2GetWorldFromId( worldId ) );
	}
	b2World_StopRecording( worldId );

	// The script did what it means to: liquid grew past its first room, springs and speed memory were made, bodies
	// met the liquid, and the honey fell asleep
	b2fFluid* f = b2GetWorldFromId( worldId )->fluid;
	ENSURE( f->capacity > 64 && f->springs != NULL && f->speedMemory != NULL );
	ENSURE( s.events > 0 );
	ENSURE( s.mostAsleep > 0 );

	// Replayed at three workers, which checks every frame's hash, it ends where the run did
	const uint8_t* data = b2Recording_GetData( rec );
	int size = b2Recording_GetSize( rec );
	b2Replay* player = b2CreateReplay( data, size, 3 );
	ENSURE( b2Replay_GetInfo( player ).frameCount == FRAMES );
	b2Replay_SetKeyframePolicy( player, 0, 8 );
	while ( b2Replay_StepFrame( player ) )
	{
	}
	ENSURE( b2Replay_HasDiverged( player ) == false );
	ENSURE( SameFluid( worldId, b2Replay_GetWorldId( player ) ) == 0 );
	b2DestroyWorld( worldId );

	// Seeking back lands on what the run had
	int targets[] = { 97, 13, 106, 0 };
	for ( int i = 0; i < (int)( sizeof( targets ) / sizeof( targets[0] ) ); ++i )
	{
		b2Replay_SeekFrame( player, targets[i] );
		ENSURE( b2Replay_GetFrame( player ) == targets[i] );
		ENSURE( b2Replay_HasDiverged( player ) == false );
		ENSURE( b2HashWorldStateDeep( b2GetWorldFromId( b2Replay_GetWorldId( player ) ) ) == hashes[targets[i]] );
	}
	b2DestroyReplay( player );
	b2DestroyRecording( rec );
	return 0;
}

// A snapshot restored in place, and loaded into a new world, continues as the run does
static int SnapshotContinues( void )
{
	Scene s;
	b2WorldId worldId = MakeWorld( 1, &s );
	Run( worldId, &s, 0, SNAPSHOT_FRAME );
	ENSURE( CountAsleep( s.fluid ) > 0 );
	int size = b2World_GetSnapshot( worldId, NULL, 0 );
	uint8_t* image = malloc( (size_t)size );
	ENSURE( b2World_GetSnapshot( worldId, image, size ) == size );

	// The run on, with the script's calls, hashed each frame
	Scene saved = s;
	uint64_t hashes[FRAMES];
	for ( int frame = SNAPSHOT_FRAME; frame < FRAMES; ++frame )
	{
		Run( worldId, &s, frame, frame + 1 );
		hashes[frame] = b2HashWorldStateDeep( b2GetWorldFromId( worldId ) );
	}

	// Back in place: held ids work again and every frame repeats
	ENSURE( b2World_RestoreFromSnapshot( worldId, image, size ) );
	s = saved;
	for ( int frame = SNAPSHOT_FRAME; frame < FRAMES; ++frame )
	{
		Run( worldId, &s, frame, frame + 1 );
		ENSURE( b2HashWorldStateDeep( b2GetWorldFromId( worldId ) ) == hashes[frame] );
	}

	// Into a new world at another worker count, stepped without calls next to the original restored again
	ENSURE( b2World_RestoreFromSnapshot( worldId, image, size ) );
	b2WorldId copy = b2CreateWorldFromSnapshot( image, size, 2 );
	ENSURE( b2World_IsValid( copy ) );
	ENSURE( SameFluid( worldId, copy ) == 0 );
	for ( int frame = 0; frame < 10; ++frame )
	{
		b2World_Step( worldId, DT, SUBSTEPS );
		b2World_Step( copy, DT, SUBSTEPS );
		ENSURE( b2HashWorldState( b2GetWorldFromId( worldId ) ) == b2HashWorldState( b2GetWorldFromId( copy ) ) );
	}
	ENSURE( SameFluid( worldId, copy ) == 0 );

	// Turning the fluid's sleep off wakes the liquid, and so do the world's sleep and a change of gravity
	b2fFluid* f = b2GetWorldFromId( copy )->fluid;
	b2FluidId copyFluid = b2World_GetFluid( copy );
	memset( f->asleep, 1, (size_t)f->count );
	b2Fluid_EnableSleep( copyFluid, false );
	ENSURE( CountAsleep( copyFluid ) == 0 && b2Fluid_IsSleepEnabled( copyFluid ) == false );
	b2Fluid_EnableSleep( copyFluid, true );
	memset( f->asleep, 1, (size_t)f->count );
	b2World_EnableSleeping( copy, false );
	b2World_Step( copy, DT, SUBSTEPS );
	ENSURE( CountAsleep( copyFluid ) == 0 );
	b2World_EnableSleeping( copy, true );
	memset( f->asleep, 1, (size_t)f->count );
	b2World_SetGravity( copy, ( b2Vec2 ){ 0.0f, -9.0f } );
	b2World_Step( copy, DT, SUBSTEPS );
	ENSURE( CountAsleep( copyFluid ) == 0 );

	b2DestroyWorld( copy );
	free( image );
	b2DestroyWorld( worldId );
	return 0;
}

// A world without a fluid: its snapshot is Box2D's image, its recording has no fluid records, and a file of Box2D's
// own, with no fork mark, still loads
static int NoFluid( void )
{
	Scene s;
	b2WorldId worldId = MakeWorld( 1, &s );
	int size = b2World_GetSnapshot( worldId, NULL, 0 );
	uint8_t* image = malloc( (size_t)size );
	b2World_GetSnapshot( worldId, image, size );
	uint32_t flags;
	memcpy( &flags, image + 12, 4 );
	ENSURE( ( flags & 0xFFFF0000u ) == 0 );

	b2Recording* rec = b2CreateRecording( 0 );
	b2World_StartRecording( worldId, rec );
	for ( int frame = 0; frame < 10; ++frame )
	{
		b2World_Step( worldId, DT, SUBSTEPS );
	}
	b2World_StopRecording( worldId );

	// An in-place restore of an image without a fluid removes one made since
	b2FluidDef def = b2DefaultFluidDef();
	b2FluidId fluidId = b2CreateFluid( worldId, &def );
	ENSURE( b2Fluid_IsValid( fluidId ) );
	ENSURE( b2World_RestoreFromSnapshot( worldId, image, size ) );
	ENSURE( b2Fluid_IsValid( fluidId ) == false );

	int recSize = b2Recording_GetSize( rec );
	uint8_t* bytes = malloc( (size_t)recSize );
	memcpy( bytes, b2Recording_GetData( rec ), (size_t)recSize );
	ENSURE( b2ValidateRecording( bytes, recSize, 1 ) );
	b2RecHeader header;
	memcpy( &header, bytes, sizeof( header ) );
	ENSURE( header.forkFormat == B2_REC_FORK_FLUIDFUN );
	header.forkFormat = 0;
	memcpy( bytes, &header, sizeof( header ) );
	ENSURE( b2ValidateRecording( bytes, recSize, 1 ) );
	header.forkFormat = 0x12345678u;
	memcpy( bytes, &header, sizeof( header ) );
	ENSURE( b2ValidateRecording( bytes, recSize, 1 ) == false );

	free( bytes );
	free( image );
	b2DestroyRecording( rec );
	b2DestroyWorld( worldId );
	return 0;
}

int FluidRecordingTest( void )
{
	RUN_SUBTEST( RecordAndSeek );
	RUN_SUBTEST( SnapshotContinues );
	RUN_SUBTEST( NoFluid );
	return 0;
}
