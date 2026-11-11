// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box2d/box2d.h"
#include "box2d/fluidfun.h"

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

	int added = b2Fluid_CreateParticlesInBox( fluidId, 0, ( b2Pos ){ -1.0f, 1.0f }, ( b2Pos ){ 1.0f, 2.0f } );
	ENSURE( added > 0 );
	ENSURE( b2Fluid_GetParticleCount( fluidId ) == added );

	// Stepping the world moves the liquid, with no call on the fluid
	float startY = (float)b2Fluid_GetPositions( fluidId )[0].y;
	for ( int i = 0; i < 30; ++i )
	{
		b2World_Step( worldId, 1.0f / 60.0f, 4 );
	}
	const b2Pos* positions = b2Fluid_GetPositions( fluidId );
	float lowest = (float)positions[0].y;
	for ( int i = 1; i < added; ++i )
	{
		lowest = positions[i].y < lowest ? (float)positions[i].y : lowest;
	}
	ENSURE( lowest < startY );
	ENSURE( lowest > 0.0f );

	// Destroying the fluid ends its id, and the world can then have another
	b2DestroyFluid( fluidId );
	ENSURE( b2Fluid_IsValid( fluidId ) == false );
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

int FluidTest( void )
{
	RUN_SUBTEST( FluidLifetime );
	return 0;
}
