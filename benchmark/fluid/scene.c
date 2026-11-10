// SPDX-License-Identifier: MIT
#include "scene.h"

#include "box2d/math_functions.h"

#include <math.h>

static b2BodyId Track( LabScene* scene, b2BodyId body )
{
	if ( scene->bodyCount < LAB_MAX_BODIES )
	{
		scene->bodies[scene->bodyCount++] = body;
	}
	return body;
}

static b2BodyId AddBody( LabScene* scene, b2WorldId worldId, b2BodyType type, float x, float y )
{
	b2BodyDef def = b2DefaultBodyDef();
	def.type = type;
	def.position = (b2Pos){ x, y };
	return Track( scene, b2CreateBody( worldId, &def ) );
}

static void AddBox( b2BodyId body, float x, float y, float halfWidth, float halfHeight, float angle, float density )
{
	b2Polygon box = b2MakeOffsetBox( halfWidth, halfHeight, (b2Vec2){ x, y }, b2MakeRot( angle ) );
	b2ShapeDef def = b2DefaultShapeDef();
	def.density = density;
	b2CreatePolygonShape( body, &def, &box );
}

// A thick plank between two points, so that fast particles cannot cross it in one substep
static void AddPlank( b2BodyId body, b2Vec2 a, b2Vec2 b, float halfThickness )
{
	b2Vec2 c = b2MulSV( 0.5f, b2Add( a, b ) );
	b2Vec2 d = b2Sub( b, a );
	AddBox( body, c.x, c.y, 0.5f * b2Length( d ), halfThickness, b2Atan2( d.y, d.x ), 0.0f );
}

static b2BodyId AddCrate( LabScene* scene, b2WorldId worldId, float x, float y, float halfSize, float density )
{
	b2BodyId body = AddBody( scene, worldId, b2_dynamicBody, x, y );
	AddBox( body, 0.0f, 0.0f, halfSize, halfSize, 0.0f, density );
	return body;
}

// A wheel of flat blades on a hub, joined to another body at the given point
static b2BodyId AddWheel( LabScene* scene, b2WorldId worldId, b2BodyId anchorBody, b2Vec2 worldPoint, float radius, int blades,
						  float density, bool motor, float speed, float torque )
{
	b2BodyId wheel = AddBody( scene, worldId, b2_dynamicBody, worldPoint.x, worldPoint.y );
	b2Circle hub = { { 0.0f, 0.0f }, 0.2f * radius };
	b2ShapeDef def = b2DefaultShapeDef();
	def.density = density;
	b2CreateCircleShape( wheel, &def, &hub );
	for ( int i = 0; i < blades; ++i )
	{
		float angle = 2.0f * B2_PI * (float)i / (float)blades;
		float middle = 0.6f * radius;
		AddBox( wheel, middle * cosf( angle ), middle * sinf( angle ), 0.4f * radius, 0.05f * radius, angle, density );
	}

	b2RevoluteJointDef joint = b2DefaultRevoluteJointDef();
	joint.base.bodyIdA = anchorBody;
	joint.base.bodyIdB = wheel;
	joint.base.localFrameA.p = b2Body_GetLocalPoint( anchorBody, (b2Pos){ worldPoint.x, worldPoint.y } );
	joint.enableMotor = motor;
	joint.motorSpeed = speed;
	joint.maxMotorTorque = torque;
	scene->joint = b2CreateRevoluteJoint( worldId, &joint );
	return wheel;
}

#define LAB_WHEEL_GAP 0.8f
#define LAB_WHEEL_SINK_END 22.0f

static b2fFluid* AddLiquid( b2WorldId worldId, const b2fFluidDef* def, b2Vec2 lower, b2Vec2 upper )
{
	b2fFluid* fluid = b2fCreateFluid( worldId, def );
	b2fFluid_CreateParticlesInBox( fluid, 0, b2ToPos( lower ), b2ToPos( upper ) );
	return fluid;
}

LabScene LabDefaultScene( LabSceneType type )
{
	LabScene s = { 0 };
	s.type = type;
	s.viewCenter = (b2Vec2){ 0.0f, 7.0f };
	s.viewHalfWidth = 13.0f;
	switch ( type )
	{
		case lab_tank:
			s.name = "Tank";
			s.tankWidth = 24.0f;
			s.tankHeight = 12.0f;
			s.liquidWidth = 10.0f;
			s.liquidHeight = 8.0f;
			s.crateCount = 5;
			break;

		case lab_hole:
			s.name = "Hole And Slopes";
			s.viewCenter = (b2Vec2){ 0.0f, 9.0f };
			s.viewHalfWidth = 12.0f;
			s.controlLabel = "Hole width";
			s.control = 0.8f;
			s.controlMin = 0.3f;
			s.controlMax = 3.0f;
			break;

		case lab_boat:
			s.name = "Paddle Boat";
			s.viewCenter = (b2Vec2){ 0.0f, 3.0f };
			s.viewHalfWidth = 32.0f;
			s.controlLabel = "Paddle speed";
			s.control = -5.0f;
			s.controlMin = -12.0f;
			s.controlMax = 12.0f;
			break;

		case lab_drum:
			s.name = "Rotating Tank";
			s.viewCenter = (b2Vec2){ 0.0f, 8.0f };
			s.viewHalfWidth = 11.0f;
			s.controlLabel = "Rotation";
			s.control = 0.5f;
			s.controlMin = -3.0f;
			s.controlMax = 3.0f;
			break;

		case lab_wheel:
			s.name = "Water Wheel";
			s.viewCenter = (b2Vec2){ 6.0f, 4.7f };
			s.viewHalfWidth = 13.0f;
			s.controlLabel = "Flow per frame";
			s.control = 15.0f;
			s.controlMin = 0.0f;
			s.controlMax = 120.0f;
			break;

		case lab_waves:
			s.name = "Wave Maker";
			s.viewCenter = (b2Vec2){ 0.0f, 3.0f };
			s.viewHalfWidth = 24.0f;
			s.controlLabel = "Piston travel";
			s.control = 3.0f;
			s.controlMin = 0.0f;
			s.controlMax = 3.0f;
			break;

		case lab_splash:
			s.name = "Splash";
			s.viewCenter = (b2Vec2){ 0.0f, 6.0f };
			s.viewHalfWidth = 12.0f;
			s.controlLabel = "Ball density";
			s.control = 0.7f;
			s.controlMin = 0.005f;
			s.controlMax = 4.0f;
			break;

		case lab_drain:
			s.name = "Deep Drain";
			s.viewCenter = (b2Vec2){ 0.0f, 9.0f };
			s.viewHalfWidth = 14.0f;
			s.controlLabel = "Hole width";
			s.control = 0.8f;
			s.controlMin = 0.3f;
			s.controlMax = 3.0f;
			break;

		case lab_pool:
			s.name = "Deep Pool";
			s.viewCenter = (b2Vec2){ 0.0f, 10.0f };
			s.viewHalfWidth = 14.0f;
			s.controlLabel = "Pool depth";
			s.control = 10.0f;
			s.controlMin = 5.0f;
			s.controlMax = 100.0f;
			break;

		case lab_drop:
			s.name = "Drop";
			s.viewCenter = (b2Vec2){ 0.0f, 4.0f };
			s.viewHalfWidth = 6.0f;
			s.controlLabel = "Gravity";
			s.control = 1.0f;
			s.controlMin = 0.0f;
			s.controlMax = 1.0f;
			break;

		case lab_chain:
			s.name = "Chain Bowl";
			s.viewCenter = (b2Vec2){ 0.0f, 4.0f };
			s.viewHalfWidth = 7.5f;
			break;

		case lab_layers:
			s.name = "Three Liquids";
			s.viewCenter = (b2Vec2){ 0.0f, 4.5f };
			s.viewHalfWidth = 9.0f;
			s.controlLabel = "Cohesion between";
			s.control = 0.0f;
			s.controlMin = 0.0f;
			s.controlMax = 0.3f;
			break;

		case lab_tap:
			s.name = "Dripping Tap";
			s.viewCenter = (b2Vec2){ 0.0f, 4.8f };
			s.viewHalfWidth = 5.0f;
			s.controlLabel = "Nozzle width";
			s.control = 0.4f;
			s.controlMin = 0.2f;
			s.controlMax = 0.8f;
			break;

		default:
			break;
	}
	return s;
}

void LabSceneSettings( LabSceneType type, b2fFluidDef* def )
{
	switch ( type )
	{
		case lab_tap:
			// Thick enough that the stream breaks into drops. From about 0.15 the liquid holds in the nozzle like a gel.
			def->liquid.cohesion = 0.1f;
			break;

		case lab_drop:
			// Thicker than water, so the drop holds together and shows its tension
			def->liquid.cohesion = 0.2f;
			break;

		default:
			break;
	}
}

static b2fFluid* CreateTank( b2WorldId worldId, LabScene* scene, const b2fFluidDef* def )
{
	static const float densities[] = { 0.5f, 0.25f, 0.75f, 0.1f, 2.0f };
	for ( int i = 0; i < scene->crateCount; ++i )
	{
		AddCrate( scene, worldId, 2.0f + 2.0f * (float)( i % 5 ), 2.0f + 2.5f * (float)( i / 5 ), 0.6f, densities[i % 5] );
	}

	b2BodyId tank = AddBody( scene, worldId, b2_staticBody, 0.0f, 0.0f );
	float w = 0.5f * scene->tankWidth, h = scene->tankHeight, t = 1.0f;
	AddBox( tank, 0.0f, -t, w + 2.0f * t, t, 0.0f, 0.0f );
	AddBox( tank, -w - t, 0.5f * h, t, 0.5f * h + 2.0f * t, 0.0f, 0.0f );
	AddBox( tank, w + t, 0.5f * h, t, 0.5f * h + 2.0f * t, 0.0f, 0.0f );

	float margin = def->particleSpacing;
	return AddLiquid( worldId, def, (b2Vec2){ -w + margin, margin }, (b2Vec2){ -w + scene->liquidWidth, scene->liquidHeight } );
}

// A reservoir drains through a hole onto two slopes and into a basin. A slot in the basin floor
// feeds a sump, and the sump is pumped back to the top of the reservoir.
static b2fFluid* CreateHole( b2WorldId worldId, LabScene* scene, const b2fFluidDef* def )
{
	b2BodyId ground = AddBody( scene, worldId, b2_staticBody, 0.0f, 0.0f );

	float half = 0.5f * scene->control;
	AddBox( ground, -3.0f - 0.5f * half, 14.0f, 3.0f - 0.5f * half, 0.25f, 0.0f, 0.0f );
	AddBox( ground, 3.0f + 0.5f * half, 14.0f, 3.0f - 0.5f * half, 0.25f, 0.0f, 0.0f );
	AddBox( ground, -6.3f, 17.0f, 0.3f, 3.25f, 0.0f, 0.0f );
	AddBox( ground, 6.3f, 17.0f, 0.3f, 3.25f, 0.0f, 0.0f );

	// Each slope runs into a wall at its high end, so nothing can get past behind it
	AddPlank( ground, (b2Vec2){ -9.8f, 12.0f }, (b2Vec2){ 6.0f, 9.8f }, 0.2f );
	AddPlank( ground, (b2Vec2){ 9.8f, 9.0f }, (b2Vec2){ -6.0f, 6.0f }, 0.2f );
	AddPlank( ground, (b2Vec2){ -9.8f, 5.0f }, (b2Vec2){ 5.0f, 3.0f }, 0.2f );

	// Basin, with a slot in the floor at the right
	AddBox( ground, -2.0f, -0.5f, 8.0f, 0.5f, 0.0f, 0.0f );
	AddBox( ground, 9.5f, -0.5f, 2.5f, 0.5f, 0.0f, 0.0f );
	AddBox( ground, -10.5f, 7.0f, 0.5f, 8.0f, 0.0f, 0.0f );
	AddBox( ground, 10.5f, 7.0f, 0.5f, 8.0f, 0.0f, 0.0f );

	// Sump under the slot
	AddBox( ground, 6.5f, -3.5f, 2.5f, 0.3f, 0.0f, 0.0f );
	AddBox( ground, 3.8f, -2.0f, 0.3f, 1.5f, 0.0f, 0.0f );
	AddBox( ground, 9.2f, -2.0f, 0.3f, 1.5f, 0.0f, 0.0f );

	AddCrate( scene, worldId, -4.0f, 2.0f, 0.6f, 0.4f );
	AddCrate( scene, worldId, 0.0f, 2.0f, 0.6f, 0.7f );

	return AddLiquid( worldId, def, (b2Vec2){ -5.9f, 14.4f }, (b2Vec2){ 5.9f, 19.5f } );
}

static b2fFluid* CreateBoat( b2WorldId worldId, LabScene* scene, const b2fFluidDef* def )
{
	b2BodyId ground = AddBody( scene, worldId, b2_staticBody, 0.0f, 0.0f );
	AddBox( ground, 0.0f, -1.0f, 32.0f, 1.0f, 0.0f, 0.0f );
	AddBox( ground, -31.0f, 4.0f, 1.0f, 5.0f, 0.0f, 0.0f );
	AddBox( ground, 31.0f, 4.0f, 1.0f, 5.0f, 0.0f, 0.0f );

	float deck = 3.9f;
	b2BodyId hull = AddBody( scene, worldId, b2_dynamicBody, -18.0f, deck );
	b2Vec2 points[] = { { -3.0f, -0.7f }, { 3.0f, -0.7f }, { 4.2f, 0.4f }, { -3.2f, 0.4f } };
	b2Hull shapeHull = b2ComputeHull( points, 4 );
	b2Polygon polygon = b2MakePolygon( &shapeHull, 0.0f );
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = 0.1f;
	b2CreatePolygonShape( hull, &shapeDef, &polygon );
	AddBox( hull, 0.0f, 0.9f, 1.4f, 0.5f, 0.0f, 0.05f );
	AddBox( hull, -4.0f, 0.25f, 0.9f, 0.1f, 0.0f, 0.1f );

	// The blade tips must clear the stern. Joined bodies do not collide, so overlapping blades would crush
	// water against the hull, and that jams the motor.
	scene->driven = AddWheel( scene, worldId, hull, (b2Vec2){ -18.0f - 4.9f, deck + 0.25f }, 1.3f, 8, 0.2f, true, scene->control, 120.0f );
	return AddLiquid( worldId, def, (b2Vec2){ -29.9f, 0.05f }, (b2Vec2){ 29.9f, 3.1f } );
}

static b2fFluid* CreateDrum( b2WorldId worldId, LabScene* scene, const b2fFluidDef* def )
{
	float half = 5.0f, wall = 0.6f;
	b2BodyId drum = AddBody( scene, worldId, b2_kinematicBody, 0.0f, 8.0f );
	AddBox( drum, 0.0f, -half - wall, half + 2.0f * wall, wall, 0.0f, 0.0f );
	AddBox( drum, 0.0f, half + wall, half + 2.0f * wall, wall, 0.0f, 0.0f );
	AddBox( drum, -half - wall, 0.0f, wall, half, 0.0f, 0.0f );
	AddBox( drum, half + wall, 0.0f, wall, half, 0.0f, 0.0f );
	b2Body_SetAngularVelocity( drum, scene->control );
	scene->driven = drum;

	AddCrate( scene, worldId, -2.0f, 10.5f, 0.6f, 0.4f );
	AddCrate( scene, worldId, 2.0f, 10.5f, 0.6f, 2.0f );

	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = 0.6f;
	b2Capsule capsule = { { -0.7f, 0.0f }, { 0.7f, 0.0f }, 0.35f };
	b2CreateCapsuleShape( AddBody( scene, worldId, b2_dynamicBody, 0.0f, 12.3f ), &shapeDef, &capsule );

	shapeDef.density = 0.9f;
	b2Vec2 corners[6];
	for ( int i = 0; i < 6; ++i )
	{
		corners[i] = (b2Vec2){ 0.6f * cosf( B2_PI * (float)i / 3.0f ), 0.6f * sinf( B2_PI * (float)i / 3.0f ) };
	}
	b2Hull hexagonHull = b2ComputeHull( corners, 6 );
	b2Polygon hexagon = b2MakePolygon( &hexagonHull, 0.0f );
	b2CreatePolygonShape( AddBody( scene, worldId, b2_dynamicBody, 3.8f, 10.5f ), &shapeDef, &hexagon );
	return AddLiquid( worldId, def, (b2Vec2){ -4.9f, 3.1f }, (b2Vec2){ 4.9f, 7.5f } );
}

// A free wheel turned only by water, in a closed room so that splash stays in. The room drains through
// a gap under its right wall, where the head of water pushes out a jet, and a wide sink below catches
// the jet and feeds the stream at a steady rate.
static b2fFluid* CreateWheel( b2WorldId worldId, LabScene* scene, const b2fFluidDef* def )
{
	b2BodyId ground = AddBody( scene, worldId, b2_staticBody, 0.0f, 0.0f );
	float gap = LAB_WHEEL_GAP;
	AddBox( ground, 0.0f, -0.3f, 10.1f, 0.3f, 0.0f, 0.0f );
	AddBox( ground, -9.8f, 5.0f, 0.3f, 11.8f, 0.0f, 0.0f );
	AddBox( ground, 9.8f, 0.5f * ( gap + 16.8f ), 0.3f, 0.5f * ( 16.8f - gap ), 0.0f, 0.0f );
	AddBox( ground, 0.0f, 16.5f, 10.1f, 0.3f, 0.0f, 0.0f );

	// Sink, under the room and out past the jet
	AddBox( ground, 0.5f * ( LAB_WHEEL_SINK_END - 10.1f ), -7.1f, 0.5f * ( LAB_WHEEL_SINK_END + 10.1f ), 0.3f, 0.0f, 0.0f );
	AddBox( ground, LAB_WHEEL_SINK_END - 0.3f, -0.7f, 0.3f, 6.7f, 0.0f, 0.0f );

	scene->driven = AddWheel( scene, worldId, ground, (b2Vec2){ 0.0f, 7.5f }, 3.2f, 10, 0.3f, true, 0.0f, 1.0f );

	// A shallow layer over the whole sink, since a deep one would surge over the far wall
	return AddLiquid( worldId, def, (b2Vec2){ -9.4f, -6.7f }, (b2Vec2){ LAB_WHEEL_SINK_END - 0.7f, -4.2f } );
}

static b2fFluid* CreateWaves( b2WorldId worldId, LabScene* scene, const b2fFluidDef* def )
{
	b2BodyId ground = AddBody( scene, worldId, b2_staticBody, 0.0f, 0.0f );
	AddBox( ground, 0.0f, -1.0f, 25.0f, 1.0f, 0.0f, 0.0f );
	AddBox( ground, 24.0f, 4.0f, 1.0f, 5.0f, 0.0f, 0.0f );
	AddBox( ground, -24.0f, 4.0f, 1.0f, 5.0f, 0.0f, 0.0f );

	// A solid beach. Water starts on top of it, never inside it.
	float toe = 6.0f, slope = 4.0f / 17.0f, depth = 2.5f;
	b2Vec2 points[] = { { toe, 0.0f }, { 23.0f, 0.0f }, { 23.0f, 4.0f } };
	b2Hull hull = b2ComputeHull( points, 3 );
	b2Polygon beach = b2MakePolygon( &hull, 0.0f );
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	b2CreatePolygonShape( ground, &shapeDef, &beach );

	float start = -19.0f;
	scene->driven = AddBody( scene, worldId, b2_kinematicBody, start, 3.0f );
	AddBox( scene->driven, 0.0f, 0.0f, 0.5f, 3.0f, 0.0f, 0.0f );

	AddCrate( scene, worldId, -6.0f, 4.0f, 0.5f, 0.3f );
	AddCrate( scene, worldId, 0.0f, 4.0f, 0.5f, 0.5f );
	AddCrate( scene, worldId, 4.0f, 4.0f, 0.5f, 0.7f );

	b2fFluid* fluid = AddLiquid( worldId, def, (b2Vec2){ start + 0.7f, 0.05f }, (b2Vec2){ toe, depth } );
	float column = 0.5f;
	for ( float x = toe + 2.0f * def->particleSpacing; x + column < toe + depth / slope; x += column + def->particleSpacing )
	{
		float bottom = slope * ( x + column - toe ) + 2.0f * def->particleSpacing;
		if ( bottom < depth )
		{
			b2fFluid_CreateParticlesInBox( fluid, 0, (b2Pos){ x, bottom }, (b2Pos){ x + column, depth } );
		}
	}
	return fluid;
}

static b2fFluid* CreateSplash( b2WorldId worldId, LabScene* scene, const b2fFluidDef* def )
{
	b2BodyId ground = AddBody( scene, worldId, b2_staticBody, 0.0f, 0.0f );
	AddBox( ground, 0.0f, -1.0f, 12.0f, 1.0f, 0.0f, 0.0f );
	AddBox( ground, -11.0f, 6.0f, 1.0f, 7.0f, 0.0f, 0.0f );
	AddBox( ground, 11.0f, 6.0f, 1.0f, 7.0f, 0.0f, 0.0f );

	scene->driven = AddBody( scene, worldId, b2_dynamicBody, 4.0f, 10.0f );
	b2Circle ball = { { 0.0f, 0.0f }, 0.9f };
	b2ShapeDef shapeDef = b2DefaultShapeDef();
	shapeDef.density = scene->control;
	b2CreateCircleShape( scene->driven, &shapeDef, &ball );

	b2fFluid* fluid = AddLiquid( worldId, def, (b2Vec2){ -9.9f, 0.05f }, (b2Vec2){ 9.9f, 3.0f } );
	b2fFluid_CreateParticlesInBox( fluid, 0, (b2Pos){ -5.5f, 8.5f }, (b2Pos){ -2.5f, 11.5f } );
	return fluid;
}

// A deep pool over a small hole, with the sump pumped back to the surface so that it keeps flowing.
static b2fFluid* CreateDrain( b2WorldId worldId, LabScene* scene, const b2fFluidDef* def )
{
	b2BodyId ground = AddBody( scene, worldId, b2_staticBody, 0.0f, 0.0f );
	float half = 0.5f * scene->control, width = 6.0f;
	float outer = width + 0.6f;
	AddBox( ground, -0.5f * ( outer + half ), -0.25f, 0.5f * ( outer - half ), 0.25f, 0.0f, 0.0f );
	AddBox( ground, 0.5f * ( outer + half ), -0.25f, 0.5f * ( outer - half ), 0.25f, 0.0f, 0.0f );
	AddBox( ground, -width - 0.3f, 8.0f, 0.3f, 9.0f, 0.0f, 0.0f );
	AddBox( ground, width + 0.3f, 8.0f, 0.3f, 9.0f, 0.0f, 0.0f );

	// Sump, under the floor
	AddBox( ground, 0.0f, -4.3f, 4.3f, 0.3f, 0.0f, 0.0f );
	AddBox( ground, -4.0f, -2.5f, 0.3f, 2.0f, 0.0f, 0.0f );
	AddBox( ground, 4.0f, -2.5f, 0.3f, 2.0f, 0.0f, 0.0f );

	AddCrate( scene, worldId, -3.0f, 13.5f, 0.6f, 0.4f );
	return AddLiquid( worldId, def, (b2Vec2){ -5.9f, 0.05f }, (b2Vec2){ 5.9f, 12.0f } );
}

// A still pool and nothing else, to see how deep it can get before it stops being still
static b2fFluid* CreatePool( b2WorldId worldId, LabScene* scene, const b2fFluidDef* def )
{
	b2BodyId ground = AddBody( scene, worldId, b2_staticBody, 0.0f, 0.0f );
	float width = 5.0f, depth = scene->control, wall = 0.5f * depth + 2.0f;
	AddBox( ground, 0.0f, -0.5f, width + 1.0f, 0.5f, 0.0f, 0.0f );
	AddBox( ground, -width - 0.5f, wall, 0.5f, wall, 0.0f, 0.0f );
	AddBox( ground, width + 0.5f, wall, 0.5f, wall, 0.0f, 0.0f );
	return AddLiquid( worldId, def, (b2Vec2){ -width + def->particleSpacing, def->particleSpacing }, (b2Vec2){ width - def->particleSpacing, depth } );
}

// A square of thick liquid over a low tray. Tension rounds it as it falls. The Gravity slider scales the world's
// gravity, down to none, where it floats.
static b2fFluid* CreateDrop( b2WorldId worldId, LabScene* scene, const b2fFluidDef* def )
{
	b2BodyId ground = AddBody( scene, worldId, b2_staticBody, 0.0f, 0.0f );
	AddBox( ground, 0.0f, -0.5f, 6.0f, 0.5f, 0.0f, 0.0f );
	// Walls 6 m high: at cohesion 0.07 the landing throws droplets over 3 m ones
	AddBox( ground, -5.5f, 3.0f, 0.5f, 3.0f, 0.0f, 0.0f );
	AddBox( ground, 5.5f, 3.0f, 0.5f, 3.0f, 0.0f, 0.0f );
	return AddLiquid( worldId, def, (b2Vec2){ -1.0f, 5.0f }, (b2Vec2){ 1.0f, 7.0f } );
}

// Three liquids stacked the wrong way up: light oil at the bottom, water, then heavy syrup on top. They turn over,
// and a crate between oil and water in density comes to rest where they meet. Water is liquid 0 and takes the
// fluid's settings; the others change density, viscosity and cohesion.
static b2fFluid* CreateLayers( b2WorldId worldId, LabScene* scene, const b2fFluidDef* def )
{
	b2BodyId tank = AddBody( scene, worldId, b2_staticBody, 0.0f, 0.0f );
	float w = 4.0f, layer = 1.6f, t = 0.5f, s = def->particleSpacing;
	AddBox( tank, 0.0f, -t, w + 2.0f * t, t, 0.0f, 0.0f );
	AddBox( tank, -w - t, 4.0f, t, 4.0f + t, 0.0f, 0.0f );
	AddBox( tank, w + t, 4.0f, t, 4.0f + t, 0.0f, 0.0f );
	AddCrate( scene, worldId, 1.5f, 7.0f, 0.4f, 0.8f );

	b2fFluid* fluid = b2fCreateFluid( worldId, def );
	b2fLiquidDef oil = def->liquid, syrup = def->liquid;
	oil.massDensity = 0.6f * def->liquid.massDensity;
	syrup.massDensity = 1.6f * def->liquid.massDensity;
	oil.cohesion = 0.1f;
	syrup.cohesion = 0.1f;
	syrup.linearViscosity = 4.0f;
	int light = b2fFluid_CreateLiquid( fluid, &oil );
	int heavy = b2fFluid_CreateLiquid( fluid, &syrup );
	b2fFluid_CreateParticlesInBox( fluid, light, (b2Pos){ -w + s, s }, (b2Pos){ w - s, layer } );
	b2fFluid_CreateParticlesInBox( fluid, 0, (b2Pos){ -w + s, layer + s }, (b2Pos){ w - s, 2.0f * layer } );
	b2fFluid_CreateParticlesInBox( fluid, heavy, (b2Pos){ -w + s, 2.0f * layer + s }, (b2Pos){ w - s, 3.0f * layer } );
	return fluid;
}

// A bowl made of one static chain loop. Its left half has segments shorter than the interaction radius and its
// right half long ones, so the two show how the liquid meets the joints between segments. Chains collide on their
// right side, so the loop runs clockwise to hold the liquid in.
#define LAB_CHAIN_RADIUS 5.0f
#define LAB_CHAIN_FINE 60
#define LAB_CHAIN_COARSE 6

static b2fFluid* CreateChain( b2WorldId worldId, LabScene* scene, const b2fFluidDef* def )
{
	b2BodyId ground = AddBody( scene, worldId, b2_staticBody, 0.0f, LAB_CHAIN_RADIUS );
	b2Vec2 points[LAB_CHAIN_FINE + LAB_CHAIN_COARSE + 4];
	int count = 0;

	// Clockwise: down the left wall, along the bowl from left to right, up the right wall, back across the top
	float r = LAB_CHAIN_RADIUS, top = 3.0f;
	points[count++] = (b2Vec2){ -r, top };
	for ( int i = 0; i <= LAB_CHAIN_FINE; ++i )
	{
		float a = B2_PI + 0.5f * B2_PI * (float)i / (float)LAB_CHAIN_FINE;
		points[count++] = (b2Vec2){ r * cosf( a ), r * sinf( a ) };
	}
	for ( int i = 1; i <= LAB_CHAIN_COARSE; ++i )
	{
		float a = 1.5f * B2_PI + 0.5f * B2_PI * (float)i / (float)LAB_CHAIN_COARSE;
		points[count++] = (b2Vec2){ r * cosf( a ), r * sinf( a ) };
	}
	points[count++] = (b2Vec2){ r, top };

	// The loop's points go anticlockwise above, so reverse them
	for ( int i = 0; i < count / 2; ++i )
	{
		b2Vec2 t = points[i];
		points[i] = points[count - 1 - i];
		points[count - 1 - i] = t;
	}

	b2ChainDef chain = b2DefaultChainDef();
	chain.points = points;
	chain.pointCount = count;
	chain.isLoop = true;
	b2CreateChain( ground, &chain );

	AddCrate( scene, worldId, 1.5f, 6.0f, 0.4f, 0.5f );
	return AddLiquid( worldId, def, (b2Vec2){ -3.0f, 1.5f }, (b2Vec2){ 3.0f, 3.5f } );
}

// A small tank drains through a nozzle into a basin. With a thick liquid it hangs from the nozzle until a drop
// breaks off. Whatever the basin gains over its first fill goes back into the tank.
#define LAB_TAP_BASIN 1.2f

// Particles merge and split, so liquid is counted by area
static float BasinArea( b2fFluid* fluid )
{
	int n = b2fFluid_GetParticleCount( fluid );
	const b2Pos* pos = b2fFluid_GetPositions( fluid );
	const float* size = b2fFluid_GetSizes( fluid );
	float area = 0.0f;
	for ( int i = 0; i < n; ++i )
	{
		area += pos[i].y < LAB_TAP_BASIN + 1.0f ? size[i] * size[i] : 0.0f;
	}
	return area;
}

static b2fFluid* CreateTap( b2WorldId worldId, LabScene* scene, const b2fFluidDef* def )
{
	b2BodyId ground = AddBody( scene, worldId, b2_staticBody, 0.0f, 0.0f );

	// Basin
	AddBox( ground, 0.0f, -0.25f, 4.5f, 0.25f, 0.0f, 0.0f );
	// Walls up past the tank, since drops landing throw the odd droplet as high as 7 m
	AddBox( ground, -4.25f, 4.75f, 0.25f, 5.0f, 0.0f, 0.0f );
	AddBox( ground, 4.25f, 4.75f, 0.25f, 5.0f, 0.0f, 0.0f );

	// Tank, with the nozzle a short tube under a hole in its floor
	float half = 0.5f * scene->control, floor = 7.0f, wall = 0.1f;
	AddBox( ground, -1.0f - 0.5f * half, floor, 1.0f - 0.5f * half, 0.1f, 0.0f, 0.0f );
	AddBox( ground, 1.0f + 0.5f * half, floor, 1.0f - 0.5f * half, 0.1f, 0.0f, 0.0f );
	AddBox( ground, -2.1f, floor + 1.2f, 0.1f, 1.3f, 0.0f, 0.0f );
	AddBox( ground, 2.1f, floor + 1.2f, 0.1f, 1.3f, 0.0f, 0.0f );
	AddBox( ground, -half - wall, floor - 0.35f, wall, 0.3f, 0.0f, 0.0f );
	AddBox( ground, half + wall, floor - 0.35f, wall, 0.3f, 0.0f, 0.0f );

	b2fFluid* fluid = AddLiquid( worldId, def, (b2Vec2){ -3.9f, 0.05f }, (b2Vec2){ 3.9f, LAB_TAP_BASIN } );
	scene->keep = BasinArea( fluid );
	b2fFluid_CreateParticlesInBox( fluid, 0, (b2Pos){ -1.9f, floor + 0.15f }, (b2Pos){ 1.9f, floor + 1.5f } );
	return fluid;
}

b2fFluid* LabCreateScene( b2WorldId worldId, LabScene* scene, const b2fFluidDef* def )
{
	scene->bodyCount = 0;
	scene->time = 0.0f;
	scene->joint = b2_nullJointId;
	scene->driven = b2_nullBodyId;
	switch ( scene->type )
	{
		case lab_hole:
			return CreateHole( worldId, scene, def );
		case lab_boat:
			return CreateBoat( worldId, scene, def );
		case lab_drum:
			return CreateDrum( worldId, scene, def );
		case lab_wheel:
			return CreateWheel( worldId, scene, def );
		case lab_waves:
			return CreateWaves( worldId, scene, def );
		case lab_splash:
			return CreateSplash( worldId, scene, def );
		case lab_drain:
			return CreateDrain( worldId, scene, def );
		case lab_pool:
			return CreatePool( worldId, scene, def );
		case lab_drop:
			return CreateDrop( worldId, scene, def );
		case lab_layers:
			return CreateLayers( worldId, scene, def );
		case lab_chain:
			return CreateChain( worldId, scene, def );
		case lab_tap:
			return CreateTap( worldId, scene, def );
		default:
			return CreateTank( worldId, scene, def );
	}
}

void LabStepScene( LabScene* scene, b2fFluid* fluid, float timeStep )
{
	float t = scene->time;
	scene->time += timeStep;
	switch ( scene->type )
	{
		case lab_hole:
		{
			// Only the bottom of the sump, so the liquid is seen falling through the slot first
			b2AABB sump = { { 3.8f, -3.4f }, { 9.2f, -2.4f } };
			b2AABB source = { { -5.0f, 19.0f }, { 5.0f, 20.0f } };
			b2fFluid_MoveParticles( fluid, sump, source, (b2Vec2){ 0.0f, -2.0f }, 60 );
		}
		break;

		case lab_drain:
		{
			b2AABB sump = { { -3.8f, -4.1f }, { 3.8f, -0.8f } };
			b2AABB source = { { -5.0f, 13.5f }, { 5.0f, 14.5f } };
			int moved = b2fFluid_MoveParticles( fluid, sump, source, (b2Vec2){ 0.0f, -2.0f }, 80 );
			scene->pumped += moved;
			scene->stall = moved > 0 ? 0 : scene->stall + 1;
			scene->longestStall = scene->stall > scene->longestStall && t > 5.0f ? scene->stall : scene->longestStall;
		}
		break;

		case lab_boat:
			b2RevoluteJoint_SetTargetMotorSpeed( scene->joint, scene->control );
			break;

		case lab_drum:
			b2Body_SetAngularVelocity( scene->driven, scene->control );
			break;

		case lab_wheel:
		{
			b2AABB sump = { { -9.5f, -6.8f }, { LAB_WHEEL_SINK_END - 0.6f, -4.0f } };
			b2AABB source = { { 1.0f, 13.0f }, { 2.2f, 13.6f } };
			b2fFluid_MoveParticles( fluid, sump, source, (b2Vec2){ 0.0f, -4.0f }, (int)scene->control );
		}
		break;

		case lab_waves:
		{
			float omega = 2.0f * B2_PI / 3.5f;
			b2Body_SetLinearVelocity( scene->driven, (b2Vec2){ scene->control * omega * cosf( omega * t ), 0.0f } );
		}
		break;

		case lab_layers:
			for ( int a = 0; a < 3; ++a )
			{
				b2fFluid_SetCohesionBetween( fluid, a, ( a + 1 ) % 3, scene->control );
			}
			break;

		case lab_tap:
			// Every fourth frame, so that what was put in the tank last time has fallen clear
			if ( (int)( t / timeStep + 0.5f ) % 4 == 0 )
			{
				float extra = BasinArea( fluid ) - scene->keep;
				if ( extra > 0.0f )
				{
					b2AABB bottom = { { -4.0f, 0.0f }, { 4.0f, 0.3f } };
					b2AABB top = { { -1.8f, 8.6f }, { 1.8f, 8.8f } };
					b2fFluid_MoveParticles( fluid, bottom, top, (b2Vec2){ 0.0f, -1.0f }, 10 );
				}
			}
			break;

		case lab_drop:
			b2World_SetGravity( b2Body_GetWorld( scene->bodies[0] ), (b2Vec2){ 0.0f, -10.0f * scene->control } );
			break;

		case lab_splash:
			if ( fmodf( t, 7.0f ) < timeStep && t > 1.0f )
			{
				b2Body_SetTransform( scene->driven, (b2Pos){ 4.0f, 10.0f }, b2Rot_identity );
				b2Body_SetLinearVelocity( scene->driven, b2Vec2_zero );
				b2Body_SetAngularVelocity( scene->driven, 0.0f );
			}
			break;

		default:
			break;
	}

	// Anything that gets out of a scene is put back and counted, so that a leak shows up as a number
	b2AABB below = { { -1e6f, -1e6f }, { 1e6f, -30.0f } };
	b2AABB top = { { scene->viewCenter.x - 1.0f, scene->viewCenter.y }, { scene->viewCenter.x + 1.0f, scene->viewCenter.y + 1.0f } };
	scene->leaked += b2fFluid_MoveParticles( fluid, below, top, b2Vec2_zero, 200 );
}
