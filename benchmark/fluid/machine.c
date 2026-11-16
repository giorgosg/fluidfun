// SPDX-License-Identifier: MIT
#include "machine.h"

#include "box2d/math_functions.h"

#include <math.h>
#include <stddef.h>

// Static planks and walls are this thick, so that fast liquid does not cross them in a substep
#define MACHINE_THICKNESS 0.3f
// A socket is a pit a marble drops into and stays in, which is how a marble switches something on
#define MACHINE_SOCKET_WIDTH 0.9f
#define MACHINE_SOCKET_DEPTH 0.65f
// Relative to water
#define MACHINE_STONE 3.0f

MachineDef DefaultMachineDef( void )
{
	MachineDef def = { 0 };
	def.fluid = b2DefaultFluidDef();
	def.fluid.capacity = 65536;
	def.liquids[machine_water] = b2DefaultLiquidDef();
	def.liquids[machine_oil] = b2MakeOilDef();
	def.liquids[machine_honey] = b2MakeHoneyDef();
	def.liquids[machine_mercury] = b2MakeMercuryDef();
	def.liquids[machine_jelly] = b2MakeJellyDef();
	def.liquids[machine_dough] = b2MakeDoughDef();
	// As the Jelly And Dough scene has them: at water's friction they slide about like soap
	def.liquids[machine_jelly].friction = 0.7f;
	def.liquids[machine_dough].friction = 0.7f;
	// There is little of these two, and at one size they pour through small openings and show more
	def.liquids[machine_honey].tuning.adaptiveRatio = 1.0f;
	def.liquids[machine_mercury].tuning.adaptiveRatio = 1.0f;
	def.marbleRadius = 0.3f;
	def.marbleDensity = 0.5f;
	def.viewHalfHeight = 7.5f;
	def.viewHertz = 0.5f;
	def.only = section_typeCount;
	return def;
}

const char* MachineSectionName( MachineSectionType type )
{
	static const char* names[] = { "Start", "Sluice And Wheel", "Dominoes", "Ferry", "Turn", "Cradle", "Jelly Catapult", "Honey Balance", "Mercury Lift", "Finale" };
	return type < section_typeCount ? names[type] : "Whole";
}

// Section coordinates to the world's and back. A section going left is the mirror image of one going right.
static b2Vec2 At( const MachineSection* s, float x, float y )
{
	return (b2Vec2){ s->origin.x + s->direction * x, s->origin.y + y };
}

static b2Vec2 Local( const MachineSection* s, b2Vec2 p )
{
	return (b2Vec2){ s->direction * ( p.x - s->origin.x ), p.y - s->origin.y };
}

static b2Vec2 BodyLocal( const MachineSection* s, b2BodyId body )
{
	if ( b2Body_IsValid( body ) == false )
	{
		return (b2Vec2){ -1000.0f, -1000.0f };
	}
	return Local( s, b2ToVec2( b2Body_GetPosition( body ) ) );
}

static float Mix( float a, float b, float t )
{
	return a + t * ( b - a );
}

static bool Inside( b2Vec2 p, float x0, float y0, float x1, float y1 )
{
	return p.x >= x0 && p.x <= x1 && p.y >= y0 && p.y <= y1;
}

static void Grow( Machine* m, b2Vec2 p )
{
	m->bounds.lowerBound = b2Min( m->bounds.lowerBound, p );
	m->bounds.upperBound = b2Max( m->bounds.upperBound, p );
}

static void GroundBox( Machine* m, b2Vec2 center, float halfWidth, float halfHeight, float angle )
{
	b2Polygon box = b2MakeOffsetBox( halfWidth, halfHeight, center, b2MakeRot( angle ) );
	b2ShapeDef def = b2DefaultShapeDef();
	b2CreatePolygonShape( m->ground, &def, &box );
}

// A static box between two corners
static void Wall( Machine* m, const MachineSection* s, float x0, float y0, float x1, float y1 )
{
	b2Vec2 a = At( s, x0, y0 ), b = At( s, x1, y1 );
	Grow( m, a );
	Grow( m, b );
	GroundBox( m, b2Lerp( a, b, 0.5f ), 0.5f * b2AbsFloat( b.x - a.x ), 0.5f * b2AbsFloat( b.y - a.y ), 0.0f );
}

// A static plank whose top face runs between two points
static void Floor( Machine* m, const MachineSection* s, float x0, float y0, float x1, float y1 )
{
	b2Vec2 a = At( s, x0, y0 ), b = At( s, x1, y1 );
	Grow( m, a );
	Grow( m, b );
	b2Vec2 d = b2Sub( b, a );
	b2Vec2 normal = b2LeftPerp( b2Normalize( d ) );
	if ( normal.y < 0.0f )
	{
		normal = b2Neg( normal );
	}
	b2Vec2 center = b2MulAdd( b2Lerp( a, b, 0.5f ), -0.5f * MACHINE_THICKNESS, normal );
	GroundBox( m, center, 0.5f * b2Length( d ), 0.5f * MACHINE_THICKNESS, b2Atan2( d.y, d.x ) );
}

// A sloping floor of bars with gaps a marble rolls over and liquid falls through, solid for its last stretch
static void Bars( Machine* m, const MachineSection* s, float x0, float y0, float x1, float y1 )
{
	float slope = ( y1 - y0 ) / ( x1 - x0 );
	float x = x0;
	while ( x1 - x > 1.2f )
	{
		Floor( m, s, x, y0 + slope * ( x - x0 ), x + 0.3f, y0 + slope * ( x + 0.3f - x0 ) );
		x += 0.6f;
	}
	Floor( m, s, x, y0 + slope * ( x - x0 ), x1, y1 );
}

// The floor a marble rolls in on, ending in a socket whose near edge is at x. The caller builds the socket's far
// wall.
static void Socket( Machine* m, const MachineSection* s, float x )
{
	Floor( m, s, 0.0f, 0.0f, x, 0.0f );
	Wall( m, s, x - 0.3f, -MACHINE_SOCKET_DEPTH - 0.3f, x + MACHINE_SOCKET_WIDTH, -MACHINE_SOCKET_DEPTH );
	Wall( m, s, x - 0.3f, -MACHINE_SOCKET_DEPTH, x, -0.3f );
}

static bool InSocket( const Machine* m, const MachineSection* s, float x )
{
	return Inside( BodyLocal( s, m->baton ), x, -MACHINE_SOCKET_DEPTH, x + MACHINE_SOCKET_WIDTH, -0.05f );
}

static b2BodyId Body( Machine* m, const MachineSection* s, b2BodyType type, float x, float y, float angle )
{
	b2BodyDef def = b2DefaultBodyDef();
	def.type = type;
	def.position = b2ToPos( At( s, x, y ) );
	def.rotation = b2MakeRot( s->direction * angle );
	b2BodyId body = b2CreateBody( m->worldId, &def );
	if ( m->bodyCount < MACHINE_MAX_BODIES )
	{
		m->bodies[m->bodyCount++] = body;
	}
	return body;
}

// A box on a body, in the body's frame as the section sees it
static b2ShapeId BoxOn( b2BodyId body, const MachineSection* s, float x, float y, float halfWidth, float halfHeight, float angle, float density )
{
	b2Polygon box = b2MakeOffsetBox( halfWidth, halfHeight, (b2Vec2){ s->direction * x, y }, b2MakeRot( s->direction * angle ) );
	b2ShapeDef def = b2DefaultShapeDef();
	def.density = density;
	return b2CreatePolygonShape( body, &def, &box );
}

static b2BodyId Ball( Machine* m, const MachineSection* s, float x, float y, float radius, float density, float restitution )
{
	b2BodyId body = Body( m, s, b2_dynamicBody, x, y, 0.0f );
	b2Circle circle = { { 0.0f, 0.0f }, radius };
	b2ShapeDef def = b2DefaultShapeDef();
	def.density = density;
	def.material.restitution = restitution;
	b2CreateCircleShape( body, &def, &circle );
	return body;
}

static b2BodyId Marble( Machine* m, const MachineSection* s, float x, float y )
{
	return Ball( m, s, x, y, m->def.marbleRadius, m->def.marbleDensity, 0.1f );
}

// A hinge between the ground and a body, at the body's origin. Angles are as the section sees them.
static b2JointId Hinge( Machine* m, const MachineSection* s, b2BodyId body, bool limit, float lower, float upper, float friction )
{
	b2RevoluteJointDef def = b2DefaultRevoluteJointDef();
	def.base.bodyIdA = m->ground;
	def.base.bodyIdB = body;
	def.base.localFrameA.p = b2ToVec2( b2Body_GetPosition( body ) );
	def.enableLimit = limit;
	def.lowerAngle = s->direction > 0.0f ? lower : -upper;
	def.upperAngle = s->direction > 0.0f ? upper : -lower;
	def.enableMotor = friction > 0.0f;
	def.maxMotorTorque = friction;
	return b2CreateRevoluteJoint( m->worldId, &def );
}

// A wheel of flat blades on a hub
static b2BodyId Wheel( Machine* m, const MachineSection* s, float x, float y, float radius, int blades, float density )
{
	b2BodyId wheel = Body( m, s, b2_dynamicBody, x, y, 0.0f );
	b2Circle hub = { { 0.0f, 0.0f }, 0.2f * radius };
	b2ShapeDef def = b2DefaultShapeDef();
	def.density = density;
	b2CreateCircleShape( wheel, &def, &hub );
	for ( int i = 0; i < blades; ++i )
	{
		float angle = 2.0f * B2_PI * (float)i / (float)blades;
		float middle = 0.6f * radius;
		BoxOn( wheel, s, middle * cosf( angle ), middle * sinf( angle ), 0.4f * radius, 0.05f * radius, angle, density );
	}
	return wheel;
}

static void Spawn( Machine* m, const MachineSection* s, MachineLiquid liquid, float x0, float y0, float x1, float y1 )
{
	b2Vec2 a = At( s, x0, y0 ), b = At( s, x1, y1 );
	b2Liquid_SpawnBox( m->liquids[liquid], b2ToPos( b2Min( a, b ) ), b2ToPos( b2Max( a, b ) ), b2Vec2_zero );
}

// Moves a kinematic body toward a point in the section. Returns true once it is there, and stops it.
static bool SlideTo( const MachineSection* s, b2BodyId body, float x, float y, float speed, float timeStep )
{
	b2Vec2 d = b2Sub( At( s, x, y ), b2ToVec2( b2Body_GetPosition( body ) ) );
	float distance = b2Length( d );
	if ( distance <= speed * timeStep )
	{
		b2Body_SetLinearVelocity( body, b2Vec2_zero );
		return true;
	}
	b2Body_SetLinearVelocity( body, b2MulSV( speed / distance, d ) );
	return false;
}

static float Speed( b2BodyId body )
{
	return b2Body_IsValid( body ) ? b2Length( b2Body_GetLinearVelocity( body ) ) : 0.0f;
}

static void Advance( MachineSection* s )
{
	s->phase += 1;
	s->clock = 0.0f;
}

// Hands the baton to the marble this section sends on
static void PassBaton( Machine* m, MachineSection* s )
{
	m->baton = s->marble;
	b2Body_SetAwake( s->marble, true );
	Advance( s );
}

// Where to look, in section coordinates. Scale is in units of the view the machine was asked for.
static void Look( Machine* m, const MachineSection* s, float x, float y, float scale )
{
	m->focus = At( s, x, y );
	m->focusHalfHeight = scale * m->def.viewHalfHeight;
}

// Looks a little ahead of a body, and a little below it, where what comes next is waiting
static void Follow( Machine* m, const MachineSection* s, b2BodyId body, float scale )
{
	b2Vec2 p = BodyLocal( s, body );
	Look( m, s, p.x + 0.35f * m->def.viewHalfHeight, p.y - 0.15f * m->def.viewHalfHeight, scale );
}

// ---------------------------------------------------------------------------------------------------------------------
// Start: a marble behind a gate at the top of a ramp

static void CreateStart( Machine* m, MachineSection* s )
{
	Floor( m, s, -9.0f, 2.2f, -7.5f, 1.8f );
	Floor( m, s, -7.5f, 1.8f, -1.5f, 0.0f );
	Floor( m, s, -1.5f, 0.0f, 0.0f, 0.0f );
	Wall( m, s, -9.4f, 1.9f, -9.0f, 3.4f );

	// A lone finale needs a marble that sinks
	bool stone = m->def.only == section_finale;
	m->baton = Ball( m, s, -8.5f, 2.1f + m->def.marbleRadius, m->def.marbleRadius, stone ? MACHINE_STONE : m->def.marbleDensity, 0.1f );
	s->parts[0] = Body( m, s, b2_kinematicBody, -7.9f, 2.4f, 0.0f );
	BoxOn( s->parts[0], s, 0.0f, 0.0f, 0.1f, 0.5f, 0.0f, 0.0f );
	s->exit = s->origin;
}

static void StepStart( Machine* m, MachineSection* s, bool active, float timeStep )
{
	if ( s->phase == 0 && s->clock > 1.5f )
	{
		b2Body_SetAwake( m->baton, true );
		Advance( s );
	}
	else if ( s->phase == 1 && SlideTo( s, s->parts[0], -7.9f, 3.6f, 2.0f, timeStep ) )
	{
		Advance( s );
	}

	if ( active )
	{
		Follow( m, s, m->baton, 1.0f );
	}
}

// ---------------------------------------------------------------------------------------------------------------------
// Sluice and wheel: the marble drops into a socket, which lifts the gate of a reservoir. The water runs down a spout,
// off a baffle and over a wheel into a sump, where a marble waits on the bottom. The sump holds less than the
// reservoir, so the marble floats up and out over its low wall. The water drains through the bars of the way out.

static void CreateSluice( Machine* m, MachineSection* s )
{
	float spacing = m->def.fluid.particleSpacing;
	Socket( m, s, 2.2f );

	// Reservoir, with the gate in its right wall
	Wall( m, s, 3.1f, -0.95f, 3.5f, 7.4f );
	Wall( m, s, 3.5f, 2.5f, 9.4f, 2.9f );
	Floor( m, s, 3.5f, 3.4f, 9.0f, 2.9f );
	Wall( m, s, 9.0f, 3.85f, 9.4f, 7.4f );
	s->parts[0] = Body( m, s, b2_kinematicBody, 9.2f, 3.4f, 0.0f );
	BoxOn( s->parts[0], s, 0.0f, 0.0f, 0.2f, 0.5f, 0.0f, 0.0f );
	Spawn( m, s, machine_water, 3.5f + spacing, 3.4f + spacing, 9.0f - spacing, 6.9f );

	// Spout and baffle, which put the water on the far half of the wheel however fast it leaves
	Floor( m, s, 9.4f, 2.9f, 11.9f, 2.7f );
	Wall( m, s, 12.9f, 2.3f, 13.2f, 5.0f );
	s->parts[1] = Wheel( m, s, 11.6f, 0.0f, 2.0f, 8, 0.3f );
	s->joint = Hinge( m, s, s->parts[1], false, 0.0f, 0.0f, 2.0f );

	// Sump
	Wall( m, s, 9.0f, -4.9f, 15.4f, -4.6f );
	Wall( m, s, 9.0f, -4.6f, 9.4f, 2.5f );
	Wall( m, s, 15.0f, -6.3f, 15.4f, -2.9f );
	Wall( m, s, 15.0f, -1.6f, 15.4f, 5.0f );

	// The marble waits in a chimney beside the low wall, open at the foot by less than a marble. All the water that
	// leaves the sump comes up through it, so it carries the marble out. In an open sump the marble drifted under
	// the wheel and stayed there.
	Wall( m, s, 13.95f, -4.2f, 14.15f, -1.6f );
	s->marble = Marble( m, s, 14.57f, -4.6f + m->def.marbleRadius );

	// The way out, over a tank that catches what the sump spills
	Bars( m, s, 15.4f, -2.9f, 19.5f, -3.7f );
	Wall( m, s, 15.4f, -6.3f, 19.5f, -6.0f );
	Wall( m, s, 19.1f, -6.0f, 19.5f, -4.1f );
	// A curtain over the end of the bars, for spray
	Wall( m, s, 19.2f, -2.7f, 19.5f, 0.5f );
	s->exit = At( s, 19.5f, -3.7f );
}

static void StepSluice( Machine* m, MachineSection* s, bool active, float timeStep )
{
	if ( s->phase == 0 && InSocket( m, s, 2.2f ) )
	{
		Advance( s );
	}
	else if ( s->phase == 1 && SlideTo( s, s->parts[0], 9.2f, 4.5f, 1.0f, timeStep ) )
	{
		Advance( s );
	}
	else if ( s->phase == 2 && BodyLocal( s, s->marble ).x > 15.2f )
	{
		PassBaton( m, s );
	}

	if ( active == false )
	{
		return;
	}

	if ( s->phase == 0 )
	{
		Follow( m, s, m->baton, 1.0f );
	}
	else if ( s->phase < 3 )
	{
		// The wheel first, then the marble as the sump lifts it
		b2Vec2 p = BodyLocal( s, s->marble );
		float lifted = b2ClampFloat( ( p.y + 4.3f ) / 1.4f, 0.0f, 1.0f );
		Look( m, s, Mix( 11.0f, p.x + 1.0f, lifted ), Mix( 0.3f, p.y, lifted ), Mix( 1.15f, 1.0f, lifted ) );
	}
	else
	{
		Follow( m, s, m->baton, 1.0f );
	}
}

// ---------------------------------------------------------------------------------------------------------------------
// Dominoes, each bigger than the one before, and a marble under the last

#define MACHINE_DOMINOES 8

static void CreateDominoes( Machine* m, MachineSection* s )
{
	Floor( m, s, 0.0f, 0.0f, 7.0f, 0.0f );
	Floor( m, s, 7.0f, 0.0f, 13.0f, -0.6f );
	float x = 1.5f, height = 0.6f;
	for ( int i = 0; i < MACHINE_DOMINOES; ++i )
	{
		b2BodyId domino = Body( m, s, b2_dynamicBody, x, 0.5f * height, 0.0f );
		BoxOn( domino, s, 0.0f, 0.0f, height / 12.0f, 0.5f * height, 0.0f, 0.4f );
		if ( i == MACHINE_DOMINOES - 1 )
		{
			// On a post beside the last one, which strikes it sideways early in its fall. On the floor the domino
			// came down on top of the marble and pinned it.
			Wall( m, s, x + 0.55f, 0.0f, x + 0.95f, 0.75f * height );
			s->marble = Marble( m, s, x + 0.72f, 0.75f * height + m->def.marbleRadius );
		}
		x += 0.55f * height;
		height *= 1.22f;
	}
	s->exit = At( s, 13.0f, -0.6f );
}

static void StepDominoes( Machine* m, MachineSection* s, bool active, float timeStep )
{
	(void)timeStep;
	if ( s->phase == 0 && active && Speed( s->marble ) > 0.5f )
	{
		PassBaton( m, s );
	}

	if ( active == false )
	{
		return;
	}

	if ( s->phase == 0 && BodyLocal( s, m->baton ).x > 0.5f )
	{
		Look( m, s, 5.5f, 0.5f, 0.9f );
	}
	else
	{
		Follow( m, s, m->baton, 1.0f );
	}
}

// ---------------------------------------------------------------------------------------------------------------------
// Ferry: the marble falls through a hole in the dock into a boat, whose paddle wheel starts. The boat crosses the
// lake and its bowsprit pushes over a flap on the far shore, which falls on a marble.

#define MACHINE_FERRY_SPEED 4.0f

static void CreateFerry( Machine* m, MachineSection* s )
{
	float spacing = m->def.fluid.particleSpacing;

	// Dock, with a hole and a post that stops the marble over it
	Floor( m, s, 0.0f, 0.0f, 4.0f, 0.0f );
	Wall( m, s, 4.7f, 0.0f, 5.0f, 1.0f );

	// Lake
	Wall( m, s, 0.0f, -4.8f, 0.4f, -0.3f );
	Wall( m, s, 0.0f, -4.8f, 20.4f, -4.5f );
	Wall( m, s, 20.0f, -4.5f, 20.4f, -2.2f );
	Spawn( m, s, machine_water, 0.4f + spacing, -4.5f + spacing, 20.0f - spacing, -2.5f );

	// Boat: a floor, two sides and a bowsprit
	b2BodyId hull = Body( m, s, b2_dynamicBody, 4.4f, -2.3f, 0.0f );
	BoxOn( hull, s, 0.0f, -0.25f, 1.7f, 0.15f, 0.0f, 0.3f );
	BoxOn( hull, s, -1.6f, 0.25f, 0.1f, 0.4f, 0.0f, 0.3f );
	BoxOn( hull, s, 1.6f, 0.25f, 0.1f, 0.4f, 0.0f, 0.3f );
	BoxOn( hull, s, 2.7f, 0.65f, 1.0f, 0.06f, 0.0f, 0.1f );
	s->parts[0] = hull;

	// The blade tips must clear the stern: joined bodies do not collide, and blades that overlap the hull crush water
	// against it
	b2BodyId wheel = Wheel( m, s, 4.4f - 2.6f, -2.15f, 0.8f, 6, 0.2f );
	b2RevoluteJointDef joint = b2DefaultRevoluteJointDef();
	joint.base.bodyIdA = hull;
	joint.base.bodyIdB = wheel;
	joint.base.localFrameA.p = b2Body_GetLocalPoint( hull, b2Body_GetPosition( wheel ) );
	joint.enableMotor = true;
	joint.motorSpeed = 0.0f;
	joint.maxMotorTorque = 30.0f;
	s->joint = b2CreateRevoluteJoint( m->worldId, &joint );
	s->parts[1] = wheel;

	// Far shore: a flap hinged at its foot, and a marble on a post that the flap knocks off
	Floor( m, s, 20.4f, -2.2f, 21.8f, -2.2f );
	Floor( m, s, 21.8f, -2.2f, 24.0f, -2.8f );
	b2BodyId flap = Body( m, s, b2_dynamicBody, 20.8f, -2.2f, 0.0f );
	BoxOn( flap, s, 0.0f, 0.6f, 0.08f, 0.6f, 0.0f, 0.5f );
	Hinge( m, s, flap, true, -1.5f, 0.05f, 0.3f );
	s->parts[2] = flap;
	Wall( m, s, 21.05f, -2.2f, 21.45f, -1.6f );
	s->marble = Marble( m, s, 21.25f, -1.6f + m->def.marbleRadius );
	s->exit = At( s, 24.0f, -2.8f );
}

static void StepFerry( Machine* m, MachineSection* s, bool active, float timeStep )
{
	(void)timeStep;
	if ( s->phase == 0 && b2Body_IsValid( m->baton ) )
	{
		b2Vec2 p = b2Body_GetLocalPoint( s->parts[0], b2Body_GetPosition( m->baton ) );
		if ( Inside( p, -1.5f, -0.3f, 1.5f, 0.7f ) )
		{
			// Clockwise for a boat going right, so that the blades push the water back
			b2RevoluteJoint_SetTargetMotorSpeed( s->joint, -s->direction * MACHINE_FERRY_SPEED );
			b2Body_SetAwake( s->parts[0], true );
			Advance( s );
		}
	}
	else if ( s->phase == 1 && ( Speed( s->marble ) > 0.5f || BodyLocal( s, s->marble ).x > 21.8f ) )
	{
		b2RevoluteJoint_SetTargetMotorSpeed( s->joint, 0.0f );
		PassBaton( m, s );
	}

	if ( active == false )
	{
		return;
	}

	if ( s->phase == 1 )
	{
		b2Vec2 p = BodyLocal( s, s->parts[0] );
		Look( m, s, p.x + 0.4f * m->def.viewHalfHeight, p.y + 0.5f, 1.0f );
	}
	else
	{
		Follow( m, s, m->baton, 1.0f );
	}
}

// ---------------------------------------------------------------------------------------------------------------------
// Turn: ramps back and forth down to the next row, which runs the other way. The count of ramps is even.

#define MACHINE_TURN_PITCH 2.4f

static void CreateTurn( Machine* m, MachineSection* s )
{
	int ramps = s->count;
	float last = -MACHINE_TURN_PITCH * (float)( ramps - 1 );
	for ( int i = 0; i < ramps; ++i )
	{
		float y = -MACHINE_TURN_PITCH * (float)i;
		if ( i % 2 == 0 )
		{
			Floor( m, s, i == 0 ? 0.0f : 0.6f, y, 5.6f, y - 1.0f );
		}
		else if ( i == ramps - 1 )
		{
			Floor( m, s, 6.6f, y, 0.0f, y - 1.2f );
		}
		else
		{
			Floor( m, s, 6.6f, y, 1.6f, y - 1.0f );
		}
	}
	Wall( m, s, 6.6f, last - 0.3f, 7.0f, 1.2f );
	Wall( m, s, 0.2f, last + 0.7f, 0.6f, -0.3f );
	s->exit = At( s, 0.0f, last - 1.2f );
}

static void StepTurn( Machine* m, MachineSection* s, bool active, float timeStep )
{
	(void)timeStep;
	if ( active )
	{
		// Straight down the middle of the ramps, and well below, to show the row the marble is coming to
		b2Vec2 p = BodyLocal( s, m->baton );
		Look( m, s, 0.5f * ( p.x + 3.3f ), p.y - 0.3f * m->def.viewHalfHeight, 1.0f );
	}
}

// ---------------------------------------------------------------------------------------------------------------------
// Cradle: five balls on rods. The marble strikes the first and the last strikes the marble beside it.

#define MACHINE_CRADLE_BALLS 5

static void CreateCradle( Machine* m, MachineSection* s )
{
	Floor( m, s, 0.0f, 0.0f, 7.2f, 0.0f );
	Floor( m, s, 7.2f, 0.0f, 10.0f, -0.5f );
	Wall( m, s, 2.4f, 4.45f, 6.8f, 4.7f );
	float radius = 0.4f, rod = 4.0f;
	for ( int i = 0; i < MACHINE_CRADLE_BALLS; ++i )
	{
		// The body's origin is the pivot
		b2BodyId ball = Body( m, s, b2_dynamicBody, 3.0f + 2.0f * radius * (float)i, 0.45f + rod, 0.0f );
		b2Circle circle = { { 0.0f, -rod }, radius };
		b2ShapeDef def = b2DefaultShapeDef();
		def.material.restitution = 1.0f;
		def.material.friction = 0.0f;
		b2CreateCircleShape( ball, &def, &circle );

		// The rod is only there to be seen
		b2Polygon box = b2MakeOffsetBox( 0.03f, 0.5f * rod, (b2Vec2){ 0.0f, -0.5f * rod }, b2Rot_identity );
		def.density = 0.01f;
		def.filter.maskBits = 0;
		b2CreatePolygonShape( ball, &def, &box );
		Hinge( m, s, ball, false, 0.0f, 0.0f, 0.0f );
	}
	float end = 3.0f + 2.0f * radius * (float)( MACHINE_CRADLE_BALLS - 1 ) + radius;
	s->marble = Marble( m, s, end + m->def.marbleRadius + 0.02f, m->def.marbleRadius );
	s->exit = At( s, 10.0f, -0.5f );
}

static void StepCradle( Machine* m, MachineSection* s, bool active, float timeStep )
{
	(void)timeStep;
	if ( s->phase == 0 && active && Speed( s->marble ) > 0.3f )
	{
		PassBaton( m, s );
	}

	if ( active )
	{
		Follow( m, s, m->baton, 1.0f );
	}
}

// ---------------------------------------------------------------------------------------------------------------------
// Jelly catapult: the marble's socket swings two trapdoors open. A block of dough lands on the floor and stays as it
// lands. A block of jelly lands on the high end of a seesaw, whose low end throws a marble.

#define MACHINE_SEESAW_LOW -0.5f
#define MACHINE_SEESAW_HIGH -0.12f

static void CreateJelly( Machine* m, MachineSection* s )
{
	float spacing = m->def.fluid.particleSpacing;
	Socket( m, s, 2.0f );
	Wall( m, s, 2.9f, -3.3f, 3.2f, 0.6f );
	Wall( m, s, 2.9f, -3.3f, 11.15f, -3.0f );
	Wall( m, s, 5.6f, -3.0f, 5.8f, -1.8f );

	// Trapdoors hinged at their near ends, which are the bodies' origins
	s->parts[0] = Body( m, s, b2_kinematicBody, 3.4f, 3.0f, 0.0f );
	BoxOn( s->parts[0], s, 1.0f, 0.0f, 1.0f, 0.08f, 0.0f, 0.0f );
	Spawn( m, s, machine_dough, 3.7f, 3.08f + spacing, 5.1f, 4.5f );
	s->parts[1] = Body( m, s, b2_kinematicBody, 6.1f, 3.0f, 0.0f );
	BoxOn( s->parts[1], s, 1.1f, 0.0f, 1.1f, 0.08f, 0.0f, 0.0f );
	Spawn( m, s, machine_jelly, 6.4f, 3.08f + spacing, 8.0f, 4.7f );

	// The seesaw stops short of level, so that its low end still slopes down and outward at the top of its throw
	b2BodyId beam = Body( m, s, b2_dynamicBody, 8.6f, -1.5f, MACHINE_SEESAW_LOW );
	BoxOn( beam, s, 0.0f, 0.0f, 2.6f, 0.1f, 0.0f, 1.0f );
	Hinge( m, s, beam, true, MACHINE_SEESAW_LOW, MACHINE_SEESAW_HIGH, 0.0f );
	s->parts[2] = beam;

	// The marble rests on the low end against a wall the end rises above
	Wall( m, s, 10.95f, -3.0f, 11.15f, -1.8f );
	s->marble = Marble( m, s, 10.55f, -2.09f );

	// The way out, under a hood that turns a high throw back down
	Floor( m, s, 11.15f, -1.8f, 15.0f, -2.5f );
	Floor( m, s, 11.6f, 2.0f, 15.0f, -0.9f );
	s->exit = At( s, 15.0f, -2.5f );
}

static void StepJelly( Machine* m, MachineSection* s, bool active, float timeStep )
{
	(void)timeStep;
	if ( s->phase == 0 && InSocket( m, s, 2.0f ) )
	{
		b2Body_SetAngularVelocity( s->parts[0], -s->direction * 4.0f );
		Advance( s );
	}
	else if ( s->phase == 1 && s->clock > 0.8f )
	{
		b2Body_SetAngularVelocity( s->parts[1], -s->direction * 4.0f );
		Advance( s );
	}
	else if ( s->phase == 2 && BodyLocal( s, s->marble ).x > 11.3f )
	{
		PassBaton( m, s );
	}

	// A trapdoor stops once it hangs straight down
	for ( int i = 0; i < 2; ++i )
	{
		if ( b2AbsFloat( b2Rot_GetAngle( b2Body_GetRotation( s->parts[i] ) ) ) > 0.5f * B2_PI && b2Body_GetAngularVelocity( s->parts[i] ) != 0.0f )
		{
			b2Body_SetAngularVelocity( s->parts[i], 0.0f );
		}
	}

	if ( active == false )
	{
		return;
	}

	if ( s->phase == 0 || s->phase == 3 )
	{
		Follow( m, s, m->baton, 1.0f );
	}
	else
	{
		Look( m, s, 7.5f, 0.3f, 1.0f );
	}
}

// ---------------------------------------------------------------------------------------------------------------------
// Honey balance: the marble's socket opens a hopper of honey over a bucket on one arm of a balance. A weight on the
// other arm holds a tooth down in front of a marble on a slope, until the honey outweighs it.

#define MACHINE_BALANCE_TILT 0.25f

static void CreateHoney( Machine* m, MachineSection* s )
{
	float spacing = m->def.fluid.particleSpacing;
	Socket( m, s, 2.0f );
	Wall( m, s, 2.9f, -1.9f, 3.2f, 0.6f );
	Wall( m, s, 2.9f, -1.9f, 8.5f, -1.6f );
	Wall( m, s, 8.2f, -1.6f, 8.5f, -0.7f );

	// Hopper, with a gate under the hole in its floor
	Wall( m, s, 5.0f, 4.3f, 5.72f, 4.5f );
	Wall( m, s, 6.08f, 4.3f, 6.8f, 4.5f );
	Wall( m, s, 5.0f, 4.5f, 5.2f, 6.6f );
	Wall( m, s, 6.6f, 4.5f, 6.8f, 6.6f );
	s->parts[0] = Body( m, s, b2_kinematicBody, 5.9f, 4.2f, 0.0f );
	BoxOn( s->parts[0], s, 0.0f, 0.0f, 0.4f, 0.1f, 0.0f, 0.0f );
	Spawn( m, s, machine_honey, 5.2f + spacing, 4.5f + spacing, 6.6f - spacing, 6.1f );

	// Balance: the beam is the bucket's floor
	b2BodyId beam = Body( m, s, b2_dynamicBody, 7.5f, 0.5f, -MACHINE_BALANCE_TILT );
	BoxOn( beam, s, 0.0f, 0.0f, 2.5f, 0.1f, 0.0f, 0.5f );
	BoxOn( beam, s, -2.4f, 0.8f, 0.06f, 0.7f, 0.0f, 0.5f );
	BoxOn( beam, s, -0.9f, 0.8f, 0.06f, 0.7f, 0.0f, 0.5f );
	BoxOn( beam, s, 2.0f, 0.4f, 0.3f, 0.3f, 0.0f, 4.0f );
	BoxOn( beam, s, 2.42f, -0.6f, 0.08f, 0.5f, 0.0f, 0.5f );
	Hinge( m, s, beam, true, -MACHINE_BALANCE_TILT, MACHINE_BALANCE_TILT, 0.0f );
	s->parts[1] = beam;

	// The marble's slope runs under the tooth
	Floor( m, s, 8.5f, -1.3f, 14.0f, -2.0f );
	s->marble = Marble( m, s, 9.15f, -1.38f + m->def.marbleRadius );
	s->exit = At( s, 14.0f, -2.0f );
}

static void StepHoney( Machine* m, MachineSection* s, bool active, float timeStep )
{
	if ( s->phase == 0 && InSocket( m, s, 2.0f ) )
	{
		Advance( s );
	}
	else if ( s->phase == 1 && SlideTo( s, s->parts[0], 5.0f, 4.2f, 1.0f, timeStep ) )
	{
		Advance( s );
	}
	else if ( s->phase == 2 && BodyLocal( s, s->marble ).x > 10.3f )
	{
		PassBaton( m, s );
	}

	if ( active == false )
	{
		return;
	}

	if ( s->phase == 0 || s->phase == 3 )
	{
		Follow( m, s, m->baton, 1.0f );
	}
	else
	{
		Look( m, s, 7.5f, 1.8f, 0.9f );
	}
}

// ---------------------------------------------------------------------------------------------------------------------
// Mercury lift: the marble's socket lifts the gate of a flask of mercury, which runs into a shaft with a stone ball
// at the bottom. Stone floats high on mercury, so the ball rises with it and rolls out over the shaft's low wall. A
// steel ball floated too, but so low that it stayed behind the wall once the mercury stopped running over it.

static void CreateMercury( Machine* m, MachineSection* s )
{
	float spacing = m->def.fluid.particleSpacing;
	Socket( m, s, 2.0f );

	// Flask
	Wall( m, s, 2.9f, -0.95f, 3.6f, 0.6f );
	Wall( m, s, 2.9f, 0.6f, 6.8f, 1.0f );
	Floor( m, s, 3.6f, 1.5f, 6.4f, 1.0f );
	Wall( m, s, 3.2f, 1.0f, 3.6f, 3.8f );
	Wall( m, s, 6.4f, 1.95f, 6.8f, 3.8f );
	s->parts[0] = Body( m, s, b2_kinematicBody, 6.6f, 1.5f, 0.0f );
	BoxOn( s->parts[0], s, 0.0f, 0.0f, 0.2f, 0.5f, 0.0f, 0.0f );
	Spawn( m, s, machine_mercury, 3.6f + spacing, 1.5f + spacing, 6.4f - spacing, 3.3f );

	// Chute and shaft. The mercury leaves the flask fast enough to clear the shaft, so a board in its mouth turns it
	// down, and another over the low wall stops what splashes.
	Floor( m, s, 6.8f, 1.0f, 8.0f, 0.4f );
	Wall( m, s, 7.6f, -4.3f, 8.1f, 0.1f );
	Wall( m, s, 7.6f, -4.3f, 9.6f, -4.0f );
	Wall( m, s, 9.2f, -4.0f, 9.6f, -1.2f );
	Wall( m, s, 9.2f, -0.2f, 9.6f, 3.8f );
	Wall( m, s, 8.55f, -0.3f, 8.75f, 1.6f );
	s->marble = Ball( m, s, 8.65f, -4.0f + m->def.marbleRadius, m->def.marbleRadius, MACHINE_STONE, 0.1f );

	Bars( m, s, 9.6f, -1.2f, 13.5f, -2.0f );
	Wall( m, s, 9.6f, -3.8f, 13.5f, -3.5f );
	Wall( m, s, 13.1f, -3.5f, 13.5f, -2.4f );
	s->exit = At( s, 13.5f, -2.0f );
}

static void StepMercury( Machine* m, MachineSection* s, bool active, float timeStep )
{
	// The gate opens only a little: the ball rises slowly through the mercury, and the shaft has to be still running
	// over when it gets to the top, for the flow to carry it out
	if ( s->phase == 0 && InSocket( m, s, 2.0f ) )
	{
		Advance( s );
	}
	else if ( s->phase == 1 && SlideTo( s, s->parts[0], 6.6f, 1.8f, 1.0f, timeStep ) )
	{
		Advance( s );
	}
	else if ( s->phase == 2 && BodyLocal( s, s->marble ).x > 9.4f )
	{
		PassBaton( m, s );
	}

	if ( active == false )
	{
		return;
	}

	if ( s->phase == 0 || s->phase == 3 )
	{
		Follow( m, s, m->baton, 1.0f );
	}
	else
	{
		Look( m, s, 7.5f, -0.8f, 0.8f );
	}
}

// ---------------------------------------------------------------------------------------------------------------------
// Finale: a pool of oil over water with crates afloat between them. The stone ball sinks to the detonator on the
// floor, and the pool goes up.

static void CreateFinale( Machine* m, MachineSection* s )
{
	float spacing = m->def.fluid.particleSpacing;
	Floor( m, s, 0.0f, 0.0f, 3.0f, 0.0f );
	Wall( m, s, 2.6f, -6.0f, 3.0f, -0.3f );
	Wall( m, s, 2.6f, -6.3f, 15.4f, -6.0f );
	Wall( m, s, 15.0f, -6.0f, 15.4f, 1.0f );
	Wall( m, s, 8.5f, -6.0f, 9.5f, -5.85f );
	Spawn( m, s, machine_water, 3.0f + spacing, -6.0f + spacing, 15.0f - spacing, -3.6f );
	Spawn( m, s, machine_oil, 3.0f + spacing, -3.6f + spacing, 15.0f - spacing, -2.4f );

	static const float densities[] = { 0.8f, 0.3f, 0.8f, 0.45f, 0.8f };
	for ( int i = 0; i < 5; ++i )
	{
		b2BodyId crate = Body( m, s, b2_dynamicBody, 6.0f + 1.8f * (float)i, -1.7f, 0.0f );
		BoxOn( crate, s, 0.0f, 0.0f, 0.4f, 0.4f, 0.0f, densities[i] );
	}
	s->exit = At( s, 15.4f, 0.0f );
}

static void StepFinale( Machine* m, MachineSection* s, bool active, float timeStep )
{
	(void)timeStep;
	b2Vec2 p = BodyLocal( s, m->baton );
	if ( s->phase == 0 && active && p.y < -5.3f )
	{
		b2ExplosionDef explosion = b2DefaultExplosionDef();
		explosion.position = b2Body_GetPosition( m->baton );
		explosion.radius = 2.5f;
		explosion.falloff = 3.0f;
		explosion.impulsePerLength = 60.0f;
		b2World_Explode( m->worldId, &explosion );
		Advance( s );
	}
	else if ( s->phase == 1 && s->clock > 3.5f )
	{
		Advance( s );
	}
	else if ( s->phase == 2 && s->clock > 5.0f )
	{
		m->finished = true;
	}

	if ( active == false )
	{
		return;
	}

	if ( s->phase == 0 )
	{
		Look( m, s, b2MaxFloat( p.x, 2.0f ) + 3.0f, b2MinFloat( p.y, 0.0f ) - 1.5f, 1.0f );
	}
	else if ( s->phase == 1 )
	{
		Look( m, s, 9.0f, -1.0f, 1.4f );
	}
	else
	{
		// Pull back to all of it
		b2Vec2 extents = b2AABB_Extents( m->bounds );
		m->focus = b2AABB_Center( m->bounds );
		m->focusHalfHeight = 1.05f * b2MaxFloat( extents.y, extents.x * 9.0f / 16.0f );
	}
}

// ---------------------------------------------------------------------------------------------------------------------

typedef void CreateFcn( Machine* m, MachineSection* s );
typedef void StepFcn( Machine* m, MachineSection* s, bool active, float timeStep );

static CreateFcn* const createFcns[section_typeCount] = { CreateStart, CreateSluice, CreateDominoes, CreateFerry, CreateTurn,
														  CreateCradle, CreateJelly, CreateHoney, CreateMercury, CreateFinale };
static StepFcn* const stepFcns[section_typeCount] = { StepStart, StepSluice, StepDominoes, StepFerry, StepTurn,
													  StepCradle, StepJelly, StepHoney, StepMercury, StepFinale };

static MachineSection* AddSection( Machine* m, MachineSectionType type, int count )
{
	MachineSection* s = m->sections + m->sectionCount;
	MachineSection* before = m->sectionCount > 0 ? s - 1 : NULL;
	m->sectionCount += 1;
	*s = (MachineSection){ 0 };
	s->type = type;
	s->count = count;
	s->origin = before != NULL ? before->exit : b2Vec2_zero;
	s->direction = before != NULL ? before->direction : 1.0f;
	if ( before != NULL && before->type == section_turn )
	{
		s->direction = -s->direction;
	}
	s->marble = b2_nullBodyId;
	createFcns[type]( m, s );
	return s;
}

void CreateMachine( Machine* machine, b2WorldId worldId, const MachineDef* def )
{
	Machine* m = machine;
	*m = (Machine){ 0 };
	m->def = *def;
	m->worldId = worldId;
	m->bounds = (b2AABB){ { 1e9f, 1e9f }, { -1e9f, -1e9f } };
	m->baton = b2_nullBodyId;

	b2BodyDef groundDef = b2DefaultBodyDef();
	m->ground = b2CreateBody( worldId, &groundDef );

	m->fluid = b2CreateFluid( worldId, &def->fluid );
	for ( int i = 0; i < machine_liquidCount; ++i )
	{
		m->liquids[i] = b2CreateLiquid( m->fluid, def->liquids + i );
	}

	AddSection( m, section_start, 0 );
	if ( def->only < section_typeCount )
	{
		// One section, and a tray to catch what it sends on
		MachineSection* s = AddSection( m, def->only, 4 );
		MachineSection tray = { .origin = s->exit, .direction = s->type == section_turn ? -s->direction : s->direction };
		Floor( m, &tray, 0.0f, 0.0f, 4.0f, -0.2f );
		Wall( m, &tray, 4.0f, -0.5f, 4.4f, 1.0f );
	}
	else
	{
		AddSection( m, section_sluice, 0 );
		AddSection( m, section_dominoes, 0 );
		AddSection( m, section_ferry, 0 );
		AddSection( m, section_turn, 4 );
		AddSection( m, section_cradle, 0 );
		AddSection( m, section_jelly, 0 );
		AddSection( m, section_honey, 0 );
		AddSection( m, section_turn, 4 );
		AddSection( m, section_mercury, 0 );
		AddSection( m, section_finale, 0 );
	}

	StepMachine( m, 0.0f );
	m->viewCenter = m->focus;
	m->viewHalfHeight = m->focusHalfHeight;
}

void StepMachine( Machine* machine, float timeStep )
{
	Machine* m = machine;
	m->time += timeStep;

	// The marble has reached the next section once it is over the start of its floor
	if ( m->active + 1 < m->sectionCount )
	{
		b2Vec2 p = BodyLocal( m->sections + m->active + 1, m->baton );
		if ( Inside( p, -0.3f, -1.5f, 4.0f, 3.0f ) )
		{
			m->active += 1;
		}
	}

	for ( int i = 0; i < m->sectionCount; ++i )
	{
		MachineSection* s = m->sections + i;
		s->clock += timeStep;
		stepFcns[s->type]( m, s, i == m->active, timeStep );
	}

	// A lone section is done a while after its marble is past its end
	MachineSection* last = m->sections + m->sectionCount - 1;
	if ( last->type != section_finale )
	{
		float direction = last->type == section_turn ? -last->direction : last->direction;
		float past = direction * ( b2ToVec2( b2Body_GetPosition( m->baton ) ).x - last->exit.x );
		m->finishedFor = past > 0.0f && m->active == m->sectionCount - 1 ? m->finishedFor + timeStep : 0.0f;
		m->finished = m->finishedFor > 4.0f;
	}
	else if ( m->finished )
	{
		m->finishedFor += timeStep;
	}

	// The view is a mass on a spring pulled toward where the active section says to look, damped so that it just
	// does not overshoot. It keeps its speed when the target jumps, so it swings over rather than starting at once.
	float omega = 2.0f * B2_PI * m->def.viewHertz;
	b2Vec2 pull = b2MulSV( omega * omega, b2Sub( m->focus, m->viewCenter ) );
	m->viewVelocity = b2MulSV( 1.0f / ( 1.0f + 2.0f * omega * timeStep ), b2MulAdd( m->viewVelocity, timeStep, pull ) );
	m->viewCenter = b2MulAdd( m->viewCenter, timeStep, m->viewVelocity );

	// In and out more slowly
	float zoomOmega = 0.6f * omega;
	float zoomPull = zoomOmega * zoomOmega * ( m->focusHalfHeight - m->viewHalfHeight );
	m->viewZoomSpeed = ( m->viewZoomSpeed + timeStep * zoomPull ) / ( 1.0f + 2.0f * zoomOmega * timeStep );
	m->viewHalfHeight += timeStep * m->viewZoomSpeed;
}
