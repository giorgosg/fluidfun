// SPDX-License-Identifier: MIT
#include "machine.h"

#include "box2d/math_functions.h"

#include <math.h>
#include <stddef.h>

// Static planks and walls are this thick, so that fast liquid does not cross them in a substep
#define MACHINE_THICKNESS 0.3f
// A socket is a pit a marble drops into and stays in, with a sensor in it, which is how a marble switches something on
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
	// Water with no pull of its own or toward walls, and thinner, so that it falls through the sluice's grate instead of
	// creeping along it to the dominoes
	def.liquids[machine_water].cohesion = 0.0f;
	def.liquids[machine_water].adhesion = 0.0f;
	def.liquids[machine_water].viscosity = 0.2f;
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
	static const char* names[] = { "Start",			 "Sluice And Wheel", "Dominoes",  "Ferry",		  "Jelly Ball", "Cradle",
								   "Dough And Jelly", "Honey Balance",	   "Peg Board", "Mercury Lift", "Finale" };
	return type < section_typeCount ? names[type] : "Whole";
}

// Sections that take the marble down to the row below, which runs the other way
static bool Reverses( MachineSectionType type )
{
	return type == section_jellyBall || type == section_pegs;
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

static void Peg( Machine* m, const MachineSection* s, float x, float y, float radius )
{
	b2Circle circle = { At( s, x, y ), radius };
	b2ShapeDef def = b2DefaultShapeDef();
	def.material.restitution = 0.3f;
	b2CreateCircleShape( m->ground, &def, &circle );
}

// A sensor box between two corners, fixed in the world. It sees marbles, whose shapes have sensor events on, and
// the fluid counts the liquid in it.
static b2ShapeId Sensor( Machine* m, const MachineSection* s, float x0, float y0, float x1, float y1 )
{
	b2Vec2 a = At( s, x0, y0 ), b = At( s, x1, y1 );
	b2Polygon box = b2MakeOffsetBox( 0.5f * b2AbsFloat( b.x - a.x ), 0.5f * b2AbsFloat( b.y - a.y ), b2Lerp( a, b, 0.5f ), b2Rot_identity );
	b2ShapeDef def = b2DefaultShapeDef();
	def.isSensor = true;
	def.enableSensorEvents = true;
	return b2CreatePolygonShape( m->ground, &def, &box );
}

// A body the sensor held at the end of the last step, or null. Anything moved by forces counts, whatever put it there:
// the marble the machine sent, or one dragged in with the mouse.
static b2BodyId Visitor( b2ShapeId sensor )
{
	b2ShapeId visitors[16];
	int count = b2Shape_GetSensorData( sensor, visitors, 16 );
	for ( int i = 0; i < count; ++i )
	{
		if ( b2Shape_IsValid( visitors[i] ) )
		{
			return b2Shape_GetBody( visitors[i] );
		}
	}
	return b2_nullBodyId;
}

static bool Visited( b2ShapeId sensor )
{
	return B2_IS_NON_NULL( Visitor( sensor ) );
}

// How much liquid a sensor held after the last step, as area at rest
static float LiquidIn( const Machine* m, b2ShapeId sensor )
{
	return b2Fluid_GetSensorArea( m->fluid, sensor );
}

// A sloping floor of bars with gaps a marble rolls over and liquid falls through, solid for its last stretch
static void Bars( Machine* m, const MachineSection* s, float x0, float y0, float x1, float y1 )
{
	float slope = ( y1 - y0 ) / ( x1 - x0 );
	float x = x0;
	while ( x1 - x > 1.2f )
	{
		Floor( m, s, x, y0 + slope * ( x - x0 ), x + 0.25f, y0 + slope * ( x + 0.25f - x0 ) );
		x += 0.6f;
	}
	Floor( m, s, x, y0 + slope * ( x - x0 ), x1, y1 );
}

// The floor a marble rolls in on, ending in a socket whose near edge is at x, and the sensor in the socket. The caller
// builds the socket's far wall.
static b2ShapeId Socket( Machine* m, const MachineSection* s, float x )
{
	Floor( m, s, 0.0f, 0.0f, x, 0.0f );
	Wall( m, s, x - 0.3f, -MACHINE_SOCKET_DEPTH - 0.3f, x + MACHINE_SOCKET_WIDTH, -MACHINE_SOCKET_DEPTH );
	Wall( m, s, x - 0.3f, -MACHINE_SOCKET_DEPTH, x, -0.3f );
	return Sensor( m, s, x + 0.1f, -MACHINE_SOCKET_DEPTH, x + MACHINE_SOCKET_WIDTH - 0.1f, -0.2f );
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
static b2ShapeId BoxOn( b2BodyId body, const MachineSection* s, float x, float y, float halfWidth, float halfHeight, float angle,
						float density )
{
	b2Polygon box = b2MakeOffsetBox( halfWidth, halfHeight, (b2Vec2){ s->direction * x, y }, b2MakeRot( s->direction * angle ) );
	b2ShapeDef def = b2DefaultShapeDef();
	def.density = density;
	def.enableSensorEvents = b2Body_GetType( body ) == b2_dynamicBody;
	return b2CreatePolygonShape( body, &def, &box );
}

// A sensor box that rides a body
static b2ShapeId SensorOn( b2BodyId body, const MachineSection* s, float x, float y, float halfWidth, float halfHeight )
{
	b2Polygon box = b2MakeOffsetBox( halfWidth, halfHeight, (b2Vec2){ s->direction * x, y }, b2Rot_identity );
	b2ShapeDef def = b2DefaultShapeDef();
	def.density = 0.0f;
	def.isSensor = true;
	def.enableSensorEvents = true;
	return b2CreatePolygonShape( body, &def, &box );
}

static b2BodyId Ball( Machine* m, const MachineSection* s, float x, float y, float radius, float density, float restitution )
{
	b2BodyId body = Body( m, s, b2_dynamicBody, x, y, 0.0f );
	b2Circle circle = { { 0.0f, 0.0f }, radius };
	b2ShapeDef def = b2DefaultShapeDef();
	def.density = density;
	def.material.restitution = restitution;
	def.enableSensorEvents = true;
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

// Swings a kinematic trapdoor down about its hinge, which is the body's origin, until it hangs straight down. Spin is
// +1 for a door that opens clockwise when the section goes right.
static void SwingOpen( const MachineSection* s, b2BodyId door, float spin )
{
	bool open = b2AbsFloat( b2Rot_GetAngle( b2Body_GetRotation( door ) ) ) > 0.5f * B2_PI;
	b2Body_SetAngularVelocity( door, open ? 0.0f : -spin * s->direction * 4.0f );
}

// A section that moves on to its next step is where the action is, and the view goes there
static void Advance( Machine* m, MachineSection* s )
{
	s->phase += 1;
	s->clock = 0.0f;
	m->active = (int)( s - m->sections );
}

// What the view follows is whatever set off the last sensor
static void PassBaton( Machine* m, MachineSection* s, b2BodyId body )
{
	m->baton = body;
	Advance( m, s );
}

// Where to look, in section coordinates. Scale is in units of the view the machine was asked for.
static void Look( Machine* m, const MachineSection* s, float x, float y, float scale )
{
	m->focus = At( s, x, y );
	m->focusHalfHeight = scale * m->def.viewHalfHeight;
}

// Looks a little ahead of a point, and a little below it, where what comes next is waiting
static void FollowPoint( Machine* m, const MachineSection* s, b2Vec2 p, float scale )
{
	Look( m, s, p.x + 0.35f * m->def.viewHalfHeight, p.y - 0.15f * m->def.viewHalfHeight, scale );
}

static void Follow( Machine* m, const MachineSection* s, b2BodyId body, float scale )
{
	FollowPoint( m, s, BodyLocal( s, body ), scale );
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
		Advance( m, s );
	}
	else if ( s->phase == 1 && SlideTo( s, s->parts[0], -7.9f, 3.6f, 2.0f, timeStep ) )
	{
		Advance( m, s );
	}

	if ( active )
	{
		Follow( m, s, m->baton, 1.0f );
	}
}

// ---------------------------------------------------------------------------------------------------------------------
// Sluice and wheel: the marble drops into a socket, which lifts the gate of a reservoir. The water runs down a spout,
// off a baffle and over a wheel into a sump, where a marble waits in a chimney. The sump holds less than the
// reservoir, so the marble floats up and out over its low wall. The water drains through the bars of the way out.

static void CreateSluice( Machine* m, MachineSection* s )
{
	float spacing = m->def.fluid.particleSpacing;
	s->sensors[0] = Socket( m, s, 2.2f );

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

	// Sump, with a sensor on its floor that tells the view to turn from the wheel to the marble once water is there
	Wall( m, s, 9.0f, -4.9f, 15.4f, -4.6f );
	Wall( m, s, 9.0f, -4.6f, 9.4f, 2.5f );
	Wall( m, s, 15.0f, -6.3f, 15.4f, -2.9f );
	Wall( m, s, 15.0f, -1.6f, 15.4f, 5.0f );
	s->sensors[1] = Sensor( m, s, 9.4f, -4.6f, 13.9f, -3.8f );

	// The marble waits in a chimney beside the low wall, open at the foot by less than a marble. All the water that
	// leaves the sump comes up through it, so it carries the marble out. In an open sump the marble drifted under
	// the wheel and stayed there. The chimney has a roof, so the wheel's spray cannot fly out over the low wall: it
	// carried water on to the dominoes before the sump had filled.
	Wall( m, s, 13.95f, -4.2f, 14.15f, -1.0f );
	Wall( m, s, 13.95f, -1.0f, 15.0f, -0.7f );
	s->marble = Marble( m, s, 14.57f, -4.6f + m->def.marbleRadius );
	s->sensors[2] = Sensor( m, s, 15.45f, -3.9f, 17.0f, -1.6f );

	// The way out, a grate over a tank that catches what the sump spills. The grate starts below the sump's lip, so
	// that the water falls on it and through it, and steep, so that the marble, which no longer rides the flow,
	// reaches the dominoes fast enough to tip the first: level with the lip, the flow shot along the grate over its gaps and
	// on to the dominoes. The sump keeps about half the reservoir, and the tank holds the rest.
	Bars( m, s, 15.4f, -3.5f, 19.5f, -4.9f );
	Wall( m, s, 15.0f, -9.3f, 15.4f, -6.3f );
	Wall( m, s, 15.0f, -9.3f, 19.5f, -9.0f );
	Wall( m, s, 19.1f, -9.0f, 19.5f, -5.3f );
	// A curtain over the end of the grate, for spray
	Wall( m, s, 19.2f, -3.9f, 19.5f, 0.5f );
	s->exit = At( s, 19.5f, -4.9f );
}

static void StepSluice( Machine* m, MachineSection* s, bool active, float timeStep )
{
	if ( s->phase == 0 && Visited( s->sensors[0] ) )
	{
		Advance( m, s );
	}
	else if ( s->phase == 1 && SlideTo( s, s->parts[0], 9.2f, 4.5f, 1.0f, timeStep ) )
	{
		Advance( m, s );
	}
	else if ( s->phase == 2 && LiquidIn( m, s->sensors[1] ) > 1.0f )
	{
		Advance( m, s );
	}
	else if ( s->phase == 3 && Visited( s->sensors[2] ) )
	{
		PassBaton( m, s, Visitor( s->sensors[2] ) );
	}

	if ( active == false )
	{
		return;
	}

	if ( s->phase == 0 || s->phase == 4 )
	{
		Follow( m, s, m->baton, 1.0f );
	}
	else if ( s->phase < 3 )
	{
		Look( m, s, 11.0f, 0.3f, 1.15f );
	}
	else
	{
		b2Vec2 p = BodyLocal( s, s->marble );
		Look( m, s, p.x + 0.5f, p.y + 1.0f, 0.85f );
	}
}

// ---------------------------------------------------------------------------------------------------------------------
// Dominoes, each bigger than the one before, and a marble on a post beside the last

#define MACHINE_DOMINOES 8

static void CreateDominoes( Machine* m, MachineSection* s )
{
	// A drop before the dominoes, so that the marble reaches the first fast enough to tip it however slowly it came in,
	// and at its foot a gap the marble rolls over and stray water falls through, into a tub. Water that got to the
	// dominoes knocked the first one on to the second before the marble came, too gently to topple it, and the marble
	// stopped against them.
	float base = -0.5f;
	Floor( m, s, 0.0f, 0.0f, 1.4f, base );
	Floor( m, s, 1.65f, base, 7.5f, base );
	Wall( m, s, 1.0f, base - 1.9f, 2.1f, base - 1.7f );
	Wall( m, s, 1.0f, base - 1.7f, 1.15f, base - 0.9f );
	Wall( m, s, 1.95f, base - 1.7f, 2.1f, base - 0.9f );
	Floor( m, s, 7.5f, base, 13.5f, base - 0.6f );
	float x = 2.0f, height = 0.6f;
	for ( int i = 0; i < MACHINE_DOMINOES; ++i )
	{
		b2BodyId domino = Body( m, s, b2_dynamicBody, x, base + 0.5f * height, 0.0f );
		BoxOn( domino, s, 0.0f, 0.0f, height / 12.0f, 0.5f * height, 0.0f, 0.4f );
		if ( i == MACHINE_DOMINOES - 1 )
		{
			// The last strikes the marble sideways early in its fall. On the floor the domino came down on top of the
			// marble and pinned it.
			Wall( m, s, x + 0.55f, base, x + 0.95f, base + 0.75f * height );
			s->marble = Marble( m, s, x + 0.72f, base + 0.75f * height + m->def.marbleRadius );
			// Beyond where the last domino can fall, since anything entering a sensor sets it off
			s->sensors[0] = Sensor( m, s, x + height + 0.4f, base - 0.6f, x + height + 3.0f, base + 0.75f * height );
		}
		x += 0.55f * height;
		height *= 1.22f;
	}
	s->exit = At( s, 13.5f, base - 0.6f );
}

static void StepDominoes( Machine* m, MachineSection* s, bool active, float timeStep )
{
	(void)timeStep;
	if ( s->phase == 0 && Visited( s->sensors[0] ) )
	{
		PassBaton( m, s, Visitor( s->sensors[0] ) );
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
// Ferry: the marble falls through a hole in the dock into a boat. A sensor on the deck starts the paddle wheel, the
// boat crosses the lake, and its bowsprit pushes a marble off the far shore.

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

	// Boat: a floor, two sides and a bowsprit low enough to meet a marble on the shore square on. The boat floats
	// with its origin near -2.2 with a marble aboard, and its keel stays below the shore, so it stops against the
	// shore wall with the bowsprit over the shore.
	b2BodyId hull = Body( m, s, b2_dynamicBody, 4.4f, -2.3f, 0.0f );
	BoxOn( hull, s, 0.0f, -0.25f, 1.7f, 0.15f, 0.0f, 0.3f );
	BoxOn( hull, s, -1.6f, 0.25f, 0.1f, 0.4f, 0.0f, 0.3f );
	BoxOn( hull, s, 1.6f, 0.25f, 0.1f, 0.4f, 0.0f, 0.3f );
	BoxOn( hull, s, 2.6f, 0.3f, 0.9f, 0.06f, 0.0f, 0.1f );
	s->sensors[0] = SensorOn( hull, s, 0.0f, 0.25f, 1.45f, 0.35f );
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

	// Far shore, with the marble the bowsprit pushes on to the slope
	Floor( m, s, 20.4f, -2.2f, 21.8f, -2.2f );
	Floor( m, s, 21.8f, -2.2f, 24.0f, -2.8f );
	s->marble = Marble( m, s, 21.0f, -2.2f + m->def.marbleRadius );
	s->sensors[1] = Sensor( m, s, 22.4f, -3.0f, 24.0f, -1.6f );
	s->exit = At( s, 24.0f, -2.8f );
}

static void StepFerry( Machine* m, MachineSection* s, bool active, float timeStep )
{
	(void)timeStep;
	if ( s->phase == 0 && Visited( s->sensors[0] ) )
	{
		// Clockwise for a boat going right, so that the blades push the water back
		b2RevoluteJoint_SetTargetMotorSpeed( s->joint, -s->direction * MACHINE_FERRY_SPEED );
		b2Body_SetAwake( s->parts[0], true );
		Advance( m, s );
	}
	else if ( s->phase == 1 && Visited( s->sensors[1] ) )
	{
		b2RevoluteJoint_SetTargetMotorSpeed( s->joint, 0.0f );
		PassBaton( m, s, Visitor( s->sensors[1] ) );
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
// Jelly ball: the marble's socket lifts a gate in front of a ball of jelly, which rolls and bounces down four ramps to
// the row below. A fluid sensor at the foot sees it arrive and drops the gate in front of the next marble, which rolls
// on under a bar the ball is too big to pass.

#define MACHINE_JELLY_RADIUS 0.55f

static void CreateJellyBall( Machine* m, MachineSection* s )
{
	s->sensors[0] = Socket( m, s, 2.0f );
	Wall( m, s, 2.9f, -0.95f, 3.2f, 0.6f );

	// Ramps, each falling 1.6, with gaps at their ends wider than the ball
	Floor( m, s, 3.2f, -0.3f, 9.6f, -1.9f );
	Floor( m, s, 10.9f, -3.4f, 2.2f, -5.0f );
	Floor( m, s, 0.8f, -6.8f, 9.6f, -8.4f );
	Floor( m, s, 10.9f, -9.9f, 0.0f, -11.5f );
	Wall( m, s, 10.9f, -10.2f, 11.3f, 0.6f );
	Wall( m, s, 0.4f, -6.8f, 0.8f, -3.8f );

	// The ball, held by a gate
	float slope = 1.6f / 6.4f;
	float x = 4.2f, y = -0.3f - slope * ( x - 3.2f ) + MACHINE_JELLY_RADIUS + 0.1f;
	b2Liquid_SpawnCircle( m->liquids[machine_jelly], b2ToPos( At( s, x, y ) ), MACHINE_JELLY_RADIUS, b2Vec2_zero );
	s->parts[0] = Body( m, s, b2_kinematicBody, x + MACHINE_JELLY_RADIUS + 0.25f, y - 0.1f, 0.0f );
	BoxOn( s->parts[0], s, 0.0f, 0.0f, 0.1f, 0.6f, 0.0f, 0.0f );

	// The next marble on the last ramp, behind a gate on its downhill side, and the bar
	float last = 1.6f / 10.9f;
	float mx = 3.4f, my = -9.9f - last * ( 10.9f - mx );
	s->marble = Marble( m, s, mx, my + m->def.marbleRadius + 0.02f );
	s->parts[1] = Body( m, s, b2_kinematicBody, mx - m->def.marbleRadius - 0.12f, my + 0.4f, 0.0f );
	BoxOn( s->parts[1], s, 0.0f, 0.0f, 0.08f, 0.4f, 0.0f, 0.0f );
	s->sensors[1] = Sensor( m, s, 4.2f, my - 0.2f, 7.5f, my + 1.6f );
	float barY = -9.9f - last * ( 10.9f - 1.4f ) + 0.75f;
	Wall( m, s, 1.2f, barY, 1.6f, barY + 0.25f );
	s->exit = At( s, 0.0f, -11.5f );
}

// The middle of the jelly in this section, or false if there is none
static bool JellyCenter( const Machine* m, const MachineSection* s, b2Vec2* center )
{
	b2FluidParticles p = b2Fluid_GetParticles( m->fluid );
	int jelly = b2Liquid_GetIndex( m->liquids[machine_jelly] );
	b2Vec2 sum = b2Vec2_zero;
	int count = 0;
	for ( int i = 0; i < p.count; ++i )
	{
		if ( p.liquids[i] != jelly )
		{
			continue;
		}
		b2Vec2 q = Local( s, b2ToVec2( p.positions[i] ) );
		if ( q.x > -1.0f && q.x < 12.0f && q.y > -13.0f && q.y < 2.0f )
		{
			sum = b2Add( sum, q );
			count += 1;
		}
	}
	if ( count == 0 )
	{
		return false;
	}
	*center = b2MulSV( 1.0f / (float)count, sum );
	return true;
}

static void StepJellyBall( Machine* m, MachineSection* s, bool active, float timeStep )
{
	if ( s->phase == 0 && Visited( s->sensors[0] ) )
	{
		Advance( m, s );
	}
	else if ( s->phase == 1 )
	{
		b2Vec2 p = BodyLocal( s, s->parts[0] );
		if ( SlideTo( s, s->parts[0], p.x, 1.8f, 2.0f, timeStep ) )
		{
			Advance( m, s );
		}
	}
	else if ( s->phase == 2 && LiquidIn( m, s->sensors[1] ) > 0.15f )
	{
		PassBaton( m, s, s->marble );
	}
	else if ( s->phase == 3 )
	{
		b2Vec2 p = BodyLocal( s, s->parts[1] );
		SlideTo( s, s->parts[1], p.x, -12.5f, 2.0f, timeStep );
	}

	if ( active == false )
	{
		return;
	}

	b2Vec2 center;
	if ( ( s->phase == 1 || s->phase == 2 ) && JellyCenter( m, s, &center ) )
	{
		// Straight down the middle of the ramps, and below the ball, to show the ramp it is coming to
		Look( m, s, 0.5f * ( center.x + 5.6f ), center.y - 0.25f * m->def.viewHalfHeight, 1.0f );
	}
	else
	{
		Follow( m, s, m->baton, 1.0f );
	}
}

// ---------------------------------------------------------------------------------------------------------------------
// Cradle: the marble drops into a socket, whose sensor pulls the pin holding the first of five balls back. It swings
// down into the others, and the last swings out across a gap into the next marble. The balls are the marble's size
// and mass, and hang at its height, so that each blow is centre to centre and passes all of its motion on.

#define MACHINE_CRADLE_BALLS 5
#define MACHINE_CRADLE_ROD 3.0f
#define MACHINE_CRADLE_PULL 0.9f

static void CreateCradle( Machine* m, MachineSection* s )
{
	float r = m->def.marbleRadius, rod = MACHINE_CRADLE_ROD, x0 = 5.0f;
	s->sensors[0] = Socket( m, s, 1.5f );
	Wall( m, s, 2.4f, -0.95f, 2.7f, 0.6f );
	Floor( m, s, 2.7f, 0.0f, 9.6f, 0.0f );
	Floor( m, s, 9.6f, 0.0f, 12.5f, -0.5f );

	float pivotY = r + 0.03f + rod;
	Wall( m, s, x0 - 0.6f, pivotY, x0 + 2.0f * r * (float)( MACHINE_CRADLE_BALLS - 1 ) + 0.6f, pivotY + 0.25f );
	for ( int i = 0; i < MACHINE_CRADLE_BALLS; ++i )
	{
		// The body's origin is the pivot. The first starts pulled back.
		b2BodyId ball = Body( m, s, b2_dynamicBody, x0 + 2.0f * r * (float)i, pivotY, i == 0 ? -MACHINE_CRADLE_PULL : 0.0f );
		b2Circle circle = { { 0.0f, -rod }, r };
		b2ShapeDef def = b2DefaultShapeDef();
		def.density = m->def.marbleDensity;
		def.material.restitution = 1.0f;
		def.material.friction = 0.0f;
		b2CreateCircleShape( ball, &def, &circle );

		// The rod is only there to be seen
		b2Polygon box = b2MakeOffsetBox( 0.02f, 0.5f * rod, (b2Vec2){ 0.0f, -0.5f * rod }, b2Rot_identity );
		def.density = 0.001f;
		def.filter.maskBits = 0;
		b2CreatePolygonShape( ball, &def, &box );
		Hinge( m, s, ball, false, 0.0f, 0.0f, 0.0f );
		if ( i == 0 )
		{
			s->parts[1] = ball;
		}
	}

	// The pin, just ahead of the held ball on its swing, and later pulled straight out from the pivot
	float c = cosf( MACHINE_CRADLE_PULL ), sn = sinf( MACHINE_CRADLE_PULL );
	b2Vec2 held = { x0 - rod * sn, pivotY - rod * c };
	b2Vec2 pin = b2MulAdd( held, r + 0.07f, (b2Vec2){ c, -sn } );
	s->parts[0] = Body( m, s, b2_kinematicBody, pin.x, pin.y, 0.0f );
	BoxOn( s->parts[0], s, 0.0f, 0.0f, 0.06f, 0.06f, 0.0f, 0.0f );

	float end = x0 + 2.0f * r * (float)( MACHINE_CRADLE_BALLS - 1 ) + r;
	s->marble = Marble( m, s, end + 0.5f + r, r );
	s->sensors[1] = Sensor( m, s, end + 0.5f + 2.0f * r + 0.4f, -0.5f, 12.5f, 1.0f );
	s->exit = At( s, 12.5f, -0.5f );
}

static void StepCradle( Machine* m, MachineSection* s, bool active, float timeStep )
{
	if ( s->phase == 0 && Visited( s->sensors[0] ) )
	{
		b2Body_SetAwake( s->parts[1], true );
		Advance( m, s );
	}
	else if ( s->phase == 1 )
	{
		float c = cosf( MACHINE_CRADLE_PULL ), sn = sinf( MACHINE_CRADLE_PULL );
		b2Vec2 p = BodyLocal( s, s->parts[0] );
		SlideTo( s, s->parts[0], p.x - 0.1f * sn, p.y - 0.1f * c, 3.0f, timeStep );
		if ( s->clock > 0.3f )
		{
			b2Body_SetLinearVelocity( s->parts[0], b2Vec2_zero );
			Advance( m, s );
		}
	}
	else if ( s->phase == 2 && Visited( s->sensors[1] ) )
	{
		PassBaton( m, s, Visitor( s->sensors[1] ) );
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
		Look( m, s, 6.5f, 1.8f, 0.7f );
	}
}

// ---------------------------------------------------------------------------------------------------------------------
// Dough and jelly: the marble's socket opens a trapdoor under a block of dough, which falls on a blade just below and
// splits into two buckets. A fluid sensor in each bucket waits for its half; with both full, a trapdoor drops a block
// of jelly on the high end of a seesaw, whose low end throws a marble.

#define MACHINE_SEESAW_LOW -0.5f
#define MACHINE_SEESAW_HIGH -0.12f

static void CreateDough( Machine* m, MachineSection* s )
{
	float spacing = m->def.fluid.particleSpacing;
	s->sensors[0] = Socket( m, s, 2.0f );
	Wall( m, s, 2.9f, -3.3f, 3.2f, 0.6f );
	Wall( m, s, 2.9f, -3.3f, 11.15f, -3.0f );

	// Buckets, with a sensor in each, and the blade between them, its edge just under the trapdoor
	Wall( m, s, 5.6f, -3.0f, 5.8f, -1.6f );
	Wall( m, s, 4.36f, -3.0f, 4.44f, 2.5f );
	b2Vec2 edge[3] = { At( s, 4.4f, 2.85f ), At( s, 4.3f, 2.4f ), At( s, 4.5f, 2.4f ) };
	b2Hull edgeHull = b2ComputeHull( edge, 3 );
	b2Polygon wedge = b2MakePolygon( &edgeHull, 0.0f );
	b2ShapeDef wedgeDef = b2DefaultShapeDef();
	b2CreatePolygonShape( m->ground, &wedgeDef, &wedge );
	s->sensors[1] = Sensor( m, s, 3.2f, -3.0f, 4.36f, -1.6f );
	s->sensors[2] = Sensor( m, s, 4.44f, -3.0f, 5.6f, -1.6f );

	// The trapdoor, hinged at its near end, which is the body's origin, between low walls
	Wall( m, s, 3.1f, 2.9f, 3.3f, 4.2f );
	Wall( m, s, 5.5f, 2.9f, 5.7f, 4.2f );
	s->parts[0] = Body( m, s, b2_kinematicBody, 3.3f, 3.0f, 0.0f );
	BoxOn( s->parts[0], s, 1.1f, 0.0f, 1.1f, 0.08f, 0.0f, 0.0f );
	Spawn( m, s, machine_dough, 3.3f + spacing, 3.08f + spacing, 5.5f - spacing, 4.0f );

	// The jelly's trapdoor
	s->parts[3] = Body( m, s, b2_kinematicBody, 6.1f, 3.0f, 0.0f );
	BoxOn( s->parts[3], s, 1.1f, 0.0f, 1.1f, 0.08f, 0.0f, 0.0f );
	Spawn( m, s, machine_jelly, 6.4f, 3.08f + spacing, 8.0f, 4.7f );

	// The seesaw stops short of level, so that its low end still slopes down and outward at the top of its throw
	b2BodyId beam = Body( m, s, b2_dynamicBody, 8.6f, -1.5f, MACHINE_SEESAW_LOW );
	BoxOn( beam, s, 0.0f, 0.0f, 2.6f, 0.1f, 0.0f, 1.0f );
	Hinge( m, s, beam, true, MACHINE_SEESAW_LOW, MACHINE_SEESAW_HIGH, 0.0f );
	s->parts[4] = beam;

	// The marble rests on the low end against a wall the end rises above
	Wall( m, s, 10.95f, -3.0f, 11.15f, -1.8f );
	s->marble = Marble( m, s, 10.55f, -2.09f );
	s->sensors[3] = Sensor( m, s, 11.4f, -2.6f, 15.0f, 0.0f );

	// The way out, under a hood that turns a high throw back down
	Floor( m, s, 11.15f, -1.8f, 15.0f, -2.5f );
	Floor( m, s, 11.6f, 2.0f, 15.0f, -0.9f );
	s->exit = At( s, 15.0f, -2.5f );
}

static void StepDough( Machine* m, MachineSection* s, bool active, float timeStep )
{
	(void)timeStep;
	if ( s->phase == 0 && Visited( s->sensors[0] ) )
	{
		Advance( m, s );
	}
	else if ( s->phase == 1 )
	{
		SwingOpen( s, s->parts[0], 1.0f );
		if ( LiquidIn( m, s->sensors[1] ) > 0.3f && LiquidIn( m, s->sensors[2] ) > 0.3f )
		{
			Advance( m, s );
		}
	}
	else if ( s->phase == 2 )
	{
		SwingOpen( s, s->parts[3], 1.0f );
		if ( Visited( s->sensors[3] ) )
		{
			PassBaton( m, s, Visitor( s->sensors[3] ) );
		}
	}
	else if ( s->phase == 3 )
	{
		SwingOpen( s, s->parts[3], 1.0f );
	}

	if ( active == false )
	{
		return;
	}

	if ( s->phase == 0 || s->phase == 3 )
	{
		Follow( m, s, m->baton, 1.0f );
	}
	else if ( s->phase == 1 )
	{
		Look( m, s, 4.6f, 1.0f, 0.75f );
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
	s->sensors[0] = Socket( m, s, 2.0f );
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
	s->sensors[1] = Sensor( m, s, 10.4f, -2.2f, 12.0f, -0.6f );
	s->exit = At( s, 14.0f, -2.0f );
}

static void StepHoney( Machine* m, MachineSection* s, bool active, float timeStep )
{
	if ( s->phase == 0 && Visited( s->sensors[0] ) )
	{
		Advance( m, s );
	}
	else if ( s->phase == 1 && SlideTo( s, s->parts[0], 5.0f, 4.2f, 1.0f, timeStep ) )
	{
		Advance( m, s );
	}
	else if ( s->phase == 2 && Visited( s->sensors[1] ) )
	{
		PassBaton( m, s, Visitor( s->sensors[1] ) );
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
// Peg board: the marble falls through staggered pegs and two turning crosses into a ramp that takes it to the row
// below, which runs the other way

// A cross that turns on its own. A free one held the marble balanced in the V between two of its arms.
static void Spinner( Machine* m, const MachineSection* s, float x, float y, float speed )
{
	b2BodyId spinner = Body( m, s, b2_kinematicBody, x, y, 0.3f );
	BoxOn( spinner, s, 0.0f, 0.0f, 0.65f, 0.05f, 0.0f, 0.0f );
	BoxOn( spinner, s, 0.0f, 0.0f, 0.05f, 0.65f, 0.0f, 0.0f );
	b2Body_SetAngularVelocity( spinner, s->direction * speed );
}

static void CreatePegs( Machine* m, MachineSection* s )
{
	Floor( m, s, 0.0f, 0.0f, 1.4f, 0.0f );
	Wall( m, s, 1.3f, -8.3f, 1.6f, -0.3f );
	Wall( m, s, 7.6f, -9.0f, 7.9f, 1.5f );

	b2Vec2 spinners[2] = { { 3.4f, -3.6f }, { 5.7f, -5.6f } };
	for ( int row = 0; row < 9; ++row )
	{
		float y = -1.4f - 0.8f * (float)row;
		// Kept more than a marble clear of the walls: a peg closer than that wedged the marble against the wall
		for ( float x = 1.6f + ( row % 2 == 0 ? 0.9f : 1.35f ); x < 6.8f; x += 0.9f )
		{
			bool clear = b2Distance( (b2Vec2){ x, y }, spinners[0] ) > 0.85f && b2Distance( (b2Vec2){ x, y }, spinners[1] ) > 0.85f;
			if ( clear )
			{
				Peg( m, s, x, y, 0.1f );
			}
		}
	}
	Spinner( m, s, spinners[0].x, spinners[0].y, 1.2f );
	Spinner( m, s, spinners[1].x, spinners[1].y, -1.5f );

	// The ramp runs out under the near wall
	Floor( m, s, 7.6f, -8.6f, 0.0f, -9.6f );
	s->exit = At( s, 0.0f, -9.6f );
}

static void StepPegs( Machine* m, MachineSection* s, bool active, float timeStep )
{
	(void)timeStep;
	if ( active )
	{
		// Over the board, low enough to show where the marble comes out
		b2Vec2 p = BodyLocal( s, m->baton );
		Look( m, s, 0.5f * ( p.x + 4.6f ), b2MinFloat( p.y, 0.0f ) - 0.2f * m->def.viewHalfHeight, 1.0f );
	}
}

// ---------------------------------------------------------------------------------------------------------------------
// Mercury lift: the marble's socket lifts the gate of a flask of mercury, which runs into a shaft with a stone ball
// at the bottom. Stone floats high on mercury, so the ball rises with it and rolls out over the shaft's low wall. A
// steel ball floated too, but so low that it stayed behind the wall once the mercury stopped running over it.

static void CreateMercury( Machine* m, MachineSection* s )
{
	float spacing = m->def.fluid.particleSpacing;
	s->sensors[0] = Socket( m, s, 2.0f );

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
	s->sensors[1] = Sensor( m, s, 9.65f, -1.8f, 11.0f, 0.0f );

	Bars( m, s, 9.6f, -1.2f, 13.5f, -2.0f );
	Wall( m, s, 9.6f, -3.8f, 13.5f, -3.5f );
	Wall( m, s, 13.1f, -3.5f, 13.5f, -2.4f );
	s->exit = At( s, 13.5f, -2.0f );
}

static void StepMercury( Machine* m, MachineSection* s, bool active, float timeStep )
{
	// The gate opens only a little: the ball rises slowly through the mercury, and the shaft has to be still running
	// over when it gets to the top, for the flow to carry it out
	if ( s->phase == 0 && Visited( s->sensors[0] ) )
	{
		Advance( m, s );
	}
	else if ( s->phase == 1 && SlideTo( s, s->parts[0], 6.6f, 1.8f, 1.0f, timeStep ) )
	{
		Advance( m, s );
	}
	else if ( s->phase == 2 && Visited( s->sensors[1] ) )
	{
		PassBaton( m, s, Visitor( s->sensors[1] ) );
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
// Finale: a pool of oil over water with crates afloat between them. The stone ball sinks, rolls down the pool's
// sloping floor to the sensor at its foot, and the pool goes up.

static void CreateFinale( Machine* m, MachineSection* s )
{
	float spacing = m->def.fluid.particleSpacing;
	Floor( m, s, 0.0f, 0.0f, 3.0f, 0.0f );
	Wall( m, s, 2.6f, -6.0f, 3.0f, -0.3f );
	Wall( m, s, 2.6f, -6.3f, 15.4f, -6.0f );
	Wall( m, s, 15.0f, -6.0f, 15.4f, 1.0f );
	// The floor slopes at 30 degrees to the sensor at its foot. On a gentler slope the ball rolled very slowly under
	// the liquid: 1.4 m in 80 s. The sensor is below where the crates plunge as they fall in, since anything in it sets
	// it off.
	float top = -2.8f, slope = 3.2f / 5.6f;
	Floor( m, s, 3.0f, top, 8.6f, -6.0f );
	Floor( m, s, 9.4f, -6.0f, 15.0f, top );
	s->sensors[0] = Sensor( m, s, 7.9f, -6.0f, 10.1f, -5.3f );

	// Water down to the sloping floor, a column at a time, and oil over it
	for ( float x = 3.0f; x < 15.0f - 0.01f; x += 0.4f )
	{
		float far = b2MaxFloat( b2AbsFloat( x - 9.0f ), b2AbsFloat( x + 0.4f - 9.0f ) ) - 0.4f;
		float floorY = b2MinFloat( top, -6.0f + slope * b2MaxFloat( far, 0.0f ) );
		float x0 = b2MaxFloat( x, 3.0f + spacing ), x1 = b2MinFloat( x + 0.4f, 15.0f - spacing ) - 0.5f * spacing;
		if ( floorY + spacing < -2.0f )
		{
			Spawn( m, s, machine_water, x0, floorY + spacing, x1, -2.0f );
		}
	}
	Spawn( m, s, machine_oil, 3.0f + spacing, -2.0f + spacing, 15.0f - spacing, -1.2f );

	static const float densities[] = { 0.8f, 0.3f, 0.8f, 0.45f, 0.8f };
	for ( int i = 0; i < 5; ++i )
	{
		b2BodyId crate = Body( m, s, b2_dynamicBody, 6.0f + 1.8f * (float)i, -0.7f, 0.0f );
		// Crates do not set off the sensor. One of 0.8, lighter than water, sank through the water to it in 85 s with no
		// ball sent, while the others of 0.8 float where oil meets water.
		b2ShapeId shape = BoxOn( crate, s, 0.0f, 0.0f, 0.4f, 0.4f, 0.0f, densities[i] );
		b2Shape_EnableSensorEvents( shape, false );
	}
	s->exit = At( s, 15.4f, 0.0f );
}

static void StepFinale( Machine* m, MachineSection* s, bool active, float timeStep )
{
	(void)timeStep;
	b2Vec2 p = BodyLocal( s, m->baton );
	if ( s->phase == 0 && Visited( s->sensors[0] ) )
	{
		b2ExplosionDef explosion = b2DefaultExplosionDef();
		explosion.position = b2Body_GetPosition( Visitor( s->sensors[0] ) );
		explosion.radius = 2.5f;
		explosion.falloff = 3.0f;
		explosion.impulsePerLength = 60.0f;
		b2World_Explode( m->worldId, &explosion );
		Advance( m, s );
	}
	else if ( s->phase == 1 && s->clock > 3.5f )
	{
		Advance( m, s );
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

static CreateFcn* const createFcns[section_typeCount] = { CreateStart,	   CreateSluice, CreateDominoes, CreateFerry,	CreateJellyBall, CreateCradle,
														  CreateDough,	   CreateHoney,	 CreatePegs,	 CreateMercury, CreateFinale };
static StepFcn* const stepFcns[section_typeCount] = { StepStart,	 StepSluice, StepDominoes, StepFerry,	StepJellyBall, StepCradle,
													  StepDough,	 StepHoney,	 StepPegs,	   StepMercury, StepFinale };

static MachineSection* AddSection( Machine* m, MachineSectionType type )
{
	MachineSection* s = m->sections + m->sectionCount;
	MachineSection* before = m->sectionCount > 0 ? s - 1 : NULL;
	m->sectionCount += 1;
	*s = (MachineSection){ 0 };
	s->type = type;
	s->origin = before != NULL ? before->exit : b2Vec2_zero;
	s->direction = before != NULL ? before->direction : 1.0f;
	if ( before != NULL && Reverses( before->type ) )
	{
		s->direction = -s->direction;
	}
	s->marble = b2_nullBodyId;
	createFcns[type]( m, s );

	// The marble arriving here makes this the active section
	if ( before != NULL )
	{
		s->entry = Sensor( m, s, 0.0f, -0.8f, 1.2f, 1.5f );
	}
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

	AddSection( m, section_start );
	if ( def->only < section_typeCount )
	{
		// One section, and a tray to catch what it sends on
		MachineSection* s = AddSection( m, def->only );
		MachineSection tray = { .origin = s->exit, .direction = Reverses( s->type ) ? -s->direction : s->direction };
		Floor( m, &tray, 0.0f, 0.0f, 4.0f, -0.2f );
		Wall( m, &tray, 4.0f, -0.5f, 4.4f, 1.0f );
	}
	else
	{
		AddSection( m, section_sluice );
		AddSection( m, section_dominoes );
		AddSection( m, section_ferry );
		AddSection( m, section_jellyBall );
		AddSection( m, section_cradle );
		AddSection( m, section_jelly );
		AddSection( m, section_honey );
		AddSection( m, section_pegs );
		AddSection( m, section_mercury );
		AddSection( m, section_finale );
	}

	StepMachine( m, 0.0f );
	m->viewCenter = m->focus;
	m->viewHalfHeight = m->focusHalfHeight;
}

void StepMachine( Machine* machine, float timeStep )
{
	Machine* m = machine;
	m->time += timeStep;

	// Something arriving at the start of a later section makes it the active one, and the view follows it. Only later:
	// a marble rolling back into the start of the section it left took the view back with it.
	b2SensorEvents events = b2World_GetSensorEvents( m->worldId );
	for ( int i = 0; i < events.beginCount; ++i )
	{
		for ( int j = m->active + 1; j < m->sectionCount; ++j )
		{
			if ( B2_ID_EQUALS( events.beginEvents[i].sensorShapeId, m->sections[j].entry ) && b2Shape_IsValid( events.beginEvents[i].visitorShapeId ) )
			{
				m->active = j;
				m->baton = b2Shape_GetBody( events.beginEvents[i].visitorShapeId );
			}
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
		float direction = Reverses( last->type ) ? -last->direction : last->direction;
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
