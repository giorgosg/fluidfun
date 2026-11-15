// SPDX-License-Identifier: MIT
// Adding particles: spawning in shapes, and emitters.
#include "fluid_internal.h"

#include "core.h"
#include "physics_world.h"
#include "recording.h"

// Liquid spawns 1.09 of the solver's spacings apart, which is 0.97 of particleSpacing. A settled pool is that much
// tighter than liquid at rest on average, from its own weight: spawned at particleSpacing, 10 and 25 m pools sank by
// 3 and 5%, and spawned here they rise by 3 and 1%. Much tighter than this holds pressure, which a free blob
// releases all at once, tearing open a void in its middle.
static float SpawnSpacing( const b2fFluid* f )
{
	return B2F_SPAWN_SPACING * f->spacing;
}

static bool AddParticle( b2fFluid* f, int liquid, b2fCoord x, b2fCoord y, float size, b2Vec2 velocity )
{
	if ( f->count == f->capacity && b2fGrow( f, f->count + 1 ) == false )
	{
		return false;
	}
	int i = f->count++;
	b2fClearParticle( f, i );
	f->pos[i].x = x;
	f->pos[i].y = y;
	f->vx[i] = velocity.x;
	f->vy[i] = velocity.y;
	f->id[i] = f->nextParticleId++;
	f->size[i] = size;
	f->liquid[i] = (unsigned char)liquid;
	return true;
}

// With adaptive sizes, the particles a spawn adds from here on take their sizes at the start of the next step, before
// they move. Ids only grow, so every particle of this liquid from the next id on is new.
static void CoarsenFromHere( b2fFluid* f, int liquid )
{
	if ( f->liquids[liquid].def.tuning.adaptiveRatio <= 1.0f )
	{
		return;
	}
	uint32_t bit = 1u << liquid;
	if ( ( f->coarsenLiquids & bit ) == 0 )
	{
		f->coarsenFrom[liquid] = f->nextParticleId;
	}
	f->coarsenLiquids |= bit;
}

static bool SpawnCalled( b2LiquidId liquidId )
{
	b2World* world = b2GetWorld( liquidId.world0 );
	B2_ASSERT( world->locked == false );
	return world->locked == false;
}

// A lattice at the finest size over the shape's bounds, centred on a point near the shape, keeping the points inside
// it, edges included. The test takes points relative to that centre, in float. Every shape spawns this way. With
// adaptive sizes the new particles take their sizes, and settle into place, at the start of the next step.
typedef struct b2fSpawnShape
{
	b2ShapeType type;
	b2Circle circle;
	b2Capsule capsule;
	b2Polygon polygon;
	b2Rot rotation;
	b2Pos center;
	// Half the size of the bounds, about the centre
	float halfWidth, halfHeight;
} b2fSpawnShape;

static bool InsideSpawnShape( const b2fSpawnShape* s, b2Vec2 p )
{
	switch ( s->type )
	{
		case b2_circleShape:
			return b2PointInCircle( &s->circle, p );
		case b2_capsuleShape:
			return b2PointInCapsule( &s->capsule, p );
		default:
			return b2PointInPolygon( &s->polygon, b2InvRotateVector( s->rotation, p ) );
	}
}

// A spawn is recorded with how many particles it added, unless the world was locked and it did nothing
#define B2_REC_SPAWN( liquidId, Name, added, ... )                                                                             \
	do                                                                                                                         \
	{                                                                                                                          \
		b2World* recWorld_ = b2GetWorld( ( liquidId ).world0 );                                                               \
		if ( recWorld_->locked == false )                                                                                      \
		{                                                                                                                      \
			B2_REC_CREATE( recWorld_, Name, added, __VA_ARGS__ );                                                              \
		}                                                                                                                      \
	}                                                                                                                          \
	while ( 0 )

static int SpawnLattice( b2LiquidId liquidId, const b2fSpawnShape* s, b2Vec2 velocity )
{
	b2fFluid* f;
	int liquid = b2fGetLiquid( liquidId, &f );
	if ( SpawnCalled( liquidId ) == false )
	{
		return 0;
	}

	float spacing = SpawnSpacing( f ), rowSpacing = 0.8660254f * spacing;
	int rows = (int)( s->halfHeight / rowSpacing ) + 1, columns = (int)( s->halfWidth / spacing ) + 1;
	CoarsenFromHere( f, liquid );
	int added = 0;
	for ( int row = -rows; row <= rows; ++row )
	{
		float y = (float)row * rowSpacing, shift = row & 1 ? 0.5f * spacing : 0.0f;
		for ( int column = -columns - 1; column <= columns; ++column )
		{
			b2Vec2 p = { (float)column * spacing + shift, y };
			if ( InsideSpawnShape( s, p ) == false )
			{
				continue;
			}
			b2Pos world = b2OffsetPos( s->center, p );
			if ( AddParticle( f, liquid, world.x, world.y, f->def.particleSpacing, velocity ) == false )
			{
				return added;
			}
			added += 1;
		}
	}
	return added;
}

int b2Liquid_SpawnBox( b2LiquidId liquidId, b2Pos lower, b2Pos upper, b2Vec2 velocity )
{
	float halfWidth = 0.5f * (float)( upper.x - lower.x ), halfHeight = 0.5f * (float)( upper.y - lower.y );
	b2fSpawnShape s = {
		.type = b2_polygonShape,
		.polygon = b2MakeBox( halfWidth, halfHeight ),
		.rotation = b2Rot_identity,
		.center = { 0.5f * ( lower.x + upper.x ), 0.5f * ( lower.y + upper.y ) },
		.halfWidth = halfWidth,
		.halfHeight = halfHeight,
	};
	int added = SpawnLattice( liquidId, &s, velocity );
	B2_REC_SPAWN( liquidId, LiquidSpawnBox, added, liquidId, lower, upper, velocity );
	return added;
}

int b2Liquid_SpawnCircle( b2LiquidId liquidId, b2Pos center, float radius, b2Vec2 velocity )
{
	b2fSpawnShape s = { .type = b2_circleShape, .circle = { b2Vec2_zero, radius }, .center = center };
	s.halfWidth = s.halfHeight = radius;
	int added = SpawnLattice( liquidId, &s, velocity );
	B2_REC_SPAWN( liquidId, LiquidSpawnCircle, added, liquidId, center, radius, velocity );
	return added;
}

int b2Liquid_SpawnCapsule( b2LiquidId liquidId, b2Pos center1, b2Pos center2, float radius, b2Vec2 velocity )
{
	b2Vec2 half = b2MulSV( 0.5f, b2SubPos( center2, center1 ) );
	b2fSpawnShape s = { .type = b2_capsuleShape, .capsule = { b2Neg( half ), half, radius }, .center = b2OffsetPos( center1, half ) };
	s.halfWidth = b2AbsFloat( half.x ) + radius;
	s.halfHeight = b2AbsFloat( half.y ) + radius;
	int added = SpawnLattice( liquidId, &s, velocity );
	B2_REC_SPAWN( liquidId, LiquidSpawnCapsule, added, liquidId, center1, center2, radius, velocity );
	return added;
}

int b2Liquid_SpawnPolygon( b2LiquidId liquidId, b2Pos origin, b2Rot rotation, const b2Polygon* polygon, b2Vec2 velocity )
{
	b2fSpawnShape s = { .type = b2_polygonShape, .polygon = *polygon, .rotation = rotation, .center = origin };
	for ( int i = 0; i < polygon->count; ++i )
	{
		b2Vec2 v = b2RotateVector( rotation, polygon->vertices[i] );
		s.halfWidth = b2MaxFloat( s.halfWidth, b2AbsFloat( v.x ) + polygon->radius );
		s.halfHeight = b2MaxFloat( s.halfHeight, b2AbsFloat( v.y ) + polygon->radius );
	}
	int added = SpawnLattice( liquidId, &s, velocity );
	B2_REC_SPAWN( liquidId, LiquidSpawnPolygon, added, liquidId, origin, rotation, *polygon, velocity );
	return added;
}

b2EmitterDef b2DefaultEmitterDef( void )
{
	b2EmitterDef def = { 0 };
	def.point2 = (b2Pos){ b2_lengthUnitsPerMeter, 0.0f };
	def.enabled = true;
	def.internalValue = B2_SECRET_COOKIE;
	return def;
}

// A body must be in the emitter's own world
static bool IsValidEmitterDef( const b2EmitterDef* def, uint16_t world0 )
{
	B2_CHECK_DEF( def );
	B2_CHECK_INPUT_RETURN( B2_IS_NULL( def->bodyId ) || ( def->bodyId.world0 == world0 && b2Body_IsValid( def->bodyId ) ), false );
	B2_CHECK_INPUT_RETURN( b2IsValidPos( def->point1 ) && b2IsValidPos( def->point2 ), false );
	B2_CHECK_INPUT_RETURN( b2IsValidVec2( def->velocity ), false );
	return true;
}

static void EndEmitter( b2fFluid* f, b2fEmitter* emitter )
{
	emitter->alive = false;
	while ( f->emitterCount > 0 && f->emitters[f->emitterCount - 1].alive == false )
	{
		f->emitterCount -= 1;
	}
}

// An emitter learns that its body is gone the next time anything looks at it, before a step or through its id. Box2D
// keeps no list of what refers to a body, and a generation check costs nothing.
static bool EndIfBodyGone( b2fFluid* f, b2fEmitter* emitter )
{
	if ( B2_IS_NON_NULL( emitter->def.bodyId ) && b2Body_IsValid( emitter->def.bodyId ) == false )
	{
		EndEmitter( f, emitter );
		return true;
	}
	return false;
}

static b2fEmitter* FindEmitter( b2EmitterId emitterId, b2fFluid** fluid )
{
	b2World* world = b2FindWorld( emitterId.world0 );
	if ( world == NULL || world->fluid == NULL )
	{
		return NULL;
	}

	b2fFluid* f = world->fluid;
	int index = emitterId.index1 - 1;
	if ( index < 0 || index >= f->emitterCount )
	{
		return NULL;
	}

	b2fEmitter* emitter = f->emitters + index;
	if ( emitter->alive == false || emitterId.generation != emitter->generation || EndIfBodyGone( f, emitter ) )
	{
		return NULL;
	}
	*fluid = f;
	return emitter;
}

static b2fEmitter* GetEmitter( b2EmitterId emitterId )
{
	b2fFluid* f;
	b2fEmitter* emitter = FindEmitter( emitterId, &f );
	B2_ASSERT( emitter != NULL );
	return emitter;
}

b2EmitterId b2CreateEmitter( b2LiquidId liquidId, const b2EmitterDef* def )
{
	b2fFluid* f;
	int liquid = b2fGetLiquid( liquidId, &f );
	if ( IsValidEmitterDef( def, liquidId.world0 ) == false )
	{
		return b2_nullEmitterId;
	}
	b2World* world = b2GetWorld( liquidId.world0 );
	B2_ASSERT( world->locked == false );
	if ( world->locked )
	{
		return b2_nullEmitterId;
	}

	int index = 0;
	while ( index < f->emitterCount && f->emitters[index].alive )
	{
		index += 1;
	}
	if ( index == B2F_MAX_EMITTERS )
	{
		return b2_nullEmitterId;
	}

	f->emitterCount = index < f->emitterCount ? f->emitterCount : index + 1;
	b2fEmitter* emitter = f->emitters + index;
	world->emitterSerial += 1;
	*emitter = ( b2fEmitter ){ .def = *def, .liquid = liquid, .alive = true, .generation = world->emitterSerial };
	b2EmitterId id = { index + 1, liquidId.world0, emitter->generation };
	B2_REC_CREATE( world, CreateEmitter, id, liquidId, *def );
	return id;
}

void b2DestroyEmitter( b2EmitterId emitterId )
{
	b2fFluid* f;
	b2fEmitter* emitter = FindEmitter( emitterId, &f );
	B2_ASSERT( emitter != NULL );
	if ( emitter != NULL )
	{
		EndEmitter( f, emitter );
		B2_REC( f->world, DestroyEmitter, emitterId );
	}
}

bool b2Emitter_IsValid( b2EmitterId emitterId )
{
	b2fFluid* f;
	return FindEmitter( emitterId, &f ) != NULL;
}

void b2Emitter_SetDef( b2EmitterId emitterId, const b2EmitterDef* def )
{
	b2fFluid* f;
	b2fEmitter* emitter = FindEmitter( emitterId, &f );
	B2_ASSERT( emitter != NULL );
	if ( emitter != NULL && IsValidEmitterDef( def, emitterId.world0 ) )
	{
		emitter->def = *def;
		B2_REC( f->world, EmitterSetDef, emitterId, *def );
	}
}

b2EmitterDef b2Emitter_GetDef( b2EmitterId emitterId )
{
	b2fEmitter* emitter = GetEmitter( emitterId );
	return emitter != NULL ? emitter->def : b2DefaultEmitterDef();
}

void b2fDestroyEmitters( b2fFluid* f, int liquid )
{
	for ( int i = 0; i < f->emitterCount; ++i )
	{
		if ( f->emitters[i].alive && f->emitters[i].liquid == liquid )
		{
			EndEmitter( f, f->emitters + i );
		}
	}
}

// Rows go across the mouth 0.866 spawn spacings apart along its normal, as in a spawned lattice, each with as many
// particles as fit at the spawn spacing, every other row shifted half a place. The liquid's travel across the mouth
// is summed, a row is owed each time it passes the row spacing, and what is left over is how far that row has gone
// since: it is put that far along the emitter's velocity, which is also where it would be by now.
//
// A mouth on a body is placed where the body is at the start of the step, and its velocity turned with the body. A row
// that left a moving mouth some time ago has moved with the liquid's velocity over the ground, the mouth with the
// body's, so it is now ahead of the mouth by the velocity relative to the mouth: the same offset as for a fixed mouth.
// Each particle then gets the body's velocity at its own point added, which carries a sprinkler's spin.
void b2fStepEmitters( b2fFluid* f, float timeStep )
{
	float spacing = SpawnSpacing( f ), rowSpacing = 0.8660254f * spacing;
	for ( int e = 0; e < f->emitterCount; ++e )
	{
		b2fEmitter* emitter = f->emitters + e;
		if ( emitter->alive == false || EndIfBodyGone( f, emitter ) )
		{
			continue;
		}

		const b2EmitterDef* def = &emitter->def;
		bool riding = B2_IS_NON_NULL( def->bodyId );
		b2Pos point1 = def->point1;
		b2Vec2 mouth = b2SubPos( def->point2, def->point1 );
		b2Vec2 v = def->velocity;
		if ( riding )
		{
			b2WorldTransform t = b2Body_GetTransform( def->bodyId );
			point1 = b2TransformWorldPoint( t, b2ToVec2( def->point1 ) );
			mouth = b2RotateVector( t.q, mouth );
			v = b2RotateVector( t.q, v );
		}

		float length = b2Length( mouth );
		if ( def->enabled == false || length <= 0.0f || timeStep <= 0.0f )
		{
			continue;
		}

		float speed = b2AbsFloat( v.x * mouth.y - v.y * mouth.x ) / length;
		if ( speed <= 0.0f )
		{
			continue;
		}

		int count = (int)( length / spacing + 0.5f );
		count = count > 1 ? count : 1;
		float step = 1.0f / (float)count;
		emitter->travel += speed * timeStep;
		while ( emitter->travel >= rowSpacing )
		{
			emitter->travel -= rowSpacing;
			b2Vec2 ahead = b2MulSV( emitter->travel / speed, v );
			float shift = emitter->rows & 1 ? 0.75f : 0.25f;
			emitter->rows += 1;
			for ( int k = 0; k < count; ++k )
			{
				b2Pos p = b2OffsetPos( point1, b2MulAdd( ahead, ( (float)k + shift ) * step, mouth ) );
				b2Vec2 velocity = riding ? b2Add( v, b2Body_GetWorldPointVelocity( def->bodyId, p ) ) : v;
				if ( AddParticle( f, emitter->liquid, p.x, p.y, f->def.particleSpacing, velocity ) == false )
				{
					// Full: nothing is owed for the time it stays full
					emitter->travel = 0.0f;
					break;
				}
			}
		}
	}
}
