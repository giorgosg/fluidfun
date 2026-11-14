// SPDX-License-Identifier: MIT
// Adding particles: spawning in shapes, and emitters.
#include "fluid_internal.h"

#include "core.h"
#include "physics_world.h"

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
	f->restSizesStale = true;

	// The particle has no grid cell until the next sub-step sorts it, so sizes must not adapt before then: the search
	// for partners would read past the grid
	f->gridValid = false;
	return true;
}

// The particles a spawn adds from here on take their sizes at the start of the next step, before they move, or do if
// the step's substep turns out not to be the one guessed. Ids only grow, so every particle of this liquid from the
// next id on is new.
static void CoarsenFromHere( b2fFluid* f, int liquid, bool guessed )
{
	uint32_t bit = 1u << liquid;
	if ( ( ( f->coarsenLiquids | f->guessedLiquids ) & bit ) == 0 )
	{
		f->coarsenFrom[liquid] = f->nextParticleId;
	}
	f->coarsenLiquids |= guessed ? 0u : bit;
	f->guessedLiquids |= guessed ? bit : 0u;
}

static bool SpawnCalled( b2LiquidId liquidId )
{
	b2World* world = b2GetWorld( liquidId.world0 );
	B2_ASSERT( world->locked == false );
	return world->locked == false;
}

// With adaptive sizes particles start at the size they would settle to, so a pool does not begin by merging all
// over. Density boxes read depth as distance to whatever is not liquid, walls and floor included, so this grades
// by distance to the nearest edge of the box: rectangular rings from the edge inwards, each as coarse as its
// distance asks, with its particles spread evenly around it. How deep sizes reach depends on the substep, which is
// only known once the fluid has stepped. Before that it is taken as B2F_FIRST_SUBSTEP and the particles are looked at
// again at the start of the first step, which puts right any far from the size the real substep asks for.
static int CreateGradedParticles( b2fFluid* f, int liquid, b2Pos lower, b2Pos upper, b2Vec2 velocity )
{
	const b2LiquidDef* def = &f->liquids[liquid].def;
	float spacing = f->spacing, ratio = def->tuning.adaptiveRatio;
	// As in the size pass, only gravity's strength matters, since depth is measured the same way in every direction
	float g = b2AbsFloat( def->gravityScale ) * b2Length( b2World_GetGravity( f->worldId ) );
	float h = f->h;
	if ( h <= 0.0f )
	{
		h = B2F_FIRST_SUBSTEP;
		CoarsenFromHere( f, liquid, true );
	}
	float reach = b2fAdaptiveReach( f, liquid, g, h );
	b2fCoord cx = 0.5f * ( lower.x + upper.x ), cy = 0.5f * ( lower.y + upper.y );
	int added = 0, ring = 0;
	float inset = 0.0f;
	while ( true )
	{
		float a = (float)( 0.5f * ( upper.x - lower.x ) ) - inset, b = (float)( 0.5f * ( upper.y - lower.y ) ) - inset;
		if ( a <= 0.0f || b <= 0.0f )
		{
			break;
		}

		float t = inset / reach;
		t = t < 1.0f ? t : 1.0f;
		float pitch = B2F_SPAWN_SPACING * spacing * sqrtf( 1.0f + ( ratio * ratio - 1.0f ) * t );
		float dy = 0.8660254f * pitch;

		// What is left is too thin for a ring, so it gets one line down its middle
		if ( a < dy || b < dy )
		{
			bool across = a > b;
			float length = 2.0f * ( across ? a : b );
			int count = (int)( length / pitch + 0.5f );
			count = count > 1 ? count : 1;
			float step = length / (float)count, size = sqrtf( step * 2.0f * ( across ? b : a ) / 0.8660254f ) / B2F_SPAWN_SPACING;
			for ( int i = 0; i < count; ++i )
			{
				float u = -0.5f * length + ( (float)i + 0.5f ) * step;
				if ( AddParticle( f, liquid, across ? cx + u : cx, across ? cy : cy + u, size, velocity ) == false )
				{
					return added;
				}
				added += 1;
			}
			break;
		}

		// Walk the ring's middle line, from the lower left corner anticlockwise
		float ha = a - 0.5f * dy, hb = b - 0.5f * dy;
		float perimeter = 4.0f * ( ha + hb );
		int count = (int)( perimeter / pitch + 0.5f );
		count = count > 4 ? count : 4;
		float step = perimeter / (float)count;
		float size = sqrtf( step * dy / 0.8660254f ) / B2F_SPAWN_SPACING;
		for ( int i = 0; i < count; ++i )
		{
			float u = ( (float)i + ( ring & 1 ? 0.5f : 0.0f ) ) * step;
			b2fCoord x, y;
			if ( u < 2.0f * ha )
			{
				x = cx - ha + u, y = cy - hb;
			}
			else if ( u < 2.0f * ( ha + hb ) )
			{
				x = cx + ha, y = cy - hb + ( u - 2.0f * ha );
			}
			else if ( u < 2.0f * ( 2.0f * ha + hb ) )
			{
				x = cx + ha - ( u - 2.0f * ( ha + hb ) ), y = cy + hb;
			}
			else
			{
				x = cx - ha, y = cy + hb - ( u - 2.0f * ( 2.0f * ha + hb ) );
			}

			if ( AddParticle( f, liquid, x, y, size, velocity ) == false )
			{
				return added;
			}
			added += 1;
		}
		inset += dy;
		ring += 1;
	}
	return added;
}

// A lattice at the finest size over the shape's bounds, centred on a point near the shape, keeping the points inside
// it, edges included. The test takes points relative to that centre, in float. Every shape spawns this way, and a box
// too unless its liquid has adaptive sizes. With adaptive sizes and adaptiveCoarsenSpawns the new particles take their
// sizes at the start of the next step.
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

static int FillLattice( b2fFluid* f, int liquid, const b2fSpawnShape* s, b2Vec2 velocity )
{
	float spacing = SpawnSpacing( f ), rowSpacing = 0.8660254f * spacing;
	int rows = (int)( s->halfHeight / rowSpacing ) + 1, columns = (int)( s->halfWidth / spacing ) + 1;
	const b2LiquidTuning* tuning = &f->liquids[liquid].def.tuning;
	if ( tuning->adaptiveRatio > 1.0f && tuning->adaptiveCoarsenSpawns )
	{
		CoarsenFromHere( f, liquid, false );
	}
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
			if ( AddParticle( f, liquid, world.x, world.y, f->spacing, velocity ) == false )
			{
				return added;
			}
			added += 1;
		}
	}
	return added;
}

static int SpawnLattice( b2LiquidId liquidId, const b2fSpawnShape* s, b2Vec2 velocity )
{
	b2fFluid* f;
	int liquid = b2fGetLiquid( liquidId, &f );
	if ( SpawnCalled( liquidId ) == false )
	{
		return 0;
	}
	return FillLattice( f, liquid, s, velocity );
}

int b2Liquid_SpawnBox( b2LiquidId liquidId, b2Pos lower, b2Pos upper, b2Vec2 velocity )
{
	b2fFluid* f;
	int liquid = b2fGetLiquid( liquidId, &f );
	if ( SpawnCalled( liquidId ) == false )
	{
		return 0;
	}

	if ( f->liquids[liquid].def.tuning.adaptiveRatio > 1.0f )
	{
		return CreateGradedParticles( f, liquid, lower, upper, velocity );
	}

	float halfWidth = 0.5f * (float)( upper.x - lower.x ), halfHeight = 0.5f * (float)( upper.y - lower.y );
	b2fSpawnShape s = {
		.type = b2_polygonShape,
		.polygon = b2MakeBox( halfWidth, halfHeight ),
		.rotation = b2Rot_identity,
		.center = { 0.5f * ( lower.x + upper.x ), 0.5f * ( lower.y + upper.y ) },
		.halfWidth = halfWidth,
		.halfHeight = halfHeight,
	};
	return FillLattice( f, liquid, &s, velocity );
}

int b2Liquid_SpawnCircle( b2LiquidId liquidId, b2Pos center, float radius, b2Vec2 velocity )
{
	b2fSpawnShape s = { .type = b2_circleShape, .circle = { b2Vec2_zero, radius }, .center = center };
	s.halfWidth = s.halfHeight = radius;
	return SpawnLattice( liquidId, &s, velocity );
}

int b2Liquid_SpawnCapsule( b2LiquidId liquidId, b2Pos center1, b2Pos center2, float radius, b2Vec2 velocity )
{
	b2Vec2 half = b2MulSV( 0.5f, b2SubPos( center2, center1 ) );
	b2fSpawnShape s = { .type = b2_capsuleShape, .capsule = { b2Neg( half ), half, radius }, .center = b2OffsetPos( center1, half ) };
	s.halfWidth = b2AbsFloat( half.x ) + radius;
	s.halfHeight = b2AbsFloat( half.y ) + radius;
	return SpawnLattice( liquidId, &s, velocity );
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
	return SpawnLattice( liquidId, &s, velocity );
}

b2EmitterDef b2DefaultEmitterDef( void )
{
	b2EmitterDef def = { 0 };
	def.point2 = (b2Pos){ 1.0f, 0.0f };
	def.enabled = true;
	return def;
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
	if ( emitter->alive == false || emitterId.generation != emitter->generation )
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
	return ( b2EmitterId ){ index + 1, liquidId.world0, emitter->generation };
}

static void EndEmitter( b2fFluid* f, b2fEmitter* emitter )
{
	emitter->alive = false;
	while ( f->emitterCount > 0 && f->emitters[f->emitterCount - 1].alive == false )
	{
		f->emitterCount -= 1;
	}
}

void b2DestroyEmitter( b2EmitterId emitterId )
{
	b2fFluid* f;
	b2fEmitter* emitter = FindEmitter( emitterId, &f );
	B2_ASSERT( emitter != NULL );
	if ( emitter != NULL )
	{
		EndEmitter( f, emitter );
	}
}

bool b2Emitter_IsValid( b2EmitterId emitterId )
{
	b2fFluid* f;
	return FindEmitter( emitterId, &f ) != NULL;
}

void b2Emitter_SetDef( b2EmitterId emitterId, const b2EmitterDef* def )
{
	b2fEmitter* emitter = GetEmitter( emitterId );
	if ( emitter != NULL )
	{
		emitter->def = *def;
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
void b2fStepEmitters( b2fFluid* f, float timeStep )
{
	float spacing = SpawnSpacing( f ), rowSpacing = 0.8660254f * spacing;
	for ( int e = 0; e < f->emitterCount; ++e )
	{
		b2fEmitter* emitter = f->emitters + e;
		const b2EmitterDef* def = &emitter->def;
		b2Vec2 mouth = b2SubPos( def->point2, def->point1 );
		float length = b2Length( mouth );
		if ( emitter->alive == false || def->enabled == false || length <= 0.0f || timeStep <= 0.0f )
		{
			continue;
		}

		b2Vec2 v = def->velocity;
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
				b2Pos p = b2OffsetPos( def->point1, b2MulAdd( ahead, ( (float)k + shift ) * step, mouth ) );
				if ( AddParticle( f, emitter->liquid, p.x, p.y, f->spacing, v ) == false )
				{
					// Full: nothing is owed for the time it stays full
					emitter->travel = 0.0f;
					break;
				}
			}
		}
	}
}
