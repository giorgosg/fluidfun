// SPDX-License-Identifier: MIT
// What the liquid tells the caller: the state of bodies in it, events when bodies and sensors gain or lose liquid,
// and queries on the particles. All of it reads what the step left and changes nothing the passes use.
#include "fluid_internal.h"

// Grows an array of events by one and returns the new slot
static void* Push( void** array, int* count, int* capacity, size_t size )
{
	if ( *count == *capacity )
	{
		*capacity = *capacity == 0 ? 16 : 2 * *capacity;
		*array = realloc( *array, (size_t)*capacity * size );
	}
	return (char*)*array + (size_t)( *count )++ * size;
}

static void BeginBody( b2fFluid* f, b2BodyId id, b2LiquidId liquidId )
{
	b2FluidBodyBeginEvent* event = Push( (void**)&f->bodyBeginEvents, &f->bodyBeginCount, &f->bodyBeginCapacity, sizeof( *event ) );
	*event = ( b2FluidBodyBeginEvent ){ id, liquidId };
}

static void EndBody( b2fFluid* f, b2BodyId id )
{
	b2FluidBodyEndEvent* event = Push( (void**)&f->bodyEndEvents, &f->bodyEndCount, &f->bodyEndCapacity, sizeof( *event ) );
	*event = ( b2FluidBodyEndEvent ){ id };
}

static int FindRecord( const b2fFluid* f, b2BodyId id )
{
	int k = id.index1 - 1;
	if ( 0 <= k && k < f->recordOfBodyCapacity && f->recordOfBody[k] > 0 )
	{
		int r = f->recordOfBody[k] - 1;
		if ( B2_ID_EQUALS( f->records[r].id, id ) )
		{
			return r;
		}
	}
	return -1;
}

static int AddRecord( b2fFluid* f, b2BodyId id )
{
	if ( f->recordCount == f->recordCapacity )
	{
		f->recordCapacity = f->recordCapacity == 0 ? 16 : 2 * f->recordCapacity;
		f->records = realloc( f->records, (size_t)f->recordCapacity * sizeof( b2fBodyRecord ) );
	}
	int k = id.index1 - 1;
	if ( k >= f->recordOfBodyCapacity )
	{
		int capacity = f->recordOfBodyCapacity == 0 ? 64 : f->recordOfBodyCapacity;
		while ( capacity <= k )
		{
			capacity *= 2;
		}
		f->recordOfBody = realloc( f->recordOfBody, (size_t)capacity * sizeof( int ) );
		memset( f->recordOfBody + f->recordOfBodyCapacity, 0, (size_t)( capacity - f->recordOfBodyCapacity ) * sizeof( int ) );
		f->recordOfBodyCapacity = capacity;
	}

	int r = f->recordCount++;
	f->records[r] = ( b2fBodyRecord ){ .id = id, .state = { .liquidId = b2_nullLiquidId } };
	f->recordOfBody[k] = r + 1;
	return r;
}

// Moves the last record into the gap. A slot that already names a newer body with the same index is left alone.
static void RemoveRecord( b2fFluid* f, int r )
{
	int k = f->records[r].id.index1 - 1;
	if ( f->recordOfBody[k] == r + 1 )
	{
		f->recordOfBody[k] = 0;
	}

	int last = f->recordCount - 1;
	if ( r != last )
	{
		f->records[r] = f->records[last];
		int moved = f->records[r].id.index1 - 1;
		if ( f->recordOfBody[moved] == last + 1 )
		{
			f->recordOfBody[moved] = r + 1;
		}
	}
	f->recordCount -= 1;
}

// inLiquid follows whether liquid was in reach, once that has held for B2F_WET_STEPS steps in a row
static void Settle( b2fFluid* f, b2fBodyRecord* record, bool wet )
{
	if ( wet == record->state.inLiquid )
	{
		record->run = 0;
		return;
	}

	record->run += 1;
	if ( record->run < B2F_WET_STEPS )
	{
		return;
	}

	record->run = 0;
	record->state.inLiquid = wet;
	if ( wet )
	{
		BeginBody( f, record->id, record->state.liquidId );
	}
	else
	{
		EndBody( f, record->id );
	}
}

// Which liquid each body near liquid is mostly in, from the contacts of the last substep, which are still those of
// the particles where they are. A fluid with one liquid needs none of this.
static void FindLiquids( b2fFluid* f, int liveLiquids, int onlyLiquid )
{
	if ( liveLiquids <= 1 )
	{
		b2LiquidId liquidId = liveLiquids == 1 ? b2fMakeLiquidId( f, onlyLiquid ) : b2_nullLiquidId;
		for ( int r = 0; r < f->recordCount; ++r )
		{
			f->records[r].state.liquidId = f->records[r].seen ? liquidId : f->records[r].state.liquidId;
		}
		return;
	}

	// With no bodies there is nothing to find, and no array yet to clear
	int needed = f->bodyCount * B2F_MAX_LIQUIDS;
	if ( needed == 0 )
	{
		return;
	}
	if ( needed > f->bodyLiquidCapacity )
	{
		f->bodyLiquidCapacity = needed;
		f->bodyLiquidMass = realloc( f->bodyLiquidMass, (size_t)needed * sizeof( float ) );
	}
	memset( f->bodyLiquidMass, 0, (size_t)needed * sizeof( float ) );

	float spacing = f->spacing;
	for ( int i = 0; i < f->count; ++i )
	{
		const b2fContact* contacts = f->contacts + B2F_CONTACTS_PER_PARTICLE * i;
		int count = f->contactCount[i];
		float ratio = f->size[i] / spacing;
		float m = f->liquids[f->liquid[i]].mass * ratio * ratio;
		for ( int c = 0; c < count; ++c )
		{
			// Once per body, as the solver counts it
			bool first = true;
			for ( int e = 0; e < c; ++e )
			{
				first = first && contacts[e].bodyIndex != contacts[c].bodyIndex;
			}
			if ( first )
			{
				f->bodyLiquidMass[contacts[c].bodyIndex * B2F_MAX_LIQUIDS + f->liquid[i]] += m;
			}
		}
	}

	for ( int b = 0; b < f->bodyCount; ++b )
	{
		int r = FindRecord( f, f->bodies[b].id );
		if ( r < 0 )
		{
			continue;
		}

		const float* masses = f->bodyLiquidMass + b * B2F_MAX_LIQUIDS;
		int best = -1;
		for ( int l = 0; l < f->liquidCount; ++l )
		{
			best = masses[l] > 0.0f && ( best < 0 || masses[l] > masses[best] ) ? l : best;
		}
		if ( best >= 0 )
		{
			f->records[r].state.liquidId = b2fMakeLiquidId( f, best );
		}
	}
}

static void UpdateBodies( b2fFluid* f )
{
	float invTime = 1.0f / f->stepTime, invSubSteps = 1.0f / (float)f->stepSubSteps;
	for ( int r = 0; r < f->recordCount; ++r )
	{
		f->records[r].seen = false;
	}

	for ( int b = 0; b < f->bodyCount; ++b )
	{
		const b2fBody* body = f->bodies + b;
		int r = FindRecord( f, body->id );
		if ( r < 0 && body->stepWet == false )
		{
			continue;
		}

		r = r < 0 ? AddRecord( f, body->id ) : r;
		b2fBodyRecord* record = f->records + r;
		record->seen = true;
		record->wet = body->stepWet;
		b2FluidBodyState* state = &record->state;
		state->force = b2MulSV( invTime, body->stepImpulse );
		state->torque = invTime * body->stepAngular;
		state->submerged = invSubSteps * body->stepSubmerged;
	}

	int liveLiquids = 0, onlyLiquid = 0;
	for ( int l = 0; l < f->liquidCount; ++l )
	{
		if ( f->liquids[l].alive )
		{
			liveLiquids += 1;
			onlyLiquid = l;
		}
	}
	FindLiquids( f, liveLiquids, onlyLiquid );

	for ( int r = 0; r < f->recordCount; )
	{
		b2fBodyRecord* record = f->records + r;
		bool wet = record->seen && record->wet;
		if ( record->seen == false && b2Body_IsValid( record->id ) == false )
		{
			// Destroyed, so it ends now whatever the delay
			if ( record->state.inLiquid )
			{
				EndBody( f, record->id );
			}
			RemoveRecord( f, r );
			continue;
		}
		else if ( record->seen == false )
		{
			// Still there but no longer near the liquid
			b2LiquidId liquidId = record->state.liquidId;
			bool inLiquid = record->state.inLiquid;
			record->state = ( b2FluidBodyState ){ .inLiquid = inLiquid, .liquidId = liquidId };
		}

		Settle( f, record, wet );
		if ( record->state.inLiquid == false && wet == false )
		{
			RemoveRecord( f, r );
			continue;
		}
		r += 1;
	}
}

void b2fGatherSensor( b2fFluid* f, b2ShapeId shapeId )
{
	for ( int i = 0; i < f->sensorCount; ++i )
	{
		if ( B2_ID_EQUALS( f->sensors[i].id, shapeId ) )
		{
			f->sensors[i].gathered = true;
			return;
		}
	}

	b2fSensor* sensor = Push( (void**)&f->sensors, &f->sensorCount, &f->sensorCapacity, sizeof( b2fSensor ) );
	*sensor = ( b2fSensor ){ shapeId, 0.0f, true };
}

// A sensor's outline, and the area of liquid in it
typedef struct b2fSensorShape
{
	b2fOutline outline;
	double area;
	int sensor;
} b2fSensorShape;

static void GrowBounds( b2fOutline* s, b2WorldTransform t, b2Vec2 local, float radius )
{
	b2Pos p = b2TransformWorldPoint( t, local );
	s->minX = p.x - radius < s->minX ? p.x - radius : s->minX;
	s->minY = p.y - radius < s->minY ? p.y - radius : s->minY;
	s->maxX = p.x + radius > s->maxX ? p.x + radius : s->maxX;
	s->maxY = p.y + radius > s->maxY ? p.y + radius : s->maxY;
}

bool b2fMakeOutline( b2ShapeId id, b2fOutline* s )
{
	*s = ( b2fOutline ){ .type = b2Shape_GetType( id ), .minX = FLT_MAX, .minY = FLT_MAX, .maxX = -FLT_MAX, .maxY = -FLT_MAX };
	s->transform = b2Body_GetTransform( b2Shape_GetBody( id ) );
	switch ( s->type )
	{
		case b2_circleShape:
			s->circle = b2Shape_GetCircle( id );
			GrowBounds( s, s->transform, s->circle.center, s->circle.radius );
			return true;

		case b2_capsuleShape:
			s->capsule = b2Shape_GetCapsule( id );
			GrowBounds( s, s->transform, s->capsule.center1, s->capsule.radius );
			GrowBounds( s, s->transform, s->capsule.center2, s->capsule.radius );
			return true;

		case b2_polygonShape:
			s->polygon = b2Shape_GetPolygon( id );
			for ( int j = 0; j < s->polygon.count; ++j )
			{
				GrowBounds( s, s->transform, s->polygon.vertices[j], s->polygon.radius );
			}
			return true;

		default:
			// Segments have no inside
			return false;
	}
}

b2fOutline b2fMakeBoxOutline( b2Pos lower, b2Pos upper )
{
	return ( b2fOutline ){ .type = b2_shapeTypeCount, .minX = lower.x, .minY = lower.y, .maxX = upper.x, .maxY = upper.y };
}

b2fOutline b2fMakeCircleOutline( b2Pos center, float radius )
{
	b2fOutline s = { .type = b2_circleShape, .circle = { { 0.0f, 0.0f }, radius } };
	s.transform.p = center;
	s.transform.q = b2Rot_identity;
	s.minX = center.x - radius;
	s.minY = center.y - radius;
	s.maxX = center.x + radius;
	s.maxY = center.y + radius;
	return s;
}

// Whether a point is inside the outline, testing the shape only for points inside its bounds. A box is its bounds,
// edges included.
bool b2fInsideOutline( const b2fOutline* s, b2Pos p )
{
	if ( p.x < s->minX || p.x > s->maxX || p.y < s->minY || p.y > s->maxY )
	{
		return false;
	}
	if ( s->type == b2_shapeTypeCount )
	{
		return true;
	}

	b2Vec2 local = b2InvTransformWorldPoint( s->transform, p );
	return s->type == b2_circleShape	? b2PointInCircle( &s->circle, local )
		   : s->type == b2_capsuleShape ? b2PointInCapsule( &s->capsule, local )
										: b2PointInPolygon( &s->polygon, local );
}

// The area at rest of the liquid whose particles are inside each sensor near it. A particle is in a sensor when its
// centre is inside the sensor's outline, tested only for particles inside its bounds.
static void MeasureSensors( b2fFluid* f )
{
	int count = 0;
	for ( int i = 0; i < f->sensorCount; ++i )
	{
		f->sensors[i].area = 0.0f;
		count += f->sensors[i].gathered ? 1 : 0;
	}
	if ( count == 0 || f->count == 0 )
	{
		return;
	}

	b2fSensorShape* shapes = malloc( (size_t)count * sizeof( b2fSensorShape ) );
	int shapeCount = 0;
	b2fCoord minX = FLT_MAX, minY = FLT_MAX, maxX = -FLT_MAX, maxY = -FLT_MAX;
	for ( int i = 0; i < f->sensorCount; ++i )
	{
		if ( f->sensors[i].gathered == false )
		{
			continue;
		}

		b2fSensorShape* s = shapes + shapeCount;
		*s = ( b2fSensorShape ){ .sensor = i };
		if ( b2fMakeOutline( f->sensors[i].id, &s->outline ) == false )
		{
			continue;
		}

		minX = s->outline.minX < minX ? s->outline.minX : minX;
		minY = s->outline.minY < minY ? s->outline.minY : minY;
		maxX = s->outline.maxX > maxX ? s->outline.maxX : maxX;
		maxY = s->outline.maxY > maxY ? s->outline.maxY : maxY;
		shapeCount += 1;
	}

	float pitch = B2F_REST_PITCH;
	for ( int i = 0; i < f->count; ++i )
	{
		b2Pos p = f->pos[i];
		if ( p.x < minX || p.x > maxX || p.y < minY || p.y > maxY )
		{
			continue;
		}

		for ( int k = 0; k < shapeCount; ++k )
		{
			b2fSensorShape* s = shapes + k;
			if ( b2fInsideOutline( &s->outline, p ) )
			{
				float size = pitch * f->size[i];
				s->area += 0.8660254 * size * size;
			}
		}
	}

	for ( int k = 0; k < shapeCount; ++k )
	{
		f->sensors[shapes[k].sensor].area = (float)shapes[k].area;
	}
	free( shapes );
}

static void UpdateSensors( b2fFluid* f )
{
	// A destroyed sensor that held liquid ends now. What is left held liquid last step or is near it now.
	bool* held = malloc( (size_t)( f->sensorCount > 0 ? f->sensorCount : 1 ) * sizeof( bool ) );
	for ( int i = 0; i < f->sensorCount; )
	{
		b2fSensor* sensor = f->sensors + i;
		if ( b2Shape_IsValid( sensor->id ) == false )
		{
			if ( sensor->area > 0.0f )
			{
				b2FluidSensorEndEvent* event = Push( (void**)&f->sensorEndEvents, &f->sensorEndCount, &f->sensorEndCapacity, sizeof( *event ) );
				event->sensorShapeId = sensor->id;
			}
			*sensor = f->sensors[--f->sensorCount];
			continue;
		}
		held[i] = sensor->area > 0.0f;
		i += 1;
	}

	MeasureSensors( f );

	// Begin and end in the order the sensors are listed, then keep only those holding liquid; the next step's gather
	// adds the others near it again
	int kept = 0;
	for ( int i = 0; i < f->sensorCount; ++i )
	{
		b2fSensor sensor = f->sensors[i];
		bool holds = sensor.area > 0.0f;
		if ( holds && held[i] == false )
		{
			b2FluidSensorBeginEvent* event = Push( (void**)&f->sensorBeginEvents, &f->sensorBeginCount, &f->sensorBeginCapacity, sizeof( *event ) );
			event->sensorShapeId = sensor.id;
		}
		else if ( holds == false && held[i] )
		{
			b2FluidSensorEndEvent* event = Push( (void**)&f->sensorEndEvents, &f->sensorEndCount, &f->sensorEndCapacity, sizeof( *event ) );
			event->sensorShapeId = sensor.id;
		}
		if ( holds )
		{
			f->sensors[kept++] = sensor;
		}
	}
	f->sensorCount = kept;
	free( held );
}

void b2fFinishStep( b2fFluid* f )
{
	f->bodyBeginCount = 0;
	f->bodyEndCount = 0;
	f->sensorBeginCount = 0;
	f->sensorEndCount = 0;

	// A step of no time ran no substeps, and leaves the last step's state as it was
	if ( f->stepSubSteps == 0 || f->stepTime <= 0.0f )
	{
		return;
	}

	UpdateBodies( f );
	if ( f->sensorCount > 0 )
	{
		UpdateSensors( f );
	}
}

void b2fFreeState( b2fFluid* f )
{
	free( f->records );
	free( f->recordOfBody );
	free( f->sensors );
	free( f->bodyLiquidMass );
	free( f->bodyBeginEvents );
	free( f->bodyEndEvents );
	free( f->sensorBeginEvents );
	free( f->sensorEndEvents );
}

b2FluidBodyState b2Fluid_GetBodyState( b2FluidId fluidId, b2BodyId bodyId )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	int r = FindRecord( f, bodyId );
	if ( r < 0 )
	{
		return ( b2FluidBodyState ){ .liquidId = b2_nullLiquidId };
	}
	return f->records[r].state;
}

b2FluidEvents b2Fluid_GetEvents( b2FluidId fluidId )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	return ( b2FluidEvents ){
		.bodyBeginEvents = f->bodyBeginEvents,
		.bodyEndEvents = f->bodyEndEvents,
		.sensorBeginEvents = f->sensorBeginEvents,
		.sensorEndEvents = f->sensorEndEvents,
		.bodyBeginCount = f->bodyBeginCount,
		.bodyEndCount = f->bodyEndCount,
		.sensorBeginCount = f->sensorBeginCount,
		.sensorEndCount = f->sensorEndCount,
	};
}

float b2Fluid_GetSensorArea( b2FluidId fluidId, b2ShapeId sensorShapeId )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	for ( int i = 0; i < f->sensorCount; ++i )
	{
		if ( B2_ID_EQUALS( f->sensors[i].id, sensorShapeId ) )
		{
			return f->sensors[i].area;
		}
	}
	return 0.0f;
}

b2FluidAmount b2Fluid_MeasureBox( b2FluidId fluidId, b2Pos lowerBound, b2Pos upperBound )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	int count = 0;
	double area = 0.0, mass = 0.0;
	float pitch = B2F_REST_PITCH;
	for ( int i = 0; i < f->count; ++i )
	{
		b2Pos p = f->pos[i];
		if ( p.x < lowerBound.x || p.x > upperBound.x || p.y < lowerBound.y || p.y > upperBound.y )
		{
			continue;
		}

		float size = pitch * f->size[i];
		double a = 0.8660254 * size * size;
		count += 1;
		area += a;
		mass += a * f->liquids[f->liquid[i]].def.density;
	}
	return ( b2FluidAmount ){ count, (float)area, (float)mass };
}

bool b2Fluid_GetSurfaceHeight( b2FluidId fluidId, b2Pos point, b2Pos* surface )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	b2Vec2 gravity = b2World_GetGravity( f->worldId );
	float g = b2Length( gravity );
	b2Vec2 up = g > 0.0f ? b2MulSV( -1.0f / g, gravity ) : ( b2Vec2 ){ 0.0f, 1.0f };
	float reach = 2.0f * f->def.particleSpacing, spray = 0.4f * f->restDensity, pitch = B2F_REST_PITCH;
	float best = -FLT_MAX;
	bool found = false;
	for ( int i = 0; i < f->count; ++i )
	{
		b2Vec2 d = b2SubPos( f->pos[i], point );
		float sideways = d.x * up.y - d.y * up.x;
		if ( sideways > reach || sideways < -reach || f->rho[i] < spray )
		{
			continue;
		}

		float height = b2Dot( d, up ) + 0.5f * pitch * f->size[i];
		best = height > best ? height : best;
		found = true;
	}

	if ( found )
	{
		*surface = b2OffsetPos( point, b2MulSV( best, up ) );
	}
	return found;
}
