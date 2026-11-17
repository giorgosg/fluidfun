// SPDX-License-Identifier: MIT
// What the liquid tells the caller: the state of bodies in it, events when bodies and sensors gain or lose liquid,
// and queries on the particles. All of it reads what the step left and changes nothing the passes use.
#include "fluid_internal.h"

#include "physics_world.h"

static void BeginBody( b2fFluid* f, b2BodyId id, b2LiquidId liquidId )
{
	b2Array_Push( f->bodyBeginEvents, ( (b2FluidBodyBeginEvent){ id, liquidId } ) );
}

static void EndBody( b2fFluid* f, b2BodyId id )
{
	b2Array_Push( f->bodyEndEvents, ( (b2FluidBodyEndEvent){ id } ) );
}

static int FindRecord( const b2fFluid* f, b2BodyId id )
{
	int k = id.index1 - 1;
	if ( 0 <= k && k < f->recordOfBody.count && f->recordOfBody.data[k] > 0 )
	{
		int r = f->recordOfBody.data[k] - 1;
		if ( B2_ID_EQUALS( f->records.data[r].id, id ) )
		{
			return r;
		}
	}
	return -1;
}

static int AddRecord( b2fFluid* f, b2BodyId id )
{
	int k = id.index1 - 1;
	b2fGrowTable( &f->recordOfBody, k, 0 );
	int r = f->records.count;
	b2Array_Push( f->records, ( (b2fBodyRecord){ .id = id, .state = { .liquidId = b2_nullLiquidId } } ) );
	f->recordOfBody.data[k] = r + 1;
	return r;
}

// Moves the last record into the gap. A slot that already names a newer body with the same index is left alone.
static void RemoveRecord( b2fFluid* f, int r )
{
	int k = f->records.data[r].id.index1 - 1;
	if ( f->recordOfBody.data[k] == r + 1 )
	{
		f->recordOfBody.data[k] = 0;
	}

	int last = b2Array_RemoveSwap( f->records, r );
	if ( last != B2_NULL_INDEX )
	{
		int moved = f->records.data[r].id.index1 - 1;
		if ( f->recordOfBody.data[moved] == last + 1 )
		{
			f->recordOfBody.data[moved] = r + 1;
		}
	}
}

// inLiquid follows whether liquid was in reach, once that has held for B2F_WET_TIME in a row
static void Settle( b2fFluid* f, b2fBodyRecord* record, bool wet )
{
	if ( wet == record->state.inLiquid )
	{
		record->run = 0.0f;
		return;
	}

	record->run += f->stepTime;
	if ( record->run < B2F_WET_TIME )
	{
		return;
	}

	record->run = 0.0f;
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
		for ( int r = 0; r < f->records.count; ++r )
		{
			f->records.data[r].state.liquidId = f->records.data[r].seen ? liquidId : f->records.data[r].state.liquidId;
		}
		return;
	}

	// With no bodies there is nothing to find, and no array yet to clear
	int needed = f->bodies.count * B2F_MAX_LIQUIDS;
	if ( needed == 0 )
	{
		return;
	}
	b2fArray_ResizeGrow( f->bodyLiquidMass, needed );
	memset( f->bodyLiquidMass.data, 0, (size_t)needed * sizeof( float ) );

	// The awake particles and the shell found contacts in the last substep; other asleep particles' were found earlier,
	// and since then filing may have moved them, so a body with only those round it keeps the liquid it had
	float fineSize = f->def.particleSpacing;
	for ( int k = 0; k < f->activeCount + f->shellCount; ++k )
	{
		int i = k < f->activeCount ? k : f->shell[k - f->activeCount];
		const b2fContact* contacts = f->contacts + B2F_CONTACTS_PER_PARTICLE * i;
		int count = f->contactCount[i];
		float ratio = f->size[i] / fineSize;
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
				f->bodyLiquidMass.data[contacts[c].bodyIndex * B2F_MAX_LIQUIDS + f->liquid[i]] += m;
			}
		}
	}

	for ( int b = 0; b < f->bodies.count; ++b )
	{
		int r = FindRecord( f, f->bodies.data[b].id );
		if ( r < 0 )
		{
			continue;
		}

		const float* masses = f->bodyLiquidMass.data + b * B2F_MAX_LIQUIDS;
		int best = -1;
		for ( int l = 0; l < f->liquidCount; ++l )
		{
			best = masses[l] > 0.0f && ( best < 0 || masses[l] > masses[best] ) ? l : best;
		}
		if ( best >= 0 )
		{
			f->records.data[r].state.liquidId = b2fMakeLiquidId( f, best );
		}
	}
}

static void UpdateBodies( b2fFluid* f )
{
	float invTime = 1.0f / f->stepTime, invSubSteps = 1.0f / (float)f->stepSubSteps;
	for ( int r = 0; r < f->records.count; ++r )
	{
		f->records.data[r].seen = false;
	}

	for ( int b = 0; b < f->bodies.count; ++b )
	{
		const b2fBody* body = f->bodies.data + b;
		int r = FindRecord( f, body->id );
		if ( r < 0 && body->stepWet == false )
		{
			continue;
		}

		// A body moved by forces that fell asleep in the liquid keeps what it last reported, in liquid or not, until
		// it wakes: the liquid round it is still, and asleep it feels none of it
		if ( r >= 0 && body->awake == false && body->adheres == false )
		{
			f->records.data[r].seen = true;
			f->records.data[r].wet = f->records.data[r].state.inLiquid;
			f->records.data[r].run = 0.0f;
			continue;
		}

		r = r < 0 ? AddRecord( f, body->id ) : r;
		b2fBodyRecord* record = f->records.data + r;
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

	for ( int r = 0; r < f->records.count; )
	{
		b2fBodyRecord* record = f->records.data + r;
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

void b2fGatherSensor( b2fFluid* f, b2ShapeId shapeId, uint32_t liquids )
{
	// A sensor no liquid collides with holds nothing. If it held liquid last step, not gathering it ends that.
	if ( liquids == 0 )
	{
		return;
	}

	for ( int i = 0; i < f->sensors.count; ++i )
	{
		if ( B2_ID_EQUALS( f->sensors.data[i].id, shapeId ) )
		{
			f->sensors.data[i].gathered = true;
			f->sensors.data[i].liquids = liquids;
			return;
		}
	}

	b2Array_Push( f->sensors, ( (b2fSensor){ shapeId, 0.0f, true, liquids } ) );
}

// A sensor's outline, and the area of liquid in it
typedef struct b2fSensorShape
{
	b2fOutline outline;
	double area;
	int sensor;
	uint32_t liquids;
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

b2FluidRegion b2MakeBoxRegion( b2Pos lowerBound, b2Pos upperBound )
{
	return ( b2FluidRegion ){ .type = b2_boxRegion, .lowerBound = lowerBound, .upperBound = upperBound, .shapeId = b2_nullShapeId };
}

b2FluidRegion b2MakeShapeRegion( b2ShapeId shapeId )
{
	return ( b2FluidRegion ){ .type = b2_shapeRegion, .shapeId = shapeId };
}

b2FluidRegion b2MakeProxyRegion( b2Pos origin, const b2ShapeProxy* proxy )
{
	return ( b2FluidRegion ){ .type = b2_proxyRegion, .shapeId = b2_nullShapeId, .origin = origin, .proxy = *proxy };
}

// A proxy is a circle, a capsule or a rounded polygon, placed at its origin with no rotation
static bool MakeProxyOutline( b2Pos origin, const b2ShapeProxy* proxy, b2fOutline* s )
{
	*s = ( b2fOutline ){ .transform = { origin, b2Rot_identity }, .minX = FLT_MAX, .minY = FLT_MAX, .maxX = -FLT_MAX, .maxY = -FLT_MAX };
	const b2Vec2* points = proxy->points;
	float radius = proxy->radius;
	b2Hull hull = { 0 };
	if ( proxy->count >= 3 )
	{
		hull = b2ComputeHull( points, proxy->count );
	}

	if ( proxy->count == 1 )
	{
		s->type = b2_circleShape;
		s->circle = ( b2Circle ){ points[0], radius };
	}
	else if ( hull.count >= 3 )
	{
		s->type = b2_polygonShape;
		s->polygon = b2MakePolygon( &hull, radius );
	}
	else
	{
		// Two points, or more on one line: the capsule between the two furthest apart
		int a = 0, b = 1;
		float longest = -1.0f;
		for ( int i = 0; i < proxy->count; ++i )
		{
			for ( int j = i + 1; j < proxy->count; ++j )
			{
				float d = b2DistanceSquared( points[i], points[j] );
				if ( d > longest )
				{
					longest = d, a = i, b = j;
				}
			}
		}
		s->type = b2_capsuleShape;
		s->capsule = ( b2Capsule ){ points[a], points[b], radius };
	}

	for ( int i = 0; i < proxy->count; ++i )
	{
		GrowBounds( s, s->transform, points[i], radius );
	}
	return true;
}

bool b2fMakeRegionOutline( const b2FluidRegion* region, b2fOutline* outline )
{
	switch ( region->type )
	{
		case b2_boxRegion:
			B2_CHECK_INPUT_RETURN( b2IsValidPos( region->lowerBound ) && b2IsValidPos( region->upperBound ), false );
			*outline = b2fMakeBoxOutline( region->lowerBound, region->upperBound );
			return true;

		case b2_shapeRegion:
			return b2fMakeOutline( region->shapeId, outline );

		case b2_proxyRegion:
		{
			const b2ShapeProxy* proxy = &region->proxy;
			B2_CHECK_INPUT_RETURN( b2IsValidPos( region->origin ), false );
			B2_CHECK_INPUT_RETURN( 0 < proxy->count && proxy->count <= B2_MAX_POLYGON_VERTICES, false );
			B2_CHECK_INPUT_RETURN( b2IsValidFloat( proxy->radius ) && proxy->radius >= 0.0f, false );
			for ( int i = 0; i < proxy->count; ++i )
			{
				B2_CHECK_INPUT_RETURN( b2IsValidVec2( proxy->points[i] ), false );
			}
			return MakeProxyOutline( region->origin, proxy, outline );
		}

		default:
			break;
	}
	B2_CHECK_INPUT_RETURN( region->type == b2_boxRegion || region->type == b2_shapeRegion || region->type == b2_proxyRegion, false );
	return false;
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
// centre is inside the sensor's outline, tested only for particles inside its bounds, and its liquid collides with the
// sensor by their filters.
static void MeasureSensors( b2fFluid* f )
{
	int count = 0;
	for ( int i = 0; i < f->sensors.count; ++i )
	{
		f->sensors.data[i].area = 0.0f;
		count += f->sensors.data[i].gathered ? 1 : 0;
	}
	if ( count == 0 || f->count == 0 )
	{
		return;
	}

	b2fSensorShape* shapes = b2StackAlloc( &f->world->stack, count * (int)sizeof( b2fSensorShape ), "sensor shapes" );
	int shapeCount = 0;
	b2fCoord minX = FLT_MAX, minY = FLT_MAX, maxX = -FLT_MAX, maxY = -FLT_MAX;
	for ( int i = 0; i < f->sensors.count; ++i )
	{
		if ( f->sensors.data[i].gathered == false )
		{
			continue;
		}

		b2fSensorShape* s = shapes + shapeCount;
		*s = ( b2fSensorShape ){ .sensor = i, .liquids = f->sensors.data[i].liquids };
		if ( b2fMakeOutline( f->sensors.data[i].id, &s->outline ) == false )
		{
			continue;
		}

		minX = s->outline.minX < minX ? s->outline.minX : minX;
		minY = s->outline.minY < minY ? s->outline.minY : minY;
		maxX = s->outline.maxX > maxX ? s->outline.maxX : maxX;
		maxY = s->outline.maxY > maxY ? s->outline.maxY : maxY;
		shapeCount += 1;
	}

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
			if ( ( s->liquids >> f->liquid[i] & 1u ) != 0 && b2fInsideOutline( &s->outline, p ) )
			{
				s->area += 0.8660254 * f->size[i] * f->size[i];
			}
		}
	}

	for ( int k = 0; k < shapeCount; ++k )
	{
		f->sensors.data[shapes[k].sensor].area = (float)shapes[k].area;
	}
	b2StackFree( &f->world->stack, shapes );
}

static void UpdateSensors( b2fFluid* f )
{
	// A destroyed sensor that held liquid ends now. What is left held liquid last step or is near it now.
	bool* held = b2StackAlloc( &f->world->stack, f->sensors.count * (int)sizeof( bool ), "sensors held" );
	for ( int i = 0; i < f->sensors.count; )
	{
		b2fSensor* sensor = f->sensors.data + i;
		if ( b2Shape_IsValid( sensor->id ) == false )
		{
			if ( sensor->area > 0.0f )
			{
				b2Array_Push( f->sensorEndEvents, ( (b2FluidSensorEndEvent){ sensor->id } ) );
			}
			b2Array_RemoveSwap( f->sensors, i );
			continue;
		}
		held[i] = sensor->area > 0.0f;
		i += 1;
	}

	MeasureSensors( f );

	// Begin and end in the order the sensors are listed, then keep only those holding liquid; the next step's gather
	// adds the others near it again
	int kept = 0;
	for ( int i = 0; i < f->sensors.count; ++i )
	{
		b2fSensor sensor = f->sensors.data[i];
		bool holds = sensor.area > 0.0f;
		if ( holds && held[i] == false )
		{
			b2Array_Push( f->sensorBeginEvents, ( (b2FluidSensorBeginEvent){ sensor.id } ) );
		}
		else if ( holds == false && held[i] )
		{
			b2Array_Push( f->sensorEndEvents, ( (b2FluidSensorEndEvent){ sensor.id } ) );
		}
		if ( holds )
		{
			f->sensors.data[kept++] = sensor;
		}
	}
	f->sensors.count = kept;
	b2StackFree( &f->world->stack, held );
}

void b2fFinishStep( b2fFluid* f )
{
	b2Array_Clear( f->bodyBeginEvents );
	b2Array_Clear( f->bodyEndEvents );
	b2Array_Clear( f->sensorBeginEvents );
	b2Array_Clear( f->sensorEndEvents );

	// A step of no time ran no substeps, and leaves the last step's state as it was
	if ( f->stepSubSteps == 0 || f->stepTime <= 0.0f )
	{
		return;
	}

	UpdateBodies( f );
	if ( f->sensors.count > 0 )
	{
		UpdateSensors( f );
	}
}

void b2fFreeState( b2fFluid* f )
{
	b2Array_Destroy( f->records );
	b2Array_Destroy( f->recordOfBody );
	b2Array_Destroy( f->sensors );
	b2Array_Destroy( f->bodyLiquidMass );
	b2Array_Destroy( f->bodyBeginEvents );
	b2Array_Destroy( f->bodyEndEvents );
	b2Array_Destroy( f->sensorBeginEvents );
	b2Array_Destroy( f->sensorEndEvents );
}

b2FluidBodyState b2Fluid_GetBodyState( b2FluidId fluidId, b2BodyId bodyId )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	int r = FindRecord( f, bodyId );
	if ( r < 0 )
	{
		return ( b2FluidBodyState ){ .liquidId = b2_nullLiquidId };
	}
	return f->records.data[r].state;
}

b2FluidEvents b2Fluid_GetEvents( b2FluidId fluidId )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	return ( b2FluidEvents ){
		.bodyBeginEvents = f->bodyBeginEvents.data,
		.bodyEndEvents = f->bodyEndEvents.data,
		.sensorBeginEvents = f->sensorBeginEvents.data,
		.sensorEndEvents = f->sensorEndEvents.data,
		.bodyBeginCount = f->bodyBeginEvents.count,
		.bodyEndCount = f->bodyEndEvents.count,
		.sensorBeginCount = f->sensorBeginEvents.count,
		.sensorEndCount = f->sensorEndEvents.count,
	};
}

float b2Fluid_GetSensorArea( b2FluidId fluidId, b2ShapeId sensorShapeId )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	for ( int i = 0; i < f->sensors.count; ++i )
	{
		if ( B2_ID_EQUALS( f->sensors.data[i].id, sensorShapeId ) )
		{
			return f->sensors.data[i].area;
		}
	}
	return 0.0f;
}

b2FluidAmount b2Fluid_Measure( b2FluidId fluidId, b2LiquidId liquidId, b2FluidRegion region )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	int liquid = b2fPickLiquid( f, liquidId );
	b2fOutline outline;
	if ( b2fMakeRegionOutline( &region, &outline ) == false )
	{
		return ( b2FluidAmount ){ 0 };
	}

	int count = 0;
	double area = 0.0, mass = 0.0;
	for ( int i = 0; i < f->count; ++i )
	{
		if ( b2fPicks( f, liquid, &outline, i ) == false )
		{
			continue;
		}

		double a = 0.8660254 * f->size[i] * f->size[i];
		count += 1;
		area += a;
		mass += a * f->liquids[f->liquid[i]].def.density;
	}
	return ( b2FluidAmount ){ count, (float)area, (float)mass };
}

bool b2Fluid_GetSurfaceHeight( b2FluidId fluidId, b2Pos point, b2Pos* surface )
{
	b2fFluid* f = b2fGetFluid( fluidId );
	b2Vec2 gravity = f->world->gravity;
	float g = b2Length( gravity );
	b2Vec2 up = g > 0.0f ? b2MulSV( -1.0f / g, gravity ) : ( b2Vec2 ){ 0.0f, 1.0f };
	float reach = 2.0f * f->def.particleSpacing, spray = 0.4f * f->restDensity;
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

		float height = b2Dot( d, up ) + 0.5f * f->size[i];
		best = height > best ? height : best;
		found = true;
	}

	if ( found )
	{
		*surface = b2OffsetPos( point, b2MulSV( best, up ) );
	}
	return found;
}
