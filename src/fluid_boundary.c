// SPDX-License-Identifier: MIT
// Shapes and bodies near the liquid: wall density, contact moves, collision, reactions on bodies and the pressure
// in contact patches.
#include "fluid_internal.h"

#include "aabb.h"
#include "body.h"
#include "contact.h"
#include "physics_world.h"
#include "shape.h"
#include "solver_set.h"

// The kernels integrated over a solid half plane at distance d from a particle, at the number density of the
// liquid at rest. Particles settle about half a spacing from a surface, which is where a lattice
// of particles would end, so the table carries on the lattice without a seam. At the denser lattice rest density
// is counted from, a wall counted for more than the liquid it stands in for and kept the liquid next to it thin,
// which gave a body lift for more than its own volume, a small body the most. Columns are density,
// near density, and the matching pressure and near pressure weights along the normal.
void b2fBuildWallTable( b2fFluid* f )
{
	float h = f->radius;
	float numberDensity = B2F_REST_FILL / ( 0.8660254f * f->spacing * f->spacing );
	enum { steps = 400 };
	float cell = h / (float)steps, area = 2.0f * cell * cell;
	for ( int i = 0; i <= B2F_WALL_TABLE_SIZE; ++i )
	{
		float d = h * (float)i / (float)B2F_WALL_TABLE_SIZE;
		float density = 0.0f, nearDensity = 0.0f, weight = 0.0f, nearWeight = 0.0f;
		for ( int iy = 0; iy < steps; ++iy )
		{
			float y = d + ( (float)iy + 0.5f ) * cell;
			for ( int ix = 0; ix < steps; ++ix )
			{
				float x = ( (float)ix + 0.5f ) * cell;
				float r2 = x * x + y * y;
				if ( r2 < h * h )
				{
					float r = sqrtf( r2 );
					float q = 1.0f - r / h;
					density += q * q;
					nearDensity += q * q * q;
					weight += q * y / r;
					nearWeight += q * q * y / r;
				}
			}
		}

		float scale = numberDensity * area;
		f->wallTable[i][0] = scale * density;
		f->wallTable[i][1] = scale * nearDensity;
		f->wallTable[i][2] = scale * weight;
		f->wallTable[i][3] = scale * nearWeight;
	}
}

// The fluid's body for a body of the world, by its index in the world, added when the first of its shapes is gathered
static int FindBody( b2fFluid* f, int bodyId )
{
	if ( bodyId < f->bodyOfIndexCapacity )
	{
		int i = f->bodyOfIndex[bodyId];
		if ( 0 <= i && i < f->bodyCount && f->bodies[i].id.index1 - 1 == bodyId )
		{
			return i;
		}
	}
	else
	{
		int capacity = f->bodyOfIndexCapacity == 0 ? 64 : f->bodyOfIndexCapacity;
		while ( capacity <= bodyId )
		{
			capacity *= 2;
		}
		f->bodyOfIndex = b2fGrowArray( f->bodyOfIndex, f->bodyOfIndexCapacity, capacity, sizeof( int ) );
		// New entries name no body, so a lookup never reads an unset one
		for ( int i = f->bodyOfIndexCapacity; i < capacity; ++i )
		{
			f->bodyOfIndex[i] = -1;
		}
		f->bodyOfIndexCapacity = capacity;
	}

	if ( f->bodyCount == f->bodyCapacity )
	{
		int capacity = f->bodyCapacity == 0 ? 32 : 2 * f->bodyCapacity;
		f->bodies = b2fGrowArray( f->bodies, f->bodyCapacity, capacity, sizeof( b2fBody ) );
		f->bodyCapacity = capacity;
	}

	b2World* world = f->world;
	b2Body* body = b2Array_Get( world->bodies, bodyId );
	b2BodyState* state = b2GetBodyState( world, body );
	f->bodies[f->bodyCount] = (b2fBody){
		.id = { bodyId + 1, world->worldId, body->generation },
		.lastVelocity = state != NULL ? state->linearVelocity : b2Vec2_zero,
		.lastAngularVelocity = state != NULL ? state->angularVelocity : 0.0f,
		.adheres = body->type != b2_dynamicBody,
		.isStatic = body->type == b2_staticBody,
	};
	f->bodyOfIndex[bodyId] = f->bodyCount;
	return f->bodyCount++;
}

static bool OverlapShape( int proxyId, uint64_t userData, void* context )
{
	(void)proxyId;
	b2fFluid* f = context;
	b2World* world = f->world;
	const b2Shape* source = b2Array_Get( world->shapes, (int)userData );

	// With several queries, a shape an earlier one took is not taken again
	if ( f->gatherMarking )
	{
		int index = (int)userData;
		if ( index >= f->gatherMarkCapacity )
		{
			int capacity = f->gatherMarkCapacity == 0 ? 64 : f->gatherMarkCapacity;
			while ( capacity <= index )
			{
				capacity *= 2;
			}
			f->gatherMark = b2fGrowArray( f->gatherMark, f->gatherMarkCapacity, capacity, sizeof( int ) );
			memset( f->gatherMark + f->gatherMarkCapacity, 0, (size_t)( capacity - f->gatherMarkCapacity ) * sizeof( int ) );
			f->gatherMarkCapacity = capacity;
		}
		if ( f->gatherMark[index] == f->gatherSerial )
		{
			return true;
		}
		f->gatherMark[index] = f->gatherSerial;
	}

	// The liquid passes through sensors, which only count the liquid in them once a step
	uint32_t liquids = b2fLiquidsCollidingWith( f, source->filter );
	if ( source->sensorIndex != B2_NULL_INDEX )
	{
		b2fGatherSensor( f, ( b2ShapeId ){ source->id + 1, world->worldId, source->generation }, liquids );
		return true;
	}

	// A shape no liquid collides with is not there for the liquid. Left out here, it is in none of the passes, nor is
	// its body unless another of its shapes is, so with one liquid a filter costs the passes nothing.
	if ( liquids == 0 )
	{
		f->shapesDropped = true;
		return true;
	}

	float area = 0.0f;
	b2Vec2 center = b2Vec2_zero;
	b2fShape shape = {
		.type = source->type,
		.categoryBits = source->filter.categoryBits,
		.maskBits = source->filter.maskBits,
		.liquids = liquids,
	};
	switch ( source->type )
	{
		case b2_circleShape:
		{
			b2Circle c = source->circle;
			shape.p1 = shape.p2 = c.center;
			shape.radius = c.radius;
			area = B2_PI * c.radius * c.radius;
			center = c.center;
		}
		break;

		case b2_capsuleShape:
		{
			b2Capsule c = source->capsule;
			shape.p1 = c.center1;
			shape.p2 = c.center2;
			shape.radius = c.radius;
			area = B2_PI * c.radius * c.radius + 2.0f * c.radius * b2Distance( c.center1, c.center2 );
			center = b2Lerp( c.center1, c.center2, 0.5f );
		}
		break;

		case b2_segmentShape:
			shape.p1 = source->segment.point1;
			shape.p2 = source->segment.point2;
			break;

		case b2_chainSegmentShape:
			shape.p1 = source->chainSegment.segment.point1;
			shape.p2 = source->chainSegment.segment.point2;
			break;

		case b2_polygonShape:
		{
			shape.polygon = source->polygon;
			shape.radius = shape.polygon.radius;
			// Its thickness, the least depth behind any face, radius included
			float thickness = FLT_MAX;
			for ( int e = 0; e < shape.polygon.count; ++e )
			{
				float depth = 0.0f;
				for ( int v = 0; v < shape.polygon.count; ++v )
				{
					float d = b2Dot( shape.polygon.normals[e], b2Sub( shape.polygon.vertices[e], shape.polygon.vertices[v] ) );
					depth = depth > d ? depth : d;
				}
				thickness = thickness < depth ? thickness : depth;
			}
			shape.thin = thickness + 2.0f * shape.radius < f->def.particleSpacing;
			b2MassData mass = b2ComputePolygonMass( &shape.polygon, 1.0f );
			area = mass.mass;
			center = mass.center;
		}
		break;

		default:
			// A shape of a type the liquid does not handle is not there for it, and does not bring in its body
			return true;
	}

	if ( f->shapeCount == f->shapeCapacity )
	{
		int capacity = f->shapeCapacity == 0 ? 32 : 2 * f->shapeCapacity;
		f->shapes = b2fGrowArray( f->shapes, f->shapeCapacity, capacity, sizeof( b2fShape ) );
		f->shapeCapacity = capacity;
	}

	shape.bodyIndex = FindBody( f, source->bodyId );
	f->shapes[f->shapeCount++] = shape;
	b2fBody* body = f->bodies + shape.bodyIndex;
	body->area += area;
	body->areaCenter = b2MulAdd( body->areaCenter, area, center );
	f->shapesFiltered = f->shapesFiltered || liquids != f->liveLiquids;
	return true;
}

static void AddPatch( b2fFluid* f, int bodyIndex, int otherIndex, b2Vec2 normal )
{
	if ( f->patchCount == f->patchCapacity )
	{
		int capacity = f->patchCapacity == 0 ? 32 : 2 * f->patchCapacity;
		f->patches = b2fGrowArray( f->patches, f->patchCapacity, capacity, sizeof( b2fPatch ) );
		f->patchCapacity = capacity;
	}
	f->patches[f->patchCount++] = ( b2fPatch ){ normal.x, normal.y, bodyIndex, otherIndex };
	f->bodies[bodyIndex].patchCount += 1;
}

static b2ShapeProxy ShapeProxy( const b2fShape* s )
{
	if ( s->type == b2_polygonShape )
	{
		return b2MakeProxy( s->polygon.vertices, s->polygon.count, s->polygon.radius );
	}
	b2Vec2 points[2] = { s->p1, s->p2 };
	return b2MakeProxy( points, s->type == b2_circleShape ? 1 : 2, s->radius );
}

// The square a coordinate falls in along one side of a grid of count squares, clamped to the grid. Clamping in float
// before the conversion keeps any coordinate in range, and keeps the order of coordinates, so bounds that overlap
// get squares that overlap.
static inline int BinIndex( b2fCoord v, b2fCoord origin, float inverse, int count )
{
	float t = (float)( v - origin ) * inverse, last = (float)( count - 1 );
	t = t > 0.0f ? ( t < last ? t : last ) : 0.0f;
	return (int)t;
}

static inline int BinOf( const b2fShapeBins* bins, b2fCoord x, b2fCoord y )
{
	return BinIndex( y, bins->y, bins->inverse, bins->height ) * bins->width + BinIndex( x, bins->x, bins->inverse, bins->width );
}

// Sorts the shapes into the bins, whose grid the caller has set, by the bounds in the shapes: every shape, or only
// those of bodies forces do not move. A counting sort over the shapes in order, so each square lists its shapes in
// order.
static void FillBins( b2fFluid* f, b2fShapeBins* bins, bool fixedOnly )
{
	int squares = bins->width * bins->height;
	if ( squares + 1 > bins->startCapacity )
	{
		int capacity = 2 * ( squares + 1 );
		b2Free( bins->start, (size_t)bins->startCapacity * sizeof( int ) );
		bins->start = b2Alloc( (size_t)capacity * sizeof( int ) );
		bins->startCapacity = capacity;
	}
	memset( bins->start, 0, ( (size_t)squares + 1 ) * sizeof( int ) );

	int total = 0;
	for ( int si = 0; si < f->shapeCount; ++si )
	{
		const b2fShape* s = f->shapes + si;
		if ( fixedOnly && f->bodies[s->bodyIndex].adheres == false )
		{
			continue;
		}
		int x0 = BinIndex( s->minX, bins->x, bins->inverse, bins->width ), x1 = BinIndex( s->maxX, bins->x, bins->inverse, bins->width );
		int y0 = BinIndex( s->minY, bins->y, bins->inverse, bins->height ), y1 = BinIndex( s->maxY, bins->y, bins->inverse, bins->height );
		for ( int y = y0; y <= y1; ++y )
		{
			for ( int x = x0; x <= x1; ++x )
			{
				bins->start[y * bins->width + x + 1] += 1;
			}
		}
		total += ( x1 - x0 + 1 ) * ( y1 - y0 + 1 );
	}

	if ( total > bins->shapeCapacity )
	{
		int capacity = 2 * total;
		b2Free( bins->shapes, (size_t)bins->shapeCapacity * sizeof( int ) );
		bins->shapes = b2Alloc( (size_t)capacity * sizeof( int ) );
		bins->shapeCapacity = capacity;
	}

	for ( int q = 0; q < squares; ++q )
	{
		bins->start[q + 1] += bins->start[q];
	}
	for ( int si = 0; si < f->shapeCount; ++si )
	{
		const b2fShape* s = f->shapes + si;
		if ( fixedOnly && f->bodies[s->bodyIndex].adheres == false )
		{
			continue;
		}
		int x0 = BinIndex( s->minX, bins->x, bins->inverse, bins->width ), x1 = BinIndex( s->maxX, bins->x, bins->inverse, bins->width );
		int y0 = BinIndex( s->minY, bins->y, bins->inverse, bins->height ), y1 = BinIndex( s->maxY, bins->y, bins->inverse, bins->height );
		for ( int y = y0; y <= y1; ++y )
		{
			for ( int x = x0; x <= x1; ++x )
			{
				bins->shapes[bins->start[y * bins->width + x]++] = si;
			}
		}
	}
	for ( int q = squares; q > 0; --q )
	{
		bins->start[q] = bins->start[q - 1];
	}
	bins->start[0] = 0;
}

// Bins the shapes into the blocks of this substep's grid, once the particles are sorted and the shapes' bounds set.
// Blocks of 1 or 2 cells a side measured no faster in the Tank, the Boat or 100 crates over a chain of 500 segments.
static void BinShapes( b2fFluid* f )
{
	b2fShapeBins* bins = &f->shapeBins;
	bins->x = f->gridX;
	bins->y = f->gridY;
	bins->inverse = 1.0f / ( B2F_BLOCK_CELLS * f->gridCell );
	bins->width = ( f->gridWidth + B2F_BLOCK_CELLS - 1 ) / B2F_BLOCK_CELLS;
	bins->height = ( f->gridHeight + B2F_BLOCK_CELLS - 1 ) / B2F_BLOCK_CELLS;
	FillBins( f, bins, false );
}

static int CompareInts( const void* a, const void* b )
{
	int x = *(const int*)a, y = *(const int*)b;
	return ( x > y ) - ( x < y );
}

// A shape's bounds in the world with its body at origin turned by rotation, grown by grow. With a body, its lowest and
// highest points along up, radius included, are widened to take the shape's.
static void SetBounds( b2fShape* s, b2Pos origin, b2Rot rotation, float grow, b2fBody* body, b2Vec2 up )
{
	b2fCoord minX = FLT_MAX, minY = FLT_MAX, maxX = -FLT_MAX, maxY = -FLT_MAX;
	int count = s->type == b2_polygonShape ? s->polygon.count : 2;
	for ( int j = 0; j < count; ++j )
	{
		b2Vec2 local = s->type == b2_polygonShape ? s->polygon.vertices[j] : ( j == 0 ? s->p1 : s->p2 );
		b2Pos p = b2OffsetPos( origin, b2RotateVector( rotation, local ) );
		minX = minX < p.x ? minX : p.x;
		minY = minY < p.y ? minY : p.y;
		maxX = maxX > p.x ? maxX : p.x;
		maxY = maxY > p.y ? maxY : p.y;
		if ( body != NULL )
		{
			float height = b2Dot( b2SubPos( p, body->center ), up );
			body->low = body->low < height - s->radius ? body->low : height - s->radius;
			body->high = body->high > height + s->radius ? body->high : height + s->radius;
		}
	}
	s->minX = minX - grow;
	s->minY = minY - grow;
	s->maxX = maxX + grow;
	s->maxY = maxY + grow;
}

// Bins the shapes of bodies forces do not move, by their bounds with the bodies where they are at the start of the
// step, into squares about as many as the shapes. Each body's shapes are listed in order too. The shapes' bounds
// are those until b2fUpdateBodies sets them for the first substep. Returns how many shapes were binned.
static int BinFixedShapes( b2fFluid* f, float range )
{
	if ( f->shapeCount > f->markCapacity )
	{
		size_t old = (size_t)f->markCapacity * sizeof( int ), size = (size_t)f->shapeCapacity * sizeof( int );
		b2Free( f->bodyShapes, old );
		b2Free( f->shapeMark, old );
		b2Free( f->candidates, old );
		f->bodyShapes = b2Alloc( size );
		f->shapeMark = b2Alloc( size );
		f->candidates = b2Alloc( size );
		f->markCapacity = f->shapeCapacity;
	}
	memset( f->shapeMark, 0, (size_t)f->shapeCount * sizeof( int ) );

	for ( int b = 0; b < f->bodyCount; ++b )
	{
		b2Body* body = b2Array_Get( f->world->bodies, f->bodies[b].id.index1 - 1 );
		f->bodies[b].transform = b2GetBodyTransformQuick( f->world, body );
		f->bodies[b].shapeCount = 0;
	}

	int count = 0;
	b2fCoord minX = FLT_MAX, minY = FLT_MAX, maxX = -FLT_MAX, maxY = -FLT_MAX;
	for ( int si = 0; si < f->shapeCount; ++si )
	{
		b2fShape* s = f->shapes + si;
		b2fBody* body = f->bodies + s->bodyIndex;
		SetBounds( s, body->transform.p, body->transform.q, s->radius, NULL, b2Vec2_zero );
		body->shapeCount += 1;
		if ( body->adheres )
		{
			minX = minX < s->minX ? minX : s->minX;
			minY = minY < s->minY ? minY : s->minY;
			maxX = maxX > s->maxX ? maxX : s->maxX;
			maxY = maxY > s->maxY ? maxY : s->maxY;
			count += 1;
		}
	}

	int first = 0;
	for ( int b = 0; b < f->bodyCount; ++b )
	{
		f->bodies[b].firstShape = first;
		first += f->bodies[b].shapeCount;
		f->bodies[b].shapeCount = 0;
	}
	for ( int si = 0; si < f->shapeCount; ++si )
	{
		b2fBody* body = f->bodies + f->shapes[si].bodyIndex;
		f->bodyShapes[body->firstShape + body->shapeCount++] = si;
	}

	if ( count == 0 )
	{
		return 0;
	}

	// Squares no smaller than the gap, and few enough that there are at most about three for each shape: the first
	// bound keeps the area to one square a shape, the second the squares along a long thin strip
	float width = (float)( maxX - minX ), height = (float)( maxY - minY );
	float side = sqrtf( width * height / (float)count );
	float along = ( width > height ? width : height ) / (float)count;
	side = side > along ? side : along;
	side = side > range ? side : range;
	b2fShapeBins* bins = &f->fixedBins;
	bins->x = minX;
	bins->y = minY;
	bins->inverse = 1.0f / side;
	bins->width = (int)( width / side ) + 1;
	bins->height = (int)( height / side ) + 1;
	FillBins( f, bins, true );
	return count;
}

// What each moving body touches or nearly touches, found once a step from where the bodies are then. Box2D's own
// contacts end 2 cm apart, well short of the gap a particle fits in, so a body lifting off had nothing under it
// in between. Only the shapes forces do not move whose bounds come within the gap of a shape's are measured, in
// their order, which is the order a test of every pair found them in.
static void FindPatches( b2fFluid* f )
{
	float range = B2F_PATCH_GAP * f->spacing;
	f->patchCount = 0;
	int fixedCount = BinFixedShapes( f, range );
	const b2fShapeBins* bins = &f->fixedBins;

	// A little past the gap, so that rounding in the bounds never leaves out a pair the distance would keep
	float reach = 1.1f * range;
	int query = 0;
	for ( int bodyIndex = 0; bodyIndex < f->bodyCount; ++bodyIndex )
	{
		b2fBody* body = f->bodies + bodyIndex;
		body->firstPatch = f->patchCount;
		body->patchCount = 0;
		const b2Body* source = b2Array_Get( f->world->bodies, body->id.index1 - 1 );
		if ( body->adheres || source->setIndex != b2_awakeSet )
		{
			continue;
		}

		b2WorldTransform transform = body->transform;
		for ( int k = 0; k < body->shapeCount && fixedCount > 0; ++k )
		{
			const b2fShape* shape = f->shapes + f->bodyShapes[body->firstShape + k];
			b2fCoord minX = shape->minX - reach, minY = shape->minY - reach, maxX = shape->maxX + reach, maxY = shape->maxY + reach;
			int x0 = BinIndex( minX, bins->x, bins->inverse, bins->width ), x1 = BinIndex( maxX, bins->x, bins->inverse, bins->width );
			int y0 = BinIndex( minY, bins->y, bins->inverse, bins->height ), y1 = BinIndex( maxY, bins->y, bins->inverse, bins->height );

			// Each shape once, though it may be in several squares, and in order
			query += 1;
			int candidateCount = 0;
			for ( int y = y0; y <= y1; ++y )
			{
				for ( int x = x0; x <= x1; ++x )
				{
					int square = y * bins->width + x;
					for ( int e = bins->start[square]; e < bins->start[square + 1]; ++e )
					{
						int j = bins->shapes[e];
						const b2fShape* other = f->shapes + j;
						if ( f->shapeMark[j] == query )
						{
							continue;
						}
						f->shapeMark[j] = query;
						if ( other->maxX >= minX && other->minX <= maxX && other->maxY >= minY && other->minY <= maxY )
						{
							f->candidates[candidateCount++] = j;
						}
					}
				}
			}
			qsort( f->candidates, (size_t)candidateCount, sizeof( int ), CompareInts );

			b2ShapeProxy proxy = ShapeProxy( shape );
			for ( int c = 0; c < candidateCount; ++c )
			{
				// Only shapes that forces do not move count from a distance. A wheel turning beside its hull is always
				// close to it, and its thrust is the liquid's push towards the hull, which this would cancel.
				const b2fShape* other = f->shapes + f->candidates[c];
				int otherIndex = other->bodyIndex;
				b2DistanceInput input = { 0 };
				input.proxyA = proxy;
				input.proxyB = ShapeProxy( other );
				input.transform = b2InvMulWorldTransforms( transform, f->bodies[otherIndex].transform );
				input.useRadii = true;
				b2SimplexCache cache = { 0 };
				b2DistanceOutput output = b2ComputeShapeDistance( &input, &cache, NULL, 0 );
				if ( output.distance < range && b2LengthSquared( output.normal ) > 0.5f )
				{
					AddPatch( f, bodyIndex, otherIndex, b2Neg( b2RotateVector( transform.q, output.normal ) ) );
				}
			}
		}

		// Shapes pressed into each other have no direction between them, so those come from Box2D's contacts, and so
		// do moving bodies that touch, which bodies joined so as not to collide never do
		// Every touching contact in the body's list
		b2World* world = f->world;
		int contactKey = source->headContactKey;
		while ( contactKey != B2_NULL_INDEX )
		{
			b2Contact* contact = b2Array_Get( world->contacts, contactKey >> 1 );
			contactKey = contact->edges[contactKey & 1].nextKey;
			if ( ( contact->flags & b2_contactTouchingFlag ) == 0 )
			{
				continue;
			}

			// Liquid fills the space of a shape it passes through, so a touch there leaves no gap it misses
			const b2Shape* shapeA = b2Array_Get( world->shapes, contact->shapeIdA );
			const b2Shape* shapeB = b2Array_Get( world->shapes, contact->shapeIdB );
			if ( f->shapesDropped &&
				 ( b2fLiquidsCollidingWith( f, shapeA->filter ) == 0 || b2fLiquidsCollidingWith( f, shapeB->filter ) == 0 ) )
			{
				continue;
			}

			// The other body counts only if some shape of it was gathered this step
			bool first = shapeA->bodyId == body->id.index1 - 1;
			int otherId = first ? shapeB->bodyId : shapeA->bodyId;
			int otherIndex = otherId < f->bodyOfIndexCapacity ? f->bodyOfIndex[otherId] : -1;
			if ( otherIndex < 0 || otherIndex >= f->bodyCount || f->bodies[otherIndex].id.index1 - 1 != otherId )
			{
				continue;
			}

			const b2Manifold* manifold = &b2GetContactSim( world, contact )->manifold;
			if ( manifold->pointCount > 0 && ( f->bodies[otherIndex].adheres == false || manifold->points[0].separation <= 0.0f ) )
			{
				AddPatch( f, bodyIndex, otherIndex, first ? b2Neg( manifold->normal ) : manifold->normal );
			}
		}
	}
}

// A particle outside the bulk, by the square of the grid outside it that it is in
typedef struct b2fSquare
{
	int64_t y, x;
	b2fCoord px, py;
} b2fSquare;

static int CompareSquares( const void* a, const void* b )
{
	const b2fSquare *p = a, *q = b;
	if ( p->y != q->y )
	{
		return p->y < q->y ? -1 : 1;
	}
	if ( p->x != q->x )
	{
		return p->x < q->x ? -1 : 1;
	}
	if ( p->py != q->py )
	{
		return p->py < q->py ? -1 : 1;
	}
	return ( p->px > q->px ) - ( p->px < q->px );
}

static int64_t SquareOf( b2fCoord v, b2fCoord origin, double side )
{
	double t = floor( (double)( v - origin ) / side ), most = 1099511627776.0;
	return (int64_t)( t < -most ? -most : ( t > most ? most : t ) );
}

// Boxes to gather shapes in when liquid lies outside the bulk: the bulk grown by grow, then for each square four times
// grow a side, in order along rows, the bounds of the particles outside the bulk in it, grown the same. Every particle
// is in one of them. Fewer boxes than particles; the caller frees room for as many as there are particles.
static b2AABB* GatherBoxes( const b2fFluid* f, const b2fBox* bulk, float grow, int* boxCount )
{
	b2AABB* boxes = b2Alloc( (size_t)f->count * sizeof( b2AABB ) );
	b2Pos origin = { bulk->minX, bulk->minY };
	b2AABB aabb = { { -grow, -grow }, { (float)( bulk->maxX - bulk->minX ) + grow, (float)( bulk->maxY - bulk->minY ) + grow } };
	boxes[0] = b2OffsetAABB( aabb, origin );
	int count = 1;

	int outside = 0;
	for ( int i = 0; i < f->count; ++i )
	{
		b2Pos p = f->pos[i];
		outside += p.x < bulk->minX || p.x > bulk->maxX || p.y < bulk->minY || p.y > bulk->maxY ? 1 : 0;
	}
	outside = outside > 0 ? outside : 1;
	b2fSquare* squares = b2Alloc( (size_t)outside * sizeof( b2fSquare ) );
	double side = 4.0 * (double)grow;
	int k = 0;
	for ( int i = 0; i < f->count; ++i )
	{
		b2Pos p = f->pos[i];
		if ( p.x < bulk->minX || p.x > bulk->maxX || p.y < bulk->minY || p.y > bulk->maxY )
		{
			squares[k++] = ( b2fSquare ){ SquareOf( p.y, bulk->minY, side ), SquareOf( p.x, bulk->minX, side ), p.x, p.y };
		}
	}
	qsort( squares, (size_t)k, sizeof( b2fSquare ), CompareSquares );

	for ( int first = 0; first < k; )
	{
		b2fBox box = { squares[first].px, squares[first].py, squares[first].px, squares[first].py };
		int next = first + 1;
		while ( next < k && squares[next].y == squares[first].y && squares[next].x == squares[first].x )
		{
			box.minX = box.minX < squares[next].px ? box.minX : squares[next].px;
			box.maxX = box.maxX > squares[next].px ? box.maxX : squares[next].px;
			box.minY = box.minY < squares[next].py ? box.minY : squares[next].py;
			box.maxY = box.maxY > squares[next].py ? box.maxY : squares[next].py;
			next += 1;
		}
		b2Pos corner = { box.minX, box.minY };
		b2AABB around = { { -grow, -grow }, { (float)( box.maxX - box.minX ) + grow, (float)( box.maxY - box.minY ) + grow } };
		boxes[count++] = b2OffsetAABB( around, corner );
		first = next;
	}
	b2Free( squares, (size_t)outside * sizeof( b2fSquare ) );
	*boxCount = count;
	return boxes;
}

// Shapes near the liquid, gathered once a step
void b2fGatherShapes( b2fFluid* f )
{
	f->shapeCount = 0;
	f->bodyCount = 0;
	f->shapesFiltered = false;
	f->shapesDropped = false;
	for ( int i = 0; i < f->sensorCount; ++i )
	{
		f->sensors[i].gathered = false;
	}
	if ( f->count > 0 )
	{
		float maxSpeed2 = 0.0f;
		for ( int i = 0; i < f->count; ++i )
		{
			float speed2 = f->vx[i] * f->vx[i] + f->vy[i] * f->vy[i];
			maxSpeed2 = maxSpeed2 > speed2 ? maxSpeed2 : speed2;
		}

		// The reach of the coarsest particle, and a generous margin, since bodies and particles both move during the
		// step: a metre in the caller's units
		float grow = f->maxRadius + 0.1f * sqrtf( maxSpeed2 ) + b2_lengthUnitsPerMeter;

		// One box over all the liquid, or with liquid outside the bulk the grid would cover, one over the bulk and one
		// over the liquid in each square outside it that has some, so the shapes between are not gathered
		b2fBox all, bulk;
		int border = f->adaptive ? (int)ceilf( f->maxRadius / f->cellSize ) + 1 : 1;
		bool bulkOnly = b2fFindBulk( f, f->cellSize, border, &all, &bulk );
		int boxCount = 1;
		b2AABB single;
		b2AABB* boxes = &single;
		if ( bulkOnly == false )
		{
			b2Pos origin = { all.minX, all.minY };
			b2AABB aabb = { { -grow, -grow }, { (float)( all.maxX - all.minX ) + grow, (float)( all.maxY - all.minY ) + grow } };
			boxes[0] = b2OffsetAABB( aabb, origin );
		}
		else
		{
			boxes = GatherBoxes( f, &bulk, grow, &boxCount );
		}

		// Every shape, in the order b2World_OverlapAABB would give them: the trees of static, kinematic and dynamic
		// bodies in turn, and within each, the boxes in turn. Shapes are left out by the liquids' filters alone, in
		// OverlapShape.
		f->gatherMarking = boxCount > 1;
		f->gatherSerial += f->gatherMarking ? 1 : 0;
		for ( int i = 0; i < b2_bodyTypeCount; ++i )
		{
			for ( int k = 0; k < boxCount; ++k )
			{
				b2DynamicTree_OverlapAABB( f->world->broadPhase.trees + i, boxes[k], B2_DEFAULT_MASK_BITS, OverlapShape, f );
			}
		}
		f->gatherMarking = false;
		if ( bulkOnly )
		{
			b2Free( boxes, (size_t)f->count * sizeof( b2AABB ) );
		}
	}

	int needed = f->bodyCount * B2F_CHUNK_COUNT;
	if ( needed > f->reactionCapacity )
	{
		f->reactions = b2fGrowArray( f->reactions, f->reactionCapacity, 2 * needed, sizeof( b2fReaction ) );
		f->reactionCapacity = 2 * needed;
	}

	FindPatches( f );
}

// A sleeping body cannot feel the liquid, so it would never lift off. The particles still find it, as a wall, and a
// body the liquid reached during the step wakes now, after it, with the bodies it rests on or against. It held still
// for that one step. Awake bodies with liquid in reach did not fall asleep (RecordReactions), so these are bodies
// that slept before the liquid came; one the liquid never reaches may sleep, however near the liquid it is.
void b2fWakeReached( b2fFluid* f )
{
	for ( int i = 0; i < f->bodyCount; ++i )
	{
		const b2fBody* body = f->bodies + i;
		if ( body->reached && body->adheres == false )
		{
			b2WakeBody( f->world, b2Array_Get( f->world->bodies, body->id.index1 - 1 ) );
		}
	}
}

static inline void SegmentDistance( b2Vec2 p, b2Vec2 a, b2Vec2 b, float* distance, b2Vec2* normal )
{
	b2Vec2 d = b2Sub( b, a );
	float denominator = b2Dot( d, d );
	float t = denominator > 0.0f ? b2ClampFloat( b2Dot( b2Sub( p, a ), d ) / denominator, 0.0f, 1.0f ) : 0.0f;
	b2Vec2 r = b2Sub( p, b2MulAdd( a, t, d ) );
	float length = b2Length( r );
	*distance = length;
	*normal = length > 1e-7f ? b2MulSV( 1.0f / length, r ) : b2LeftPerp( b2Normalize( d ) );
}

// Signed distance from a local point, positive outside, with the outward normal.
bool b2fShapeDistance( const b2fShape* s, b2Vec2 p, float* distance, b2Vec2* normal )
{
	switch ( s->type )
	{
		case b2_polygonShape:
		{
			const b2Polygon* polygon = &s->polygon;
			float best = -FLT_MAX;
			int bestIndex = 0;
			for ( int i = 0; i < polygon->count; ++i )
			{
				float separation = b2Dot( polygon->normals[i], b2Sub( p, polygon->vertices[i] ) );
				if ( separation > best )
				{
					best = separation;
					bestIndex = i;
				}
			}

			if ( best <= 0.0f )
			{
				*distance = best - polygon->radius;
				*normal = polygon->normals[bestIndex];
				return true;
			}

			float closest = FLT_MAX;
			for ( int i = 0; i < polygon->count; ++i )
			{
				int next = i + 1 < polygon->count ? i + 1 : 0;
				float d;
				b2Vec2 n;
				SegmentDistance( p, polygon->vertices[i], polygon->vertices[next], &d, &n );
				if ( d < closest )
				{
					closest = d;
					*normal = n;
				}
			}
			*distance = closest - polygon->radius;
			return true;
		}

		case b2_chainSegmentShape:
		{
			// Chains collide on their right side only
			b2Vec2 e = b2Sub( s->p2, s->p1 );
			if ( b2Cross( b2Sub( p, s->p1 ), e ) < 0.0f )
			{
				return false;
			}
			SegmentDistance( p, s->p1, s->p2, distance, normal );
			return true;
		}

		default:
			SegmentDistance( p, s->p1, s->p2, distance, normal );
			*distance -= s->radius;
			return true;
	}
}

// Whether a point lies on or in a shape, to within a tolerance. A chain segment has no inside, so a point just behind
// one is on it too.
static bool Buries( const b2fFluid* f, const b2fShape* s, b2Pos point, float tolerance )
{
	const b2fBody* body = f->bodies + s->bodyIndex;
	b2Vec2 local = b2InvRotateVector( body->motion.rotation, b2SubPos( point, body->origin ) );
	float distance;
	b2Vec2 normal;
	if ( s->type == b2_chainSegmentShape )
	{
		SegmentDistance( local, s->p1, s->p2, &distance, &normal );
		return distance < tolerance;
	}
	return b2fShapeDistance( s, local, &distance, &normal ) && distance < tolerance;
}

// Each shape in reach counts as a solid half plane through its nearest point. Where shapes held fixed against each
// other meet, as the tiles of a floor or the segments of a chain do, the nearest point of one can be a corner that
// lies on the other: its half plane then counts again solid the other already covers, and pushes along the corner's
// normal, sideways across the seam. Such a contact is left out. Shapes of different moving bodies always count, since
// they part again.
static inline bool FixedTogether( const b2fFluid* f, const b2fShape* a, const b2fShape* b )
{
	return a->bodyIndex == b->bodyIndex || ( f->bodies[a->bodyIndex].isStatic && f->bodies[b->bodyIndex].isStatic );
}

static void ContactTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	bool filtered = f->shapesFiltered;
	const b2fShapeBins* bins = &f->shapeBins;
	for ( int i = start; i < end; ++i )
	{
		float lattice = b2fLatticeSize( f->size[i] ), h = 2.0f * lattice, tableScale = (float)B2F_WALL_TABLE_SIZE / h;
		float tolerance = B2F_SEAM_TOLERANCE * lattice;
		b2fCoord x = f->pos[i].x, y = f->pos[i].y;
		// Every shape in reach, each with its nearest point
		struct
		{
			const b2fShape* shape;
			float distance;
			b2Vec2 normal;
			b2Pos point;
		} found[2 * B2F_CONTACTS_PER_PARTICLE];
		int foundCount = 0;
		int bin = BinOf( bins, x, y );
		for ( int k = bins->start[bin], binEnd = bins->start[bin + 1]; k < binEnd && foundCount < 2 * B2F_CONTACTS_PER_PARTICLE; ++k )
		{
			const b2fShape* s = f->shapes + bins->shapes[k];
			if ( x < s->minX || x > s->maxX || y < s->minY || y > s->maxY )
			{
				continue;
			}

			// A shape this particle's liquid passes through makes no contact, so wall density, the wall moves,
			// adhesion, the reaction on the body, added mass, patches and the body's state never see the pair
			if ( filtered && ( s->liquids >> f->liquid[i] & 1u ) == 0 )
			{
				continue;
			}

			const b2fBody* body = f->bodies + s->bodyIndex;
			b2Vec2 local = b2InvRotateVector( body->motion.rotation, b2SubPos( ( b2Pos ){ x, y }, body->origin ) );
			float distance;
			b2Vec2 normal;
			if ( b2fShapeDistance( s, local, &distance, &normal ) == false || distance >= h )
			{
				continue;
			}

			b2Vec2 n = b2RotateVector( body->motion.rotation, normal );
			found[foundCount].shape = s;
			found[foundCount].distance = distance;
			found[foundCount].normal = n;
			found[foundCount].point = ( b2Pos ){ x - distance * n.x, y - distance * n.y };
			foundCount += 1;
		}

		// A shape whose nearest point is buried in another shape in reach that is at least as close is left out. That
		// other shape counts whether or not it is left out itself: along a curved chain the nearest point of a segment
		// two away is the end of the one between. Where two nearest points each lie on the other shape, as near the
		// vertex between two segments of a curve, the closer stays: a face rather than the corner beside it, whose
		// normal leans along the surface. Of two equally close, as over the middle of a seam, the first found stays.
		// Choosing by the order shapes were found leaned the push and the adhesion one way along a chain at every
		// vertex, which kept the liquid in a bowl circling.
		int kept[B2F_CONTACTS_PER_PARTICLE];
		int count = 0;
		for ( int c = 0; c < foundCount && count < B2F_CONTACTS_PER_PARTICLE; ++c )
		{
			bool buried = false;
			for ( int e = 0; e < foundCount && buried == false; ++e )
			{
				bool closer = found[e].distance < found[c].distance || ( found[e].distance == found[c].distance && e < c );
				buried = e != c && closer && FixedTogether( f, found[c].shape, found[e].shape ) && Buries( f, found[e].shape, found[c].point, tolerance );
			}
			if ( buried == false )
			{
				kept[count++] = c;
			}
		}

		float density = 0.0f, nearDensity = 0.0f;
		b2fContact* contacts = f->contacts + B2F_CONTACTS_PER_PARTICLE * i;
		for ( int c = 0; c < count; ++c )
		{
			int e = kept[c];
			float distance = found[e].distance > 0.0f ? found[e].distance : 0.0f;
			float t = distance * tableScale;
			int index = (int)t;
			index = index < B2F_WALL_TABLE_SIZE ? index : B2F_WALL_TABLE_SIZE - 1;
			float blend = t - (float)index;
			const float* a = f->wallTable[index];
			const float* b = f->wallTable[index + 1];
			density += a[0] + blend * ( b[0] - a[0] );
			nearDensity += a[1] + blend * ( b[1] - a[1] );
			float share = 1.0f - distance / h;
			contacts[c] = (b2fContact){ found[e].shape->bodyIndex, found[e].normal.x, found[e].normal.y, a[2] + blend * ( b[2] - a[2] ), a[3] + blend * ( b[3] - a[3] ), 0.0f, share };
		}
		f->contactCount[i] = count;
		f->wallDensity[i] = density;
		f->wallNearDensity[i] = nearDensity;
	}
}

// Reactions go to a chunk chosen by particle index, and chunks sum in order, so the total does not
// depend on which worker ran which chunk.
// A particle near two shapes of one body counts once for it, as close as its nearest shape. Zero for any but the
// first contact with each body.
static inline float ContactShare( const b2fContact* contacts, int count, int c )
{
	int bodyIndex = contacts[c].bodyIndex;
	for ( int e = 0; e < c; ++e )
	{
		if ( contacts[e].bodyIndex == bodyIndex )
		{
			return 0.0f;
		}
	}

	float share = contacts[c].share;
	for ( int e = c + 1; e < count; ++e )
	{
		if ( contacts[e].bodyIndex == bodyIndex && contacts[e].share > share )
		{
			share = contacts[e].share;
		}
	}
	return share;
}

static void CollideTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	float h = f->h, inv = 1.0f / h, fineSize = f->def.particleSpacing;
	bool adaptive = f->adaptive, filtered = f->shapesFiltered;
	float g = b2Length( f->worldGravity );
	b2Vec2 up = g > 0.0f ? b2MulSV( -1.0f / g, f->worldGravity ) : b2Vec2_zero;
	float spray = 0.4f * f->restDensity;
	const b2fShapeBins* bins = &f->shapeBins;

	for ( int chunk = start; chunk < end; ++chunk )
	{
		b2fReaction* reactions = f->reactions + chunk * f->bodyCount;
		memset( reactions, 0, (size_t)f->bodyCount * sizeof( b2fReaction ) );

		int first = (int)( (int64_t)f->count * chunk / B2F_CHUNK_COUNT );
		int last = (int)( (int64_t)f->count * ( chunk + 1 ) / B2F_CHUNK_COUNT );
		for ( int i = first; i < last; ++i )
		{
			// The particle's mass goes with its area. What a patch needs of the liquid around its body is the density
			// its buoyancy goes by, and how high the liquid stands on the body, against gravity, which is also the share
			// under that the caller reads.
			const b2fLiquid* liquid = f->liquids + f->liquid[i];
			const b2LiquidDef* def = &liquid->def;
			float ratio = f->size[i] / fineSize, m = liquid->mass * ratio * ratio, margin = 0.5f * b2fLatticeSize( f->size[i] );
			float friction = def->friction, added = def->tuning.addedMass;
			float buoyantDensity = def->tuning.buoyancyAssist * def->density * def->gravityScale;

			const b2fContact* contacts = f->contacts + B2F_CONTACTS_PER_PARTICLE * i;
			for ( int c = 0; c < f->contactCount[i]; ++c )
			{
				const b2fBody* body = f->bodies + contacts[c].bodyIndex;
				if ( body->awake )
				{
					float impulse = m * contacts[c].move * inv;
					float impulseX = -impulse * contacts[c].normalX;
					float impulseY = -impulse * contacts[c].normalY;
					float rx = (float)( f->pos[i].x - body->center.x ), ry = (float)( f->pos[i].y - body->center.y );
					b2fReaction* reaction = reactions + contacts[c].bodyIndex;
					reaction->x += impulseX;
					reaction->y += impulseY;
					reaction->angular += rx * impulseY - ry * impulseX;
					float share = ContactShare( contacts, f->contactCount[i], c );
					if ( share > 0.0f )
					{
						reaction->wetMass += m;
						reaction->wetDensity += m * buoyantDensity;

						// Only the row against the body: counting every particle within reach made a face with two
						// rows beside it weigh twice one with a thin film
						if ( share > 0.5f )
						{
							reaction->wetLength += f->size[i];
							reaction->wetNormalX += f->size[i] * contacts[c].normalX;
							reaction->wetNormalY += f->size[i] * contacts[c].normalY;
						}

						// The level of the liquid on the body. Liquid lying on an upward face, such as a film or a
						// splash on top of a floating body, is not the level.
						float facing = contacts[c].normalX * up.x + contacts[c].normalY * up.y;
						if ( f->rho[i] >= spray && facing < B2F_TOP_FACING )
						{
							float top = rx * up.x + ry * up.y + 0.5f * f->size[i] - body->low;
							reaction->wetTop = top > reaction->wetTop ? top : reaction->wetTop;
						}
					}

					share = added > 0.0f && body->motion.invMass > 0.0f ? share : 0.0f;
					if ( share > 0.0f )
					{
						float lump = m * added * share;
						reaction->mass += lump;
						reaction->momentX += lump * rx;
						reaction->momentY += lump * ry;
						reaction->inertia += lump * ( rx * rx + ry * ry );
					}
				}
				else if ( body->adheres == false )
				{
					// A sleeping body feels nothing, but the liquid within its reach wakes it next step
					reactions[contacts[c].bodyIndex].wetMass += m;
				}
			}

			// The shapes whose bounds could hold the particle, in order. A push that carries it into another square
			// leaves it on this square's list. Pushes are short next to a square, and in the test scenes none brought a
			// particle into the bounds of a shape missing from the list.
			int bin = BinOf( bins, f->pos[i].x, f->pos[i].y );
			for ( int k = bins->start[bin], binEnd = bins->start[bin + 1]; k < binEnd; ++k )
			{
				int si = bins->shapes[k];
				const b2fShape* s = f->shapes + si;
				b2fCoord x = f->pos[i].x, y = f->pos[i].y;
				if ( x < s->minX || x > s->maxX || y < s->minY || y > s->maxY )
				{
					continue;
				}

				// The push out of the shape, friction and their reaction on the body find shapes themselves rather
				// than from the contacts, so they skip a shape the particle's liquid passes through here
				if ( filtered && ( s->liquids >> f->liquid[i] & 1u ) == 0 )
				{
					continue;
				}

				const b2fBody* body = f->bodies + s->bodyIndex;
				b2Pos world = { x, y };
				b2Vec2 local = b2InvRotateVector( body->motion.rotation, b2SubPos( world, body->origin ) );
				float distance = 0.0f;
				b2Vec2 normal = b2Vec2_zero;
				bool crossed = false;

				// A small particle keeps only half its size from a surface, so a fast one can pass a segment or a thin
				// capsule within one substep and come out behind it, where the push would carry it on through. If its
				// path this substep cuts the segment, or the capsule's core, it goes back to the side it came from. A
				// chain segment collides on its right side only, so only a path from there counts.
				if ( s->type == b2_segmentShape || s->type == b2_chainSegmentShape || s->type == b2_capsuleShape )
				{
					b2Vec2 before = b2InvRotateVector( body->motion.rotation, b2SubPos( f->prev[i], body->origin ) );
					b2Vec2 edge = b2Sub( s->p2, s->p1 );
					float sideBefore = b2Cross( b2Sub( before, s->p1 ), edge ), sideNow = b2Cross( b2Sub( local, s->p1 ), edge );
					float length2 = b2Dot( edge, edge );
					bool cuts = s->type == b2_chainSegmentShape ? sideBefore >= 0.0f && sideNow < 0.0f
																: ( sideBefore > 0.0f && sideNow < 0.0f ) || ( sideBefore < 0.0f && sideNow > 0.0f );
					if ( cuts && length2 > 0.0f )
					{
						b2Vec2 cut = b2Lerp( before, local, sideBefore / ( sideBefore - sideNow ) );
						float along = b2Dot( b2Sub( cut, s->p1 ), edge ) / length2;
						if ( along >= 0.0f && along <= 1.0f )
						{
							// Signed along the normal on the side the particle came from, so negative
							float sign = sideBefore >= 0.0f ? 1.0f : -1.0f, length = sqrtf( length2 );
							distance = sign * sideNow / length - s->radius;
							normal = b2MulSV( sign / length, b2RightPerp( edge ) );
							crossed = true;
						}
					}
				}

				// A polygon thinner than a particle spacing: a fast particle can land past its middle, where the
				// nearest face is the far one, or pass it whole. If its path this substep entered through a face other
				// than the one the distance finds, or the distance finds none, it goes back out of the face it came in
				// by.
				if ( s->type == b2_polygonShape && s->thin )
				{
					b2Vec2 before = b2InvRotateVector( body->motion.rotation, b2SubPos( f->prev[i], body->origin ) );
					const b2Polygon* polygon = &s->polygon;
					float earliest = 2.0f, entryDistance = 0.0f;
					b2Vec2 entryNormal = b2Vec2_zero;
					for ( int e = 0; e < polygon->count; ++e )
					{
						b2Vec2 v = polygon->vertices[e], n = polygon->normals[e];
						float sideBefore = b2Dot( n, b2Sub( before, v ) ), sideNow = b2Dot( n, b2Sub( local, v ) );
						if ( sideBefore < 0.0f || sideNow >= 0.0f )
						{
							continue;
						}
						float t = sideBefore / ( sideBefore - sideNow );
						b2Vec2 edge = b2Sub( polygon->vertices[e + 1 < polygon->count ? e + 1 : 0], v );
						float along = b2Dot( b2Sub( b2Lerp( before, local, t ), v ), edge ) / b2Dot( edge, edge );
						if ( along >= 0.0f && along <= 1.0f && t < earliest )
						{
							earliest = t;
							entryDistance = sideNow - s->radius;
							entryNormal = n;
						}
					}
					if ( earliest <= 1.0f )
					{
						bool found = b2fShapeDistance( s, local, &distance, &normal );
						if ( found == false || distance >= margin || b2Dot( normal, entryNormal ) < 0.999f )
						{
							distance = entryDistance;
							normal = entryNormal;
							crossed = true;
						}
					}
				}

				// Behind a chain segment, which has no inside to push a particle out of
				if ( crossed == false && ( b2fShapeDistance( s, local, &distance, &normal ) == false || distance >= margin ) )
				{
					continue;
				}

				b2Vec2 n = b2RotateVector( body->motion.rotation, normal );
				float push = margin - distance;

				// Sizes can change next to a shape. What a body feels is limited to what a particle could carry in.
				if ( adaptive && crossed == false && push > 2.0f * margin )
				{
					push = 2.0f * margin;
				}
				float moveX = push * n.x, moveY = push * n.y;

				// Wet friction, as in Clavet et al. 2005 section 6.1: a fixed share of the sliding speed along the
				// surface goes, however hard the particle is pressed. Friction that grew with the push held particles
				// pressed under heavy liquid or a heavy body in place, so a film one particle thick never drained.
				b2Vec2 r = b2SubPos( world, body->center );
				float w = body->motion.angularVelocity;
				float bodyVx = body->motion.linearVelocity.x - w * r.y;
				float bodyVy = body->motion.linearVelocity.y + w * r.x;
				float relX = (float)( x + moveX - f->prev[i].x ) * inv - bodyVx;
				float relY = (float)( y + moveY - f->prev[i].y ) * inv - bodyVy;
				float normalSpeed = relX * n.x + relY * n.y;
				float tangentX = relX - normalSpeed * n.x, tangentY = relY - normalSpeed * n.y;
				moveX -= h * friction * tangentX;
				moveY -= h * friction * tangentY;

				f->pos[i].x = x + moveX;
				f->pos[i].y = y + moveY;

				if ( body->awake )
				{
					float impulseX = -m * moveX * inv, impulseY = -m * moveY * inv;
					b2fReaction* reaction = reactions + s->bodyIndex;
					reaction->x += impulseX;
					reaction->y += impulseY;
					reaction->angular += r.x * impulseY - r.y * impulseX;
				}
			}
		}
	}
}

// The open side of a body's outline: the mean outward normal of where liquid is against it, which is nothing with
// liquid all the way round, less the parts facing shapes forces do not move that the body is close to. Up when no
// liquid is against it.
static b2Vec2 OpenSide( const b2fFluid* f, const b2fBody* body, const b2fReaction* total, b2Vec2 up )
{
	b2Vec2 gap = total->wetLength > 0.0f ? b2MulSV( 1.0f / total->wetLength, ( b2Vec2 ){ total->wetNormalX, total->wetNormalY } ) : up;
	for ( int i = body->firstPatch; i < body->firstPatch + body->patchCount; ++i )
	{
		b2Vec2 normal = { f->patches[i].normalX, f->patches[i].normalY };
		float along = b2Dot( gap, normal );
		if ( f->bodies[f->patches[i].otherIndex].adheres && along > 0.0f )
		{
			gap = b2MulSub( gap, along, normal );
		}
	}
	return gap;
}

// The share of the body's height under liquid: the liquid's level on it, its top over the body's height. Twice the
// mean height of the liquid within reach, used before, counted the liquid under a floating body's bottom too and read
// 0.37 for a crate half under.
static float SubmergedShare( const b2fBody* body, const b2fReaction* total )
{
	float height = body->high - body->low;
	if ( height <= 0.0f )
	{
		return 0.0f;
	}
	float share = total->wetTop / height;
	return share > 1.0f ? 1.0f : share;
}

// Liquid cannot fill a gap narrower than a few particles, so a body touching or nearly touching something misses the
// liquid's push on that side. Still liquid pushes on a body with the weight of the liquid it displaces, Archimedes'
// principle, whatever the depth, and does not turn it. So the gap gives what the particles' push falls short of that,
// along the direction away from the other shape: a body lighter than the liquid lifts off, a heavier one stays down,
// and as particles fill the gap their push takes over and this fades. Reading the pressure in the gap from the depth
// instead had to be right to a few percent of that pressure, many times the body's buoyancy in deep liquid, to tell
// the two apart. It never pulls. The share of the body under liquid is the one b2Fluid_GetBodyState
// reports.
static void ApplyPatches( b2fFluid* f, b2fReaction* totals )
{
	float g = b2Length( f->worldGravity );
	if ( g == 0.0f )
	{
		return;
	}
	b2Vec2 up = b2MulSV( -1.0f / g, f->worldGravity );

	for ( int bodyIndex = 0; bodyIndex < f->bodyCount; ++bodyIndex )
	{
		const b2fBody* body = f->bodies + bodyIndex;
		b2fReaction* total = totals + bodyIndex;
		float height = body->high - body->low;
		if ( body->awake == false || body->motion.invMass == 0.0f || total->wetDensity <= 0.0f || height <= 0.0f )
		{
			continue;
		}

		float share = SubmergedShare( body, total );
		float lift = total->wetDensity / total->wetMass * body->area * share * g * f->h;
		float needX = lift * up.x - total->x, needY = lift * up.y - total->y, needAngular = -total->angular;
		float arm = 0.5f * sqrtf( body->area );

		// Settled particles lock together like packed grains and can hold a body whose weight and buoyancy differ by
		// a tenth in place, anywhere in the liquid. A body with liquid all the way round that is nearly still gets
		// its buoyancy exactly, up or down, so it cannot hang there. Once it moves the particles' push takes over,
		// which is what slows it. A body at the surface or with a dry side, such as a hull, is left to the particles.
		// A crate afloat with its top dry has a third of its outline open, and a hull more. A small body has few
		// particles against it, so up to a tenth open still counts as all the way round. The side facing a shape the
		// body is close to is shut, not open: liquid would be there if it fitted.
		float open = b2Length( OpenSide( f, body, total, up ) );
		float round = open < 0.1f ? 1.0f : ( 0.25f - open ) / 0.15f;
		float still = 1.0f - b2Length( body->motion.linearVelocity ) / B2F_STILL_SPEED;
		if ( round > 0.0f && still > 0.0f )
		{
			// With liquid over nine tenths of its height all of it counts as under, since the estimate above is only
			// good to a few percent. A crate standing in shallow liquid has its sides wet and counts for what is under.
			float full = total->wetDensity / total->wetMass * body->area * g * f->h * ( share > 0.9f ? 1.0f : share );
			float impulse = round * still * ( full - total->x * up.x - total->y * up.y );
			total->x += impulse * up.x;
			total->y += impulse * up.y;
			needX -= impulse * up.x;
			needY -= impulse * up.y;

			// Buoyancy acts at the centre of the body's area, and still liquid turns the body no other way. A wedge
			// of locked particles under a heavy crate otherwise propped it up on one edge.
			float turn = round * still * ( full * ( body->buoyancyArm.x * up.y - body->buoyancyArm.y * up.x ) - total->angular );
			total->angular += turn;
			needAngular -= turn;
		}

		for ( int i = body->firstPatch; i < body->firstPatch + body->patchCount; ++i )
		{
			const b2fPatch* patch = f->patches + i;
			const b2fBody* other = f->bodies + patch->otherIndex;
			b2Vec2 normal = { patch->normalX, patch->normalY };
			if ( other->adheres == false )
			{
				// Off another moving body never downwards, and only when that body is awake in the liquid too. The
				// area of a hull's planks says little about the lift of a hull, so pushing it down from a wet wheel or
				// cargo to make up the difference sank the boat. Sideways still counts: liquid pressing on the outer
				// sides of crates in a row otherwise clamps them together.
				float down = b2Dot( normal, up );
				normal = down < 0.0f ? b2MulSub( normal, down, up ) : normal;
				float length = b2Length( normal );
				if ( other->awake == false || totals[patch->otherIndex].wetMass <= 0.0f || length < 0.1f )
				{
					continue;
				}
				normal = b2MulSV( 1.0f / length, normal );
			}

			float impulse = needX * normal.x + needY * normal.y;
			if ( impulse <= 0.0f )
			{
				continue;
			}

			float impulseX = impulse * normal.x, impulseY = impulse * normal.y;
			float angular = needAngular < -impulse * arm ? -impulse * arm : ( needAngular > impulse * arm ? impulse * arm : needAngular );
			total->x += impulseX;
			total->y += impulseY;
			total->angular += angular;
			needX -= impulseX;
			needY -= impulseY;
			needAngular -= angular;
		}
	}
}

// What b2Fluid_GetBodyState reports, summed over the step: the liquid's push once patches are applied, before added
// mass shares the body's change of motion with the liquid, and the share of the body under liquid, the one patches
// use, and all of it with liquid all the way round. Nothing here feeds back into the liquid or the bodies, except
// sleep: an awake body with liquid within reach does not fall asleep this step, and a sleeping one wakes after it
// (b2fWakeReached).
static void RecordReactions( b2fFluid* f, const b2fReaction* totals )
{
	float g = b2Length( f->worldGravity );
	b2Vec2 up = g > 0.0f ? b2MulSV( -1.0f / g, f->worldGravity ) : b2Vec2_zero;
	for ( int i = 0; i < f->bodyCount; ++i )
	{
		b2fBody* body = f->bodies + i;
		const b2fReaction* total = totals + i;
		body->reached = body->reached || total->wetMass > 0.0f;
		if ( body->awake == false )
		{
			continue;
		}

		body->stepImpulse.x += total->x;
		body->stepImpulse.y += total->y;
		body->stepAngular += total->angular;
		if ( total->wetMass <= 0.0f )
		{
			continue;
		}

		// Liquid in reach keeps a body moved by forces from sleeping. Box2D adds the step's time to sleepTime after
		// the substeps, and puts an island to sleep only once each of its bodies has been slow for B2_TIME_TO_SLEEP,
		// so the body and its island stay awake until that long after the liquid last reached it. A body floating
		// in still liquid moves slower than the sleep threshold, but the liquid holds it up only while it is awake;
		// asleep it is a wall that feels nothing. Without this it slept for a step and was woken about once a second.
		if ( body->adheres == false )
		{
			b2Array_Get( f->world->bodies, body->id.index1 - 1 )->sleepTime = 0.0f;
		}

		// Without gravity there is no under, and only liquid all round counts
		body->stepWet = true;
		float share = g > 0.0f ? SubmergedShare( body, total ) : 0.0f;
		body->stepSubmerged += b2Length( OpenSide( f, body, total, up ) ) < 0.1f ? 1.0f : share;
	}
}

// Changes an awake body's velocity in Box2D's state, as b2Body_ApplyLinearImpulseToCenter does
static void ApplyImpulse( b2fBody* body, b2Vec2 impulse, float angularImpulse )
{
	body->state->linearVelocity = b2MulAdd( body->state->linearVelocity, body->motion.invMass, impulse );
	body->state->angularVelocity += body->motion.invInertia * angularImpulse;
}

// The body and the liquid within its reach take the change of the body's motion this substep as one lump: the
// liquid's push, and what Box2D added since the last substep. A light body then speeds up about as a body in
// liquid does, rather than as one in air pushed by the liquid, and the liquid near it is carried along. A body
// floating or resting at a steady speed has no change of motion to share, so nothing changes for it. The lump's
// mass, first moment and inertia about the centre of mass give a 3x3 solve, which keeps momentum.
static void Respond( b2fBody* body, const b2fReaction* total )
{
	const b2fMotion* motion = &body->motion;
	body->sharing = false;
	if ( motion->invMass == 0.0f || total->mass <= 0.0f )
	{
		if ( total->x != 0.0f || total->y != 0.0f || total->angular != 0.0f )
		{
			ApplyImpulse( body, ( b2Vec2 ){ total->x, total->y }, total->angular );
			body->lastVelocity = b2MulAdd( body->lastVelocity, motion->invMass, ( b2Vec2 ){ total->x, total->y } );
			body->lastAngularVelocity += motion->invInertia * total->angular;
		}
		return;
	}

	float bodyMass = 1.0f / motion->invMass, lumpMass = bodyMass + total->mass;
	float forceX = total->x + bodyMass * body->solverDelta.x;
	float forceY = total->y + bodyMass * body->solverDelta.y;
	float dw = 0.0f, bodyInertia = 0.0f;
	if ( motion->invInertia > 0.0f )
	{
		bodyInertia = 1.0f / motion->invInertia;
		float torque = total->angular + bodyInertia * body->solverAngularDelta;
		float sx = total->momentX, sy = total->momentY;
		float pivot = bodyInertia + total->inertia - ( sx * sx + sy * sy ) / lumpMass;
		dw = ( torque + ( sy * forceX - sx * forceY ) / lumpMass ) / pivot;
		forceX += sy * dw;
		forceY -= sx * dw;
	}
	b2Vec2 dv = { forceX / lumpMass, forceY / lumpMass };

	b2Vec2 bodyDelta = b2Sub( dv, body->solverDelta );
	float bodyAngularDelta = motion->invInertia > 0.0f ? dw - body->solverAngularDelta : 0.0f;
	ApplyImpulse( body, b2MulSV( bodyMass, bodyDelta ), bodyInertia * bodyAngularDelta );
	body->lastVelocity = b2Add( body->lastVelocity, bodyDelta );
	body->lastAngularVelocity += bodyAngularDelta;

	body->shareVelocity = dv;
	body->shareAngular = dw;
	body->sharing = true;
}

// Each particle within reach of a body moves with the body's change of motion, in proportion to how much of it the
// lump counted
static void ShareTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	float h = f->h;
	for ( int i = start; i < end; ++i )
	{
		float added = f->liquids[f->liquid[i]].def.tuning.addedMass;

		const b2fContact* contacts = f->contacts + B2F_CONTACTS_PER_PARTICLE * i;
		int count = f->contactCount[i];
		for ( int c = 0; c < count; ++c )
		{
			const b2fBody* body = f->bodies + contacts[c].bodyIndex;
			float share = body->sharing && added > 0.0f ? ContactShare( contacts, count, c ) : 0.0f;
			if ( share > 0.0f )
			{
				float rx = (float)( f->pos[i].x - body->center.x ), ry = (float)( f->pos[i].y - body->center.y );
				float scale = h * added * share;
				f->pos[i].x += scale * ( body->shareVelocity.x - body->shareAngular * ry );
				f->pos[i].y += scale * ( body->shareVelocity.y + body->shareAngular * rx );
			}
		}
	}
}

static void SetShapeBounds( b2fFluid* f );

// A body's motion part way through a step, from its sim, which holds where it was at the start of the step, and its
// state, which holds its velocity and how far it has moved since then. A sleeping body has no state and is still.
static void ReadMotion( b2World* world, b2fBody* body )
{
	const b2Body* source = b2Array_Get( world->bodies, body->id.index1 - 1 );
	b2SolverSet* set = b2Array_Get( world->solverSets, source->setIndex );
	const b2BodySim* sim = b2Array_Get( set->bodySims, source->localIndex );
	b2fMotion* motion = &body->motion;
	motion->localCenter = sim->localCenter;
	body->awake = source->setIndex == b2_awakeSet;
	if ( body->awake )
	{
		b2BodyState* state = b2Array_Get( set->bodyStates, source->localIndex );
		body->state = state;
		motion->center = b2OffsetPos( sim->center, state->deltaPosition );
		motion->rotation = b2NormalizeRot( b2MulRot( state->deltaRotation, sim->transform.q ) );
		motion->linearVelocity = state->linearVelocity;
		motion->angularVelocity = state->angularVelocity;
		motion->invMass = sim->invMass;
		motion->invInertia = sim->invInertia;
		return;
	}

	body->state = NULL;
	motion->center = sim->center;
	motion->rotation = sim->transform.q;
	motion->linearVelocity = b2Vec2_zero;
	motion->angularVelocity = 0.0f;
	motion->invMass = 0.0f;
	motion->invInertia = 0.0f;
}

void b2fUpdateBodies( b2fFluid* f )
{
	for ( int i = 0; i < f->bodyCount; ++i )
	{
		b2fBody* body = f->bodies + i;
		ReadMotion( f->world, body );
		body->solverDelta = body->awake ? b2Sub( body->motion.linearVelocity, body->lastVelocity ) : b2Vec2_zero;
		body->solverAngularDelta = body->awake ? body->motion.angularVelocity - body->lastAngularVelocity : 0.0f;
		body->lastVelocity = body->motion.linearVelocity;
		body->lastAngularVelocity = body->motion.angularVelocity;
		b2Vec2 offset = b2RotateVector( body->motion.rotation, body->motion.localCenter );
		body->center = body->motion.center;
		body->origin = b2OffsetPos( body->center, b2Neg( offset ) );
		body->low = FLT_MAX;
		body->high = -FLT_MAX;
		body->buoyancyArm = body->area > 0.0f ? b2Sub( b2RotateVector( body->motion.rotation, b2MulSV( 1.0f / body->area, body->areaCenter ) ), offset ) : b2Vec2_zero;
	}

	SetShapeBounds( f );
}

// Every body where Box2D has it and still, as a wall that feels nothing, for passes that run outside a step
void b2fHoldBodies( b2fFluid* f )
{
	for ( int i = 0; i < f->bodyCount; ++i )
	{
		b2fBody* body = f->bodies + i;
		b2Body* source = b2Array_Get( f->world->bodies, body->id.index1 - 1 );
		const b2BodySim* sim = b2GetBodySim( f->world, source );
		b2WorldTransform transform = sim->transform;
		b2Vec2 localCenter = sim->localCenter;
		body->motion = ( b2fMotion ){ .rotation = transform.q, .localCenter = localCenter };
		body->state = NULL;
		body->origin = transform.p;
		body->center = b2OffsetPos( transform.p, b2RotateVector( transform.q, localCenter ) );
		body->motion.center = body->center;
		body->awake = false;
		body->sharing = false;
		body->solverDelta = b2Vec2_zero;
		body->solverAngularDelta = 0.0f;
		body->low = FLT_MAX;
		body->high = -FLT_MAX;
		body->buoyancyArm = b2Vec2_zero;
	}
	SetShapeBounds( f );
}

// Every shape's bounds with its body where the substep has it, grown by the farthest a particle reaches, and every
// body's lowest and highest points
static void SetShapeBounds( b2fFluid* f )
{
	float g = b2Length( f->worldGravity );
	b2Vec2 up = g > 0.0f ? b2MulSV( -1.0f / g, f->worldGravity ) : b2Vec2_zero;

	for ( int i = 0; i < f->shapeCount; ++i )
	{
		b2fShape* s = f->shapes + i;
		b2fBody* body = f->bodies + s->bodyIndex;
		SetBounds( s, body->origin, body->motion.rotation, s->radius + f->maxRadius, body, up );
	}
}

void b2fFindContacts( b2fFluid* f, b2SubStepContext* context )
{
	int n = f->count;
	if ( f->shapeCount > 0 )
	{
		BinShapes( f );
		b2SubStep_ParallelFor( context, ContactTask, n, 512, f );
	}
	else
	{
		memset( f->contactCount, 0, (size_t)n * sizeof( int ) );
		memset( f->wallDensity, 0, (size_t)n * sizeof( float ) );
		memset( f->wallNearDensity, 0, (size_t)n * sizeof( float ) );
	}
}

// The push out of shapes and the friction along them, with nothing for the bodies, for passes that run outside a step
// with the bodies held
void b2fCollideHeld( b2fFluid* f )
{
	if ( f->shapeCount > 0 )
	{
		CollideTask( 0, B2F_CHUNK_COUNT, 0, f );
	}
}

void b2fCollide( b2fFluid* f, b2SubStepContext* context )
{
	if ( f->shapeCount == 0 )
	{
		return;
	}

	b2SubStep_ParallelFor( context, CollideTask, B2F_CHUNK_COUNT, 1, f );

	// Chunk 0 holds the totals, summed in chunk order so the result does not depend on the workers
	b2fReaction* totals = f->reactions;
	for ( int chunk = 1; chunk < B2F_CHUNK_COUNT; ++chunk )
	{
		for ( int i = 0; i < f->bodyCount; ++i )
		{
			const b2fReaction* r = f->reactions + chunk * f->bodyCount + i;
			b2fReaction* t = totals + i;
			t->x += r->x;
			t->y += r->y;
			t->angular += r->angular;
			t->mass += r->mass;
			t->momentX += r->momentX;
			t->momentY += r->momentY;
			t->inertia += r->inertia;
			t->wetMass += r->wetMass;
			t->wetDensity += r->wetDensity;
			t->wetLength += r->wetLength;
			t->wetNormalX += r->wetNormalX;
			t->wetNormalY += r->wetNormalY;
			t->wetTop = r->wetTop > t->wetTop ? r->wetTop : t->wetTop;
		}
	}

	bool patches = false;
	for ( int l = 0; l < f->liquidCount; ++l )
	{
		patches = patches || f->liquids[l].def.tuning.buoyancyAssist > 0.0f;
	}
	if ( patches )
	{
		ApplyPatches( f, totals );
	}

	RecordReactions( f, totals );

	bool sharing = false;
	for ( int i = 0; i < f->bodyCount; ++i )
	{
		b2fBody* body = f->bodies + i;
		if ( body->awake )
		{
			Respond( body, totals + i );
			sharing = sharing || body->sharing;
		}
		else
		{
			body->sharing = false;
		}
	}

	if ( sharing )
	{
		b2SubStep_ParallelFor( context, ShareTask, f->count, 1024, f );
	}
}
