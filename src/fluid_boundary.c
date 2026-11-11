// SPDX-License-Identifier: MIT
// Shapes and bodies near the liquid: wall density, contact moves, collision, reactions on bodies and the pressure
// in contact patches.
#include "fluid_internal.h"

// The kernels integrated over a solid half plane at distance d from a particle, at the number density of the
// liquid at rest. Particles settle about half a spacing from a surface, which is where a lattice
// of particles would end, so the table carries on the lattice without a seam. At the denser lattice rest density
// is counted from, a wall counted for more than the liquid it stands in for and kept the liquid next to it thin,
// which gave a body lift for more than its own volume, a small body the most. Columns are density,
// near density, and the matching pressure and near pressure weights along the normal.
void b2fBuildWallTable( b2fFluid* f )
{
	float h = f->radius;
	float numberDensity = B2F_REST_FILL / ( 0.8660254f * f->def.particleSpacing * f->def.particleSpacing );
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

static int FindBody( b2fFluid* f, b2BodyId id )
{
	for ( int i = 0; i < f->bodyCount; ++i )
	{
		if ( B2_ID_EQUALS( f->bodies[i].id, id ) )
		{
			return i;
		}
	}

	if ( f->bodyCount == f->bodyCapacity )
	{
		f->bodyCapacity = f->bodyCapacity == 0 ? 32 : 2 * f->bodyCapacity;
		f->bodies = realloc( f->bodies, (size_t)f->bodyCapacity * sizeof( b2fBody ) );
	}
	f->bodies[f->bodyCount] = (b2fBody){
		.id = id,
		.lastVelocity = b2Body_GetLinearVelocity( id ),
		.lastAngularVelocity = b2Body_GetAngularVelocity( id ),
		.adheres = b2Body_GetType( id ) != b2_dynamicBody,
	};
	return f->bodyCount++;
}

static bool OverlapShape( b2ShapeId shapeId, void* context )
{
	b2fFluid* f = context;
	if ( b2Shape_IsSensor( shapeId ) )
	{
		return true;
	}

	if ( f->shapeCount == f->shapeCapacity )
	{
		f->shapeCapacity = f->shapeCapacity == 0 ? 32 : 2 * f->shapeCapacity;
		f->shapes = realloc( f->shapes, (size_t)f->shapeCapacity * sizeof( b2fShape ) );
	}

	b2fShape* shape = f->shapes + f->shapeCount;
	memset( shape, 0, sizeof( b2fShape ) );
	shape->type = b2Shape_GetType( shapeId );
	shape->bodyIndex = FindBody( f, b2Shape_GetBody( shapeId ) );
	float area = 0.0f;
	switch ( shape->type )
	{
		case b2_circleShape:
		{
			b2Circle c = b2Shape_GetCircle( shapeId );
			shape->p1 = shape->p2 = c.center;
			shape->radius = c.radius;
			area = B2_PI * c.radius * c.radius;
		}
		break;

		case b2_capsuleShape:
		{
			b2Capsule c = b2Shape_GetCapsule( shapeId );
			shape->p1 = c.center1;
			shape->p2 = c.center2;
			shape->radius = c.radius;
			area = B2_PI * c.radius * c.radius + 2.0f * c.radius * b2Distance( c.center1, c.center2 );
		}
		break;

		case b2_segmentShape:
		{
			b2Segment s = b2Shape_GetSegment( shapeId );
			shape->p1 = s.point1;
			shape->p2 = s.point2;
		}
		break;

		case b2_chainSegmentShape:
		{
			b2ChainSegment s = b2Shape_GetChainSegment( shapeId );
			shape->p1 = s.segment.point1;
			shape->p2 = s.segment.point2;
		}
		break;

		case b2_polygonShape:
			shape->polygon = b2Shape_GetPolygon( shapeId );
			shape->radius = shape->polygon.radius;
			area = b2ComputePolygonMass( &shape->polygon, 1.0f ).mass;
			break;

		default:
			return true;
	}

	f->bodies[shape->bodyIndex].area += area;
	f->shapeCount += 1;
	return true;
}

static void AddPatch( b2fFluid* f, int bodyIndex, int otherIndex, b2Vec2 normal )
{
	if ( f->patchCount == f->patchCapacity )
	{
		f->patchCapacity = f->patchCapacity == 0 ? 32 : 2 * f->patchCapacity;
		f->patches = realloc( f->patches, (size_t)f->patchCapacity * sizeof( b2fPatch ) );
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

// What each moving body touches or nearly touches, found once a step from where the bodies are then. Box2D's own
// contacts end 2 cm apart, well short of the gap a particle fits in, so a body lifting off had nothing under it
// in between.
static void FindPatches( b2fFluid* f )
{
	float range = B2F_PATCH_GAP * f->def.particleSpacing;
	f->patchCount = 0;
	for ( int bodyIndex = 0; bodyIndex < f->bodyCount; ++bodyIndex )
	{
		b2fBody* body = f->bodies + bodyIndex;
		body->firstPatch = f->patchCount;
		body->patchCount = 0;
		if ( body->adheres || b2Body_IsAwake( body->id ) == false )
		{
			continue;
		}

		b2WorldTransform transform = b2Body_GetTransform( body->id );
		for ( int i = 0; i < f->shapeCount; ++i )
		{
			if ( f->shapes[i].bodyIndex != bodyIndex )
			{
				continue;
			}

			b2ShapeProxy proxy = ShapeProxy( f->shapes + i );
			for ( int j = 0; j < f->shapeCount; ++j )
			{
				int otherIndex = f->shapes[j].bodyIndex;
				if ( otherIndex == bodyIndex )
				{
					continue;
				}

				b2DistanceInput input = { 0 };
				input.proxyA = proxy;
				input.proxyB = ShapeProxy( f->shapes + j );
				input.transform = b2InvMulWorldTransforms( transform, b2Body_GetTransform( f->bodies[otherIndex].id ) );
				input.useRadii = true;
				b2SimplexCache cache = { 0 };
				b2DistanceOutput output = b2ComputeShapeDistance( &input, &cache, NULL, 0 );
				if ( output.distance < range && b2LengthSquared( output.normal ) > 0.5f )
				{
					AddPatch( f, bodyIndex, otherIndex, b2Neg( b2RotateVector( transform.q, output.normal ) ) );
				}
			}
		}

		// Shapes pressed into each other have no direction between them, so those come from Box2D's contacts
		enum { contactCapacity = 32 };
		b2ContactData contacts[contactCapacity];
		int count = b2Body_GetContactData( body->id, contacts, contactCapacity );
		for ( int i = 0; i < count; ++i )
		{
			const b2Manifold* manifold = &contacts[i].manifold;
			b2BodyId bodyA = b2Shape_GetBody( contacts[i].shapeIdA ), bodyB = b2Shape_GetBody( contacts[i].shapeIdB );
			bool first = B2_ID_EQUALS( bodyA, body->id );
			b2BodyId otherId = first ? bodyB : bodyA;
			int otherIndex = 0;
			while ( otherIndex < f->bodyCount && B2_ID_EQUALS( f->bodies[otherIndex].id, otherId ) == false )
			{
				otherIndex += 1;
			}
			if ( otherIndex < f->bodyCount && manifold->pointCount > 0 && manifold->points[0].separation <= 0.0f )
			{
				AddPatch( f, bodyIndex, otherIndex, first ? b2Neg( manifold->normal ) : manifold->normal );
			}
		}
	}
}

// Shapes near the liquid, gathered once a step
void b2fGatherShapes( b2fFluid* f )
{
	f->shapeCount = 0;
	f->bodyCount = 0;
	if ( f->count > 0 )
	{
		b2fCoord minX = f->pos[0].x, maxX = f->pos[0].x, minY = f->pos[0].y, maxY = f->pos[0].y;
		float maxSpeed2 = 0.0f;
		for ( int i = 0; i < f->count; ++i )
		{
			minX = minX < f->pos[i].x ? minX : f->pos[i].x;
			maxX = maxX > f->pos[i].x ? maxX : f->pos[i].x;
			minY = minY < f->pos[i].y ? minY : f->pos[i].y;
			maxY = maxY > f->pos[i].y ? maxY : f->pos[i].y;
			float speed2 = f->vx[i] * f->vx[i] + f->vy[i] * f->vy[i];
			maxSpeed2 = maxSpeed2 > speed2 ? maxSpeed2 : speed2;
		}

		// A generous margin, since bodies and particles both move during the step
		float grow = f->radius + 0.1f * sqrtf( maxSpeed2 ) + 1.0f;
		b2Pos origin = { minX, minY };
		b2AABB aabb = { { -grow, -grow }, { (float)( maxX - minX ) + grow, (float)( maxY - minY ) + grow } };
		b2World_OverlapAABB( f->worldId, origin, aabb, b2DefaultQueryFilter(), OverlapShape, f );
	}

	int needed = f->bodyCount * B2F_CHUNK_COUNT;
	if ( needed > f->reactionCapacity )
	{
		f->reactionCapacity = 2 * needed;
		f->reactions = realloc( f->reactions, (size_t)f->reactionCapacity * sizeof( b2fReaction ) );
	}

	// A sleeping body cannot feel the liquid, so it would never lift off. For now nothing near the
	// liquid sleeps, which is the blunt answer to the sleeping problem in DESIGN.md.
	for ( int i = 0; i < f->bodyCount; ++i )
	{
		if ( b2Body_GetType( f->bodies[i].id ) == b2_dynamicBody && b2Body_IsAwake( f->bodies[i].id ) == false )
		{
			b2Body_SetAwake( f->bodies[i].id, true );
		}
	}

	FindPatches( f );
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

static void ContactTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	float h = f->radius, tableScale = (float)B2F_WALL_TABLE_SIZE / h;
	bool adaptive = f->adaptive;
	for ( int i = start; i < end; ++i )
	{
		if ( adaptive )
		{
			h = 2.0f * f->size[i];
			tableScale = (float)B2F_WALL_TABLE_SIZE / h;
		}

		b2fCoord x = f->pos[i].x, y = f->pos[i].y;
		float density = 0.0f, nearDensity = 0.0f;
		int count = 0;
		b2fContact* contacts = f->contacts + B2F_CONTACTS_PER_PARTICLE * i;
		for ( int si = 0; si < f->shapeCount && count < B2F_CONTACTS_PER_PARTICLE; ++si )
		{
			const b2fShape* s = f->shapes + si;
			if ( x < s->minX || x > s->maxX || y < s->minY || y > s->maxY )
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

			float t = ( distance > 0.0f ? distance : 0.0f ) * tableScale;
			int index = (int)t;
			index = index < B2F_WALL_TABLE_SIZE ? index : B2F_WALL_TABLE_SIZE - 1;
			float blend = t - (float)index;
			const float* a = f->wallTable[index];
			const float* b = f->wallTable[index + 1];
			b2Vec2 n = b2RotateVector( body->motion.rotation, normal );
			density += a[0] + blend * ( b[0] - a[0] );
			nearDensity += a[1] + blend * ( b[1] - a[1] );
			float share = 1.0f - ( distance > 0.0f ? distance : 0.0f ) / h;
			contacts[count++] = (b2fContact){ s->bodyIndex, n.x, n.y, a[2] + blend * ( b[2] - a[2] ), a[3] + blend * ( b[3] - a[3] ), 0.0f, share };
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
	float margin = f->margin, h = f->h, inv = 1.0f / h, friction = f->liquids[0].def.friction, m = f->mass;
	bool adaptive = f->adaptive, multi = f->liquidCount > 1;
	float baseMass = f->mass, spacing = f->def.particleSpacing, added = f->liquids[0].def.addedMass;

	// What a patch needs of the liquid around its body: the density its buoyancy goes by, and how high the liquid
	// stands on the body, against gravity
	const b2fLiquidDef* def = &f->liquids[0].def;
	float buoyantDensity = def->contactPressure * def->massDensity * def->gravityScale;
	float g = b2Length( f->worldGravity );
	b2Vec2 up = g > 0.0f ? b2MulSV( -1.0f / g, f->worldGravity ) : b2Vec2_zero;

	for ( int chunk = start; chunk < end; ++chunk )
	{
		b2fReaction* reactions = f->reactions + chunk * f->bodyCount;
		memset( reactions, 0, (size_t)f->bodyCount * sizeof( b2fReaction ) );

		int first = (int)( (int64_t)f->count * chunk / B2F_CHUNK_COUNT );
		int last = (int)( (int64_t)f->count * ( chunk + 1 ) / B2F_CHUNK_COUNT );
		for ( int i = first; i < last; ++i )
		{
			if ( multi )
			{
				const b2fLiquid* liquid = f->liquids + f->liquid[i];
				baseMass = liquid->mass;
				m = baseMass;
				friction = liquid->def.friction;
				buoyantDensity = liquid->def.contactPressure * liquid->def.massDensity * liquid->def.gravityScale;
				added = liquid->def.addedMass;
			}
			if ( adaptive )
			{
				float ratio = f->size[i] / spacing;
				m = baseMass * ratio * ratio;
				margin = 0.5f * f->size[i];
			}

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
						reaction->wetHeight += m * ( rx * up.x + ry * up.y );
						reaction->wetLength += f->size[i];
						reaction->wetNormalX += f->size[i] * contacts[c].normalX;
						reaction->wetNormalY += f->size[i] * contacts[c].normalY;
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
			}

			for ( int si = 0; si < f->shapeCount; ++si )
			{
				const b2fShape* s = f->shapes + si;
				b2fCoord x = f->pos[i].x, y = f->pos[i].y;
				if ( x < s->minX || x > s->maxX || y < s->minY || y > s->maxY )
				{
					continue;
				}

				const b2fBody* body = f->bodies + s->bodyIndex;
				b2Pos world = { x, y };
				b2Vec2 local = b2InvRotateVector( body->motion.rotation, b2SubPos( world, body->origin ) );
				float distance;
				b2Vec2 normal;
				if ( b2fShapeDistance( s, local, &distance, &normal ) == false || distance >= margin )
				{
					continue;
				}

				b2Vec2 n = b2RotateVector( body->motion.rotation, normal );
				float push = margin - distance;

				// Sizes can change next to a shape. What a body feels is limited to what a particle could carry in.
				if ( adaptive && push > 2.0f * margin )
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

// Liquid cannot fill a gap narrower than a few particles, so a body touching or nearly touching something misses the
// liquid's push on that side. Still liquid pushes on a body with the weight of the liquid it displaces, Archimedes'
// principle, whatever the depth, and does not turn it. So the gap gives what the particles' push falls short of that,
// along the direction away from the other shape: a body lighter than the liquid lifts off, a heavier one stays down,
// and as particles fill the gap their push takes over and this fades. Reading the pressure in the gap from the depth
// instead had to be right to a few percent of that pressure, many times the body's buoyancy in deep liquid, to tell
// the two apart. It never pulls. The share of the body under liquid is taken from how high the liquid
// within reach stands on it: twice its mean height above the body's lowest point, which is exact for a box wet up its
// sides.
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

		float share = 2.0f * ( total->wetHeight / total->wetMass - body->low ) / height;
		share = share < 0.0f ? 0.0f : ( share > 1.0f ? 1.0f : share );
		float lift = total->wetDensity / total->wetMass * body->area * share * g * f->h;
		float needX = lift * up.x - total->x, needY = lift * up.y - total->y, needAngular = -total->angular;
		float arm = 0.5f * sqrtf( body->area );

		// Settled particles lock together like packed grains and can hold a body whose weight and buoyancy differ by
		// a tenth in place, anywhere in the liquid. A body with liquid all the way round that is nearly still gets
		// its buoyancy exactly, up or down, so it cannot hang there. Once it moves the particles' push takes over,
		// which is what slows it. A body at the surface or with a dry side, such as a hull, is left to the particles.
		float open = b2Length( ( b2Vec2 ){ total->wetNormalX, total->wetNormalY } ) / total->wetLength;
		float round = 1.0f - 4.0f * open;
		float still = 1.0f - b2Length( body->motion.linearVelocity ) / B2F_STILL_SPEED;
		if ( round > 0.0f && still > 0.0f )
		{
			// All of it is under liquid then, which the estimate above only gets to within a few percent
			float full = total->wetDensity / total->wetMass * body->area * g * f->h;
			float impulse = round * still * ( full - total->x * up.x - total->y * up.y );
			total->x += impulse * up.x;
			total->y += impulse * up.y;
			needX -= impulse * up.x;
			needY -= impulse * up.y;
		}

		for ( int i = body->firstPatch; i < body->firstPatch + body->patchCount; ++i )
		{
			const b2fPatch* patch = f->patches + i;
			const b2fBody* other = f->bodies + patch->otherIndex;
			float impulse = needX * patch->normalX + needY * patch->normalY;
			if ( impulse <= 0.0f )
			{
				continue;
			}

			// Off another moving body only upwards, and only when that body is in the liquid too. The area of a
			// hull's planks says little about the lift of a hull, so pushing it down from a wet wheel or cargo to make
			// up the difference sank the boat.
			if ( other->adheres == false && ( totals[patch->otherIndex].wetMass <= 0.0f || patch->normalX * up.x + patch->normalY * up.y <= 0.0f ) )
			{
				continue;
			}

			float impulseX = impulse * patch->normalX, impulseY = impulse * patch->normalY;
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

// The body and the liquid within its reach take the change of the body's motion this substep as one lump: the
// liquid's push, and what Box2D added since the last substep. A light body then speeds up about as a body in
// liquid does, rather than as one in air pushed by the liquid, and the liquid near it is carried along. A body
// floating or resting at a steady speed has no change of motion to share, so nothing changes for it. The lump's
// mass, first moment and inertia about the centre of mass give a 3x3 solve, which keeps momentum.
static void Respond( b2SubStepContext* context, b2fBody* body, const b2fReaction* total )
{
	const b2SubStepBody* motion = &body->motion;
	body->sharing = false;
	if ( motion->invMass == 0.0f || total->mass <= 0.0f )
	{
		if ( total->x != 0.0f || total->y != 0.0f || total->angular != 0.0f )
		{
			b2SubStep_ApplyImpulse( context, body->id, ( b2Vec2 ){ total->x, total->y }, total->angular );
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
	b2SubStep_ApplyImpulse( context, body->id, b2MulSV( bodyMass, bodyDelta ), bodyInertia * bodyAngularDelta );
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
	float h = f->h, added = f->liquids[0].def.addedMass;
	bool multi = f->liquidCount > 1;
	for ( int i = start; i < end; ++i )
	{
		if ( multi )
		{
			added = f->liquids[f->liquid[i]].def.addedMass;
		}

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

void b2fUpdateBodies( b2fFluid* f, b2SubStepContext* context )
{
	for ( int i = 0; i < f->bodyCount; ++i )
	{
		b2fBody* body = f->bodies + i;
		body->awake = b2SubStep_GetBodyMotion( context, body->id, &body->motion );
		body->solverDelta = body->awake ? b2Sub( body->motion.linearVelocity, body->lastVelocity ) : b2Vec2_zero;
		body->solverAngularDelta = body->awake ? body->motion.angularVelocity - body->lastAngularVelocity : 0.0f;
		body->lastVelocity = body->motion.linearVelocity;
		body->lastAngularVelocity = body->motion.angularVelocity;
		b2Vec2 offset = b2RotateVector( body->motion.rotation, body->motion.localCenter );
		body->center = body->motion.center;
		body->origin = b2OffsetPos( body->center, b2Neg( offset ) );
		body->low = FLT_MAX;
		body->high = -FLT_MAX;
	}

	float g = b2Length( f->worldGravity );
	b2Vec2 up = g > 0.0f ? b2MulSV( -1.0f / g, f->worldGravity ) : b2Vec2_zero;

	for ( int i = 0; i < f->shapeCount; ++i )
	{
		b2fShape* s = f->shapes + i;
		b2fBody* body = f->bodies + s->bodyIndex;
		b2fCoord minX = FLT_MAX, minY = FLT_MAX, maxX = -FLT_MAX, maxY = -FLT_MAX;
		int count = s->type == b2_polygonShape ? s->polygon.count : 2;
		for ( int j = 0; j < count; ++j )
		{
			b2Vec2 local = s->type == b2_polygonShape ? s->polygon.vertices[j] : ( j == 0 ? s->p1 : s->p2 );
			b2Pos p = b2OffsetPos( body->origin, b2RotateVector( body->motion.rotation, local ) );
			minX = minX < p.x ? minX : p.x;
			minY = minY < p.y ? minY : p.y;
			maxX = maxX > p.x ? maxX : p.x;
			maxY = maxY > p.y ? maxY : p.y;
			float height = b2Dot( b2SubPos( p, body->center ), up );
			body->low = body->low < height - s->radius ? body->low : height - s->radius;
			body->high = body->high > height + s->radius ? body->high : height + s->radius;
		}
		float grow = s->radius + f->maxRadius;
		s->minX = minX - grow;
		s->minY = minY - grow;
		s->maxX = maxX + grow;
		s->maxY = maxY + grow;
	}
}

void b2fFindContacts( b2fFluid* f, b2SubStepContext* context )
{
	int n = f->count;
	if ( f->shapeCount > 0 )
	{
		b2SubStep_ParallelFor( context, ContactTask, n, 512, f );
	}
	else
	{
		memset( f->contactCount, 0, (size_t)n * sizeof( int ) );
		memset( f->wallDensity, 0, (size_t)n * sizeof( float ) );
		memset( f->wallNearDensity, 0, (size_t)n * sizeof( float ) );
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
			t->wetHeight += r->wetHeight;
			t->wetLength += r->wetLength;
			t->wetNormalX += r->wetNormalX;
			t->wetNormalY += r->wetNormalY;
		}
	}

	bool patches = false;
	for ( int l = 0; l < f->liquidCount; ++l )
	{
		patches = patches || f->liquids[l].def.contactPressure > 0.0f;
	}
	if ( patches )
	{
		ApplyPatches( f, totals );
	}

	bool sharing = false;
	for ( int i = 0; i < f->bodyCount; ++i )
	{
		b2fBody* body = f->bodies + i;
		if ( body->awake )
		{
			Respond( context, body, totals + i );
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
