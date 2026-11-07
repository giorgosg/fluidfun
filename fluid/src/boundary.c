// SPDX-License-Identifier: MIT
// Shapes and bodies near the liquid: wall density, contact moves, collision, reactions on bodies and the pressure
// in contact patches.
#include "internal.h"

// The kernels integrated over a solid half plane at distance d from a particle, at the liquid's
// number density. Particles settle about half a spacing from a surface, which is where a lattice
// of particles would end, so the table carries on the lattice without a seam. Columns are density,
// near density, and the matching pressure and near pressure weights along the normal.
void b2fBuildWallTable( b2fFluid* f )
{
	float h = f->radius;
	float numberDensity = 1.0f / ( 0.8660254f * f->def.particleSpacing * f->def.particleSpacing );
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
	f->bodies[f->bodyCount] = (b2fBody){ .id = id };
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
	switch ( shape->type )
	{
		case b2_circleShape:
		{
			b2Circle c = b2Shape_GetCircle( shapeId );
			shape->p1 = shape->p2 = c.center;
			shape->radius = c.radius;
		}
		break;

		case b2_capsuleShape:
		{
			b2Capsule c = b2Shape_GetCapsule( shapeId );
			shape->p1 = c.center1;
			shape->p2 = c.center2;
			shape->radius = c.radius;
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
			break;

		default:
			return true;
	}

	f->shapeCount += 1;
	return true;
}

// Contacts come from the last step, since the world is locked during the substeps.
static void FindPatches( b2fFluid* f )
{
	enum { contactCapacity = 32 };
	b2ContactData contacts[contactCapacity];

	f->patchCount = 0;
	for ( int bodyIndex = 0; bodyIndex < f->bodyCount; ++bodyIndex )
	{
		b2fBody* body = f->bodies + bodyIndex;
		body->firstPatch = f->patchCount;
		body->patchCount = 0;
		if ( b2Body_GetType( body->id ) != b2_dynamicBody || b2Body_IsAwake( body->id ) == false )
		{
			continue;
		}

		int count = b2Body_GetContactData( body->id, contacts, contactCapacity );
		for ( int i = 0; i < count; ++i )
		{
			const b2Manifold* manifold = &contacts[i].manifold;
			// The manifold outlives the touch by a little. A gap this thin still has no liquid in it.
			float maxGap = 2.0f * f->margin;
			if ( manifold->pointCount != 2 || manifold->points[0].separation > maxGap || manifold->points[1].separation > maxGap )
			{
				continue;
			}

			if ( f->patchCount == f->patchCapacity )
			{
				f->patchCapacity = f->patchCapacity == 0 ? 32 : 2 * f->patchCapacity;
				f->patches = realloc( f->patches, (size_t)f->patchCapacity * sizeof( b2fPatch ) );
			}

			b2BodyId bodyA = b2Shape_GetBody( contacts[i].shapeIdA );
			b2Pos center = b2Body_GetWorldCenter( bodyA );
			float sign = B2_ID_EQUALS( bodyA, body->id ) ? -1.0f : 1.0f;
			b2fPatch* patch = f->patches + f->patchCount;
			patch->bodyIndex = bodyIndex;
			patch->normalX = sign * manifold->normal.x;
			patch->normalY = sign * manifold->normal.y;
			for ( int j = 0; j < 2; ++j )
			{
				patch->x[j] = (float)center.x + manifold->points[j].anchorA.x;
				patch->y[j] = (float)center.y + manifold->points[j].anchorA.y;
			}
			float dx = patch->x[1] - patch->x[0], dy = patch->y[1] - patch->y[0];
			patch->length = sqrtf( dx * dx + dy * dy );
			f->patchCount += 1;
			body->patchCount += 1;
		}
	}

	int needed = 2 * f->patchCount * B2F_CHUNK_COUNT;
	if ( needed > f->patchImpulseCapacity )
	{
		f->patchImpulseCapacity = 2 * needed;
		f->patchImpulses = realloc( f->patchImpulses, (size_t)f->patchImpulseCapacity * sizeof( float ) );
	}
}

// Shapes near the liquid, gathered once a step
void b2fGatherShapes( b2fFluid* f )
{
	f->shapeCount = 0;
	f->bodyCount = 0;
	if ( f->count > 0 )
	{
		float minX = f->x[0], maxX = f->x[0], minY = f->y[0], maxY = f->y[0], maxSpeed2 = 0.0f;
		for ( int i = 0; i < f->count; ++i )
		{
			minX = minX < f->x[i] ? minX : f->x[i];
			maxX = maxX > f->x[i] ? maxX : f->x[i];
			minY = minY < f->y[i] ? minY : f->y[i];
			maxY = maxY > f->y[i] ? maxY : f->y[i];
			float speed2 = f->vx[i] * f->vx[i] + f->vy[i] * f->vy[i];
			maxSpeed2 = maxSpeed2 > speed2 ? maxSpeed2 : speed2;
		}

		// A generous margin, since bodies and particles both move during the step
		float grow = f->radius + 0.1f * sqrtf( maxSpeed2 ) + 1.0f;
		b2AABB aabb = { { minX - grow, minY - grow }, { maxX + grow, maxY + grow } };
		b2World_OverlapAABB( f->worldId, b2Pos_zero, aabb, b2DefaultQueryFilter(), OverlapShape, f );
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
	bool adaptive = f->def.adaptiveRatio > 1.0f;
	for ( int i = start; i < end; ++i )
	{
		if ( adaptive )
		{
			h = 2.0f * f->size[i];
			tableScale = (float)B2F_WALL_TABLE_SIZE / h;
		}

		float x = f->x[i], y = f->y[i], density = 0.0f, nearDensity = 0.0f;
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
			b2Vec2 local = b2InvRotateVector( body->motion.rotation, b2Sub( ( b2Vec2 ){ x, y }, body->origin ) );
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
			contacts[count++] = (b2fContact){ s->bodyIndex, n.x, n.y, a[2] + blend * ( b[2] - a[2] ), a[3] + blend * ( b[3] - a[3] ), 0.0f };
		}
		f->contactCount[i] = count;
		f->wallDensity[i] = density;
		f->wallNearDensity[i] = nearDensity;
	}
}

// The liquid's push on a body's own faces, close to the ends of its contacts, gives the pressure to use
// on the part of the body that liquid cannot reach.
static inline void SamplePatches( b2fFluid* f, int chunk, const b2fBody* body, float x, float y, float impulse )
{
	float window = 2.0f * f->radius, window2 = window * window;
	float* impulses = f->patchImpulses + 2 * f->patchCount * chunk;
	for ( int i = body->firstPatch; i < body->firstPatch + body->patchCount; ++i )
	{
		const b2fPatch* patch = f->patches + i;
		for ( int end = 0; end < 2; ++end )
		{
			float dx = x - patch->x[end], dy = y - patch->y[end];
			if ( dx * dx + dy * dy < window2 )
			{
				impulses[2 * i + end] += impulse;
			}
		}
	}
}

// Reactions go to a chunk chosen by particle index, and chunks sum in order, so the total does not
// depend on which worker ran which chunk.
static void CollideTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	float margin = f->margin, h = f->h, inv = 1.0f / h, friction = f->def.friction, m = f->mass;
	bool adaptive = f->def.adaptiveRatio > 1.0f;
	float baseMass = f->mass, spacing = f->def.particleSpacing;

	for ( int chunk = start; chunk < end; ++chunk )
	{
		b2fReaction* reactions = f->reactions + chunk * f->bodyCount;
		memset( reactions, 0, (size_t)f->bodyCount * sizeof( b2fReaction ) );
		memset( f->patchImpulses + 2 * f->patchCount * chunk, 0, 2 * (size_t)f->patchCount * sizeof( float ) );

		int first = (int)( (int64_t)f->count * chunk / B2F_CHUNK_COUNT );
		int last = (int)( (int64_t)f->count * ( chunk + 1 ) / B2F_CHUNK_COUNT );
		for ( int i = first; i < last; ++i )
		{
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
					float rx = f->x[i] - body->motion.center.x, ry = f->y[i] - body->motion.center.y;
					b2fReaction* reaction = reactions + contacts[c].bodyIndex;
					reaction->x += impulseX;
					reaction->y += impulseY;
					reaction->angular += rx * impulseY - ry * impulseX;
					SamplePatches( f, chunk, body, f->x[i], f->y[i], impulse );
				}
			}

			for ( int si = 0; si < f->shapeCount; ++si )
			{
				const b2fShape* s = f->shapes + si;
				float x = f->x[i], y = f->y[i];
				if ( x < s->minX || x > s->maxX || y < s->minY || y > s->maxY )
				{
					continue;
				}

				const b2fBody* body = f->bodies + s->bodyIndex;
				b2Vec2 world = { x, y };
				b2Vec2 local = b2InvRotateVector( body->motion.rotation, b2Sub( world, body->origin ) );
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

				// Coulomb-limited slip against the surface
				b2Vec2 r = { x - body->motion.center.x, y - body->motion.center.y };
				float w = body->motion.angularVelocity;
				float bodyVx = body->motion.linearVelocity.x - w * r.y;
				float bodyVy = body->motion.linearVelocity.y + w * r.x;
				float relX = ( x + moveX - f->px[i] ) * inv - bodyVx;
				float relY = ( y + moveY - f->py[i] ) * inv - bodyVy;
				float normalSpeed = relX * n.x + relY * n.y;
				float tangentX = relX - normalSpeed * n.x, tangentY = relY - normalSpeed * n.y;
				float tangentSpeed = sqrtf( tangentX * tangentX + tangentY * tangentY );
				if ( tangentSpeed > 1e-6f )
				{
					float slip = friction * push * inv;
					slip = slip < tangentSpeed ? slip : tangentSpeed;
					moveX -= h * slip * tangentX / tangentSpeed;
					moveY -= h * slip * tangentY / tangentSpeed;
				}

				f->x[i] = x + moveX;
				f->y[i] = y + moveY;

				if ( body->awake )
				{
					float impulseX = -m * moveX * inv, impulseY = -m * moveY * inv;
					b2fReaction* reaction = reactions + s->bodyIndex;
					reaction->x += impulseX;
					reaction->y += impulseY;
					reaction->angular += r.x * impulseY - r.y * impulseX;
					SamplePatches( f, chunk, body, x, y, sqrtf( impulseX * impulseX + impulseY * impulseY ) );
				}
			}
		}
	}
}

// A wet end is enough to bring the whole gap up to pressure, so the larger end sets the push.
static void ApplyPatches( b2fFluid* f, b2SubStepContext* context )
{
	float window = 2.0f * f->radius;
	for ( int i = 0; i < f->patchCount; ++i )
	{
		float sum[2] = { 0.0f, 0.0f };
		for ( int chunk = 0; chunk < B2F_CHUNK_COUNT; ++chunk )
		{
			const float* impulses = f->patchImpulses + 2 * f->patchCount * chunk + 2 * i;
			sum[0] += impulses[0];
			sum[1] += impulses[1];
		}

		const b2fPatch* patch = f->patches + i;
		const b2fBody* body = f->bodies + patch->bodyIndex;
		if ( body->awake == false )
		{
			continue;
		}

		float impulse = f->def.contactPressure * ( sum[0] > sum[1] ? sum[0] : sum[1] ) * patch->length / window;
		float impulseX = impulse * patch->normalX, impulseY = impulse * patch->normalY;
		float rx = 0.5f * ( patch->x[0] + patch->x[1] ) - body->motion.center.x;
		float ry = 0.5f * ( patch->y[0] + patch->y[1] ) - body->motion.center.y;
		b2SubStep_ApplyImpulse( context, body->id, ( b2Vec2 ){ impulseX, impulseY }, rx * impulseY - ry * impulseX );
	}
}

void b2fUpdateBodies( b2fFluid* f, b2SubStepContext* context )
{
	for ( int i = 0; i < f->bodyCount; ++i )
	{
		b2fBody* body = f->bodies + i;
		body->awake = b2SubStep_GetBodyMotion( context, body->id, &body->motion );
		b2Vec2 offset = b2RotateVector( body->motion.rotation, body->motion.localCenter );
		body->origin = b2Sub( body->motion.center, offset );
	}

	for ( int i = 0; i < f->shapeCount; ++i )
	{
		b2fShape* s = f->shapes + i;
		const b2fBody* body = f->bodies + s->bodyIndex;
		float minX = FLT_MAX, minY = FLT_MAX, maxX = -FLT_MAX, maxY = -FLT_MAX;
		int count = s->type == b2_polygonShape ? s->polygon.count : 2;
		for ( int j = 0; j < count; ++j )
		{
			b2Vec2 local = s->type == b2_polygonShape ? s->polygon.vertices[j] : ( j == 0 ? s->p1 : s->p2 );
			b2Vec2 p = b2Add( body->origin, b2RotateVector( body->motion.rotation, local ) );
			minX = minX < p.x ? minX : p.x;
			minY = minY < p.y ? minY : p.y;
			maxX = maxX > p.x ? maxX : p.x;
			maxY = maxY > p.y ? maxY : p.y;
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
	for ( int chunk = 0; chunk < B2F_CHUNK_COUNT; ++chunk )
	{
		for ( int i = 0; i < f->bodyCount; ++i )
		{
			const b2fReaction* r = f->reactions + chunk * f->bodyCount + i;
			if ( f->bodies[i].awake && ( r->x != 0.0f || r->y != 0.0f || r->angular != 0.0f ) )
			{
				b2SubStep_ApplyImpulse( context, f->bodies[i].id, ( b2Vec2 ){ r->x, r->y }, r->angular );
			}
		}
	}

	if ( f->def.contactPressure > 0.0f )
	{
		ApplyPatches( f, context );
	}
}
