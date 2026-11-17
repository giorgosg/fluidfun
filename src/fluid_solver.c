// SPDX-License-Identifier: MIT
// Double density relaxation (Clavet et al. 2005) with viscosity, run as parallel passes over sorted particles. Viscosity
// shares the move pass and the pairs the density pass found. Pressure can be kept from pulling, with cohesion as a
// separate pull. Liquids with springs get Clavet's springs and plasticity in a pass of their own before it.
#include "fluid_internal.h"

#include "platform.h"

// The density excess that pressure acts on. Below rest density only the tensile share of it pulls.
static inline float TensileClamp( float excess, float tensile )
{
	return excess > 0.0f ? excess : tensile * excess;
}

// Densities for one particle, from current positions, keeping each neighbour within reach for the move pass. A pair
// uses the mean of the two sizes and weights the other particle by its area, which keeps moves equal and opposite in
// momentum when sizes differ. A neighbour of a liquid this particle's liquid does not push on is not counted and not
// kept, so no pair move, viscosity or cohesion acts between them either.
static inline void Density( b2fFluid* f, int i, b2fPair* pairs, float* density, float* nearDensity )
{
	b2fCoord x = f->pos[i].x, y = f->pos[i].y;
	float si = f->size[i], a = 0.0f, b = 0.0f;
	uint32_t sees = f->liquidSees[f->liquid[i]];
	bool all = sees == UINT32_MAX;
	int n = 0;
	FOR_NEIGHBOURS( f, i, j )
	{
		float dx = (float)( f->pos[j].x - x ), dy = (float)( f->pos[j].y - y ), d2 = dx * dx + dy * dy, sj = f->size[j], h = b2fLatticeSize( si + sj );
		if ( d2 < h * h && j != i && ( all || ( sees >> f->liquid[j] & 1u ) != 0 ) )
		{
			float mean = 0.5f * ( si + sj ), w = sj * sj / ( mean * mean );
			float l = sqrtf( d2 ), q = 1.0f - l / h;
			a += w * q * q;
			b += w * q * q * q;
			if ( n < B2F_PAIRS_PER_PARTICLE )
			{
				pairs[n] = ( b2fPair ){ j, l };
			}
			n += 1;
		}
	}
	f->pairCount[i] = n <= B2F_PAIRS_PER_PARTICLE ? n : -1;
	*density = a + f->wallDensity[i];
	*nearDensity = b + f->wallNearDensity[i];
}

// Shapes cannot move out of the way, so the particle takes the whole of the move. Below rest
// density the pressure part is negative, but a wall does not pull on a particle. Adhesion does: Akinci et al. 2013's
// spline (Eq. 7), which pulls a particle between h/2 and h from the shape and peaks at 3h/4, scaled so that its peak
// is 1. Only shapes that forces do not move, static and kinematic, pull: at bodies the pull also lifted crates
// heavier than the liquid off the floor.
static inline void ApplyWallMoves( b2fFluid* f, int i, float pressure, float nearPressure, float adhesion, float h )
{
	float limit = 0.5f * b2fLatticeSize( f->size[i] );
	b2fContact* contacts = f->contacts + B2F_CONTACTS_PER_PARTICLE * i;
	for ( int c = 0; c < f->contactCount[i]; ++c )
	{
		float move = pressure * contacts[c].pressureWeight + nearPressure * contacts[c].nearWeight;
		move = move < 0.0f ? 0.0f : ( move > limit ? limit : move );
		if ( adhesion > 0.0f && f->bodies.data[contacts[c].bodyIndex].adheres )
		{
			float d = ( 1.0f - contacts[c].share ) * h;
			float spline = -4.0f * d * d / h + 6.0f * d - 2.0f * h;
			if ( d > 0.5f * h && spline > 0.0f )
			{
				move -= B2F_ADHESION_SCALE * adhesion * h * sqrtf( sqrtf( spline / ( 0.25f * h ) ) );
			}
		}
		contacts[c].move = move;
		f->pos[i].x += move * contacts[c].normalX;
		f->pos[i].y += move * contacts[c].normalY;
	}
}

// After the density pass found a particle's neighbours, offers a spring to each neighbour of its liquid it has none
// with, which FormSprings makes once the pass is done: a spring forms in one place, with one rest length, for the pair.
// Of the two ends only the one with the lower id offers, so a pair is offered once. Clavet's Algorithm 4 starts a
// spring at the interaction radius, where it has no force until plasticity shortens it; FormSprings starts it at the
// current distance instead, which lets an elastic liquid, with no plasticity, hold the shape it had. The pass writes
// only this particle's offers, and reads springCount and the springs of the spring pass, which nothing changes in it.
static void OfferSprings( b2fFluid* f, int i, const b2fPair* pairs )
{
	// The shell is asleep and forms nothing, and FormSprings reads only the awake particles' offers
	if ( i >= f->activeCount )
	{
		return;
	}
	uint32_t offers = 0;
	const b2LiquidDef* def = &f->liquids[f->liquid[i]].def;
	// Asleep, its springs stay as they fell asleep; none forms between an asleep and an awake particle, which would join
	// an awake group to an asleep one
	if ( def->springStiffness > 0.0f && f->asleep[i] == 0 && f->springCount[i] < B2F_SPRINGS_PER_PARTICLE )
	{
		// A particle with more neighbours than the list holds (pairCount -1) offers to the ones the list has
		int n = f->pairCount[i] >= 0 ? f->pairCount[i] : B2F_PAIRS_PER_PARTICLE;
		// A liquid without formSpringsAfterSpawn forms springs only where one end is new
		bool forming = def->formSpringsAfterSpawn || f->springFormed[i] == false;
		int id = f->id[i], first = f->springStart.data[i], last = f->springStart.data[i + 1];
		const int* list = f->springList.data;
		const b2fSpringWork* work = f->springWork.data;
		for ( int p = 0; p < n; ++p )
		{
			int j = pairs[p].j;
			if ( f->id[j] < id || f->liquid[j] != f->liquid[i] || f->asleep[j] != 0 ||
				 f->springCount[j] >= B2F_SPRINGS_PER_PARTICLE || ( forming == false && f->springFormed[j] ) )
			{
				continue;
			}
			// Its springs between awake ends are all listed, since the spring pass worked them out
			bool found = false;
			for ( int e = first; e < last && found == false; ++e )
			{
				found = work[list[e]].a == j || work[list[e]].b == j;
			}
			offers |= found ? 0u : 1u << p;
		}
	}
	f->springOffers[i] = offers;
}

_Static_assert( B2F_PAIRS_PER_PARTICLE <= 32, "a spring offer is a bit of a 32-bit word for each pair" );

// The springs the density pass offered, made on one thread in particle order and each particle's pairs in order, so the
// table and its order do not depend on worker count. A spring forms only while both ends take part in fewer than
// B2F_SPRINGS_PER_PARTICLE springs, at the distance the pass found, the same for both ends.
static void FormSprings( b2fFluid* f )
{
	for ( int i = 0; i < f->activeCount; ++i )
	{
		uint32_t offers = f->springOffers[i];
		const b2fPair* pairs = f->pairs.data + B2F_PAIRS_PER_PARTICLE * i;
		while ( offers != 0 && f->springCount[i] < B2F_SPRINGS_PER_PARTICLE )
		{
			int p = (int)b2CTZ32( offers );
			offers &= offers - 1u;
			int j = pairs[p].j;
			if ( f->springCount[j] < B2F_SPRINGS_PER_PARTICLE )
			{
				b2fSpring spring = { f->id[i], f->id[j], pairs[p].l };
				b2Array_Push( f->springs, spring );
				f->springCount[i] += 1;
				f->springCount[j] += 1;
			}
		}
	}
}

// The same for one liquid with uniform sizes, where every pair's reach is the interaction radius and weighs 1. It
// gives the same densities up to rounding, and is kept because it is faster: without it and its move pass below, the two passes
// took 12% longer in the Tank and 17% longer in the Pool with uniform sizes.
static inline void UniformDensity( b2fFluid* f, int i, b2fPair* pairs, float* density, float* nearDensity )
{
	float h = f->radius, h2 = h * h, inv = 1.0f / h;
	b2fCoord x = f->pos[i].x, y = f->pos[i].y;
	float a = 0.0f, b = 0.0f;
	int n = 0;
	FOR_NEIGHBOURS( f, i, j )
	{
		float dx = (float)( f->pos[j].x - x ), dy = (float)( f->pos[j].y - y ), d2 = dx * dx + dy * dy;
		if ( d2 < h2 && j != i )
		{
			float l = sqrtf( d2 ), q = 1.0f - l * inv;
			float q2 = q * q;
			a += q2;
			b += q2 * q;
			if ( n < B2F_PAIRS_PER_PARTICLE )
			{
				pairs[n] = ( b2fPair ){ j, l };
			}
			n += 1;
		}
	}
	f->pairCount[i] = n <= B2F_PAIRS_PER_PARTICLE ? n : -1;
	*density = a + f->wallDensity[i];
	*nearDensity = b + f->wallNearDensity[i];
}

// One liquid with uniform sizes runs its own density and move passes
static inline bool UsesUniformPasses( const b2fFluid* f )
{
	return f->liquidCount <= 1 && f->adaptive == false;
}

// The move pass for one liquid with uniform sizes. It gives the same moves as the general pass up to rounding, faster.
static void UniformAccumulate( b2fFluid* f, int start, int end )
{
	const b2LiquidDef* def = &f->liquids[0].def;
	float h = f->radius, h2 = h * h, inv = 1.0f / h;
	float scale = def->tuning.stiffness * h, nearScale = def->tuning.nearStiffness * h, rest = 2.0f * f->restDensity;
	float tensile = def->tuning.tensilePressure, single = f->restDensity, pullScale = def->cohesion * h;
	bool clamped = tensile < 1.0f;
	float sigma = def->viscosity, beta = def->tuning.quadraticViscosity, dt = f->h;
	bool viscous = sigma > 0.0f || beta > 0.0f;
	for ( int i = start; i < end; ++i )
	{
		if ( f->asleep[i] != 0 )
		{
			continue;
		}
		b2fCoord x = f->pos[i].x, y = f->pos[i].y;
		float moveX = 0.0f, moveY = 0.0f;
		float rhoI = f->rho[i], nearI = f->rhoNear[i], pressureI = TensileClamp( rhoI - single, tensile );
		float vx = f->vx[i], vy = f->vy[i], dvx = 0.0f, dvy = 0.0f;
#define B2F_UNIFORM_PAIR( j, l )                                                                                     \
	{                                                                                                                  \
		float q = 1.0f - l * inv;                                                                                      \
		float excess = clamped ? pressureI + TensileClamp( f->rho[j] - single, tensile ) : rhoI + f->rho[j] - rest;   \
		float pull = pullScale * q * ( 1.0f - 2.0f * q );                                                              \
		float d = 0.5f * ( scale * excess * q + nearScale * ( nearI + f->rhoNear[j] ) * q * q - pull ) / l;            \
		moveX -= d * dx;                                                                                               \
		moveY -= d * dy;                                                                                               \
		float u = ( ( vx - f->vx[j] ) * dx + ( vy - f->vy[j] ) * dy ) / l;                                            \
		if ( viscous && u > 0.0f )                                                                                     \
		{                                                                                                              \
			float impulse = 0.5f * dt * q * ( sigma * u + beta * u * u ) / l;                                          \
			dvx -= impulse * dx;                                                                                       \
			dvy -= impulse * dy;                                                                                       \
		}                                                                                                              \
	}
		int n = f->pairCount[i];
		if ( n >= 0 )
		{
			const b2fPair* pairs = f->pairs.data + B2F_PAIRS_PER_PARTICLE * i;
			for ( int p = 0; p < n; ++p )
			{
				int j = pairs[p].j;
				float dx = (float)( f->pos[j].x - x ), dy = (float)( f->pos[j].y - y ), d2 = dx * dx + dy * dy, l = pairs[p].l;
				if ( d2 > 1e-12f )
				{
					B2F_UNIFORM_PAIR( j, l )
				}
			}
		}
		else
		{
			FOR_NEIGHBOURS( f, i, j )
			{
				float dx = (float)( f->pos[j].x - x ), dy = (float)( f->pos[j].y - y ), d2 = dx * dx + dy * dy;
				if ( d2 < h2 && d2 > 1e-12f && j != i )
				{
					float l = sqrtf( d2 );
					B2F_UNIFORM_PAIR( j, l )
				}
			}
		}
#undef B2F_UNIFORM_PAIR
		f->dx[i] = moveX;
		f->dy[i] = moveY;
		f->viscX[i] = dvx;
		f->viscY[i] = dvy;
	}
}

// Whether a shell particle, asleep beside awake liquid or against a shape, should wake: its density has moved from what
// it was in its first substep asleep, or an awake neighbour within reach is fast (fluid_sleep.c)
static bool ShellWakes( b2fFluid* f, int i, const b2fPair* pairs )
{
	// Just asleep, this density is the one it wakes by. The bit goes at the end of the substep, since neighbours read
	// asleep in this pass.
	if ( f->asleep[i] & B2F_ASLEEP_NEW )
	{
		f->calm[i].x = f->rho[i];
	}
	else if ( b2AbsFloat( f->rho[i] - f->calm[i].x ) > B2F_WAKE_DENSITY * f->restDensity )
	{
		return true;
	}

	float fast2 = B2F_WAKE_SPEED * B2F_WAKE_SPEED;
	int n = f->pairCount[i];
	if ( n >= 0 )
	{
		for ( int p = 0; p < n; ++p )
		{
			int j = pairs[p].j;
			if ( f->asleep[j] == 0 && f->vx[j] * f->vx[j] + f->vy[j] * f->vy[j] > fast2 )
			{
				return true;
			}
		}
		return false;
	}

	// Too many to keep: any in the window counts
	FOR_NEIGHBOURS( f, i, j )
	{
		if ( f->asleep[j] == 0 && f->vx[j] * f->vx[j] + f->vy[j] * f->vy[j] > fast2 )
		{
			return true;
		}
	}
	return false;
}

// The density pass runs over the awake particles and then the shell; the other asleep particles skip it, and keep the
// density they last had
static inline int DensityParticle( const b2fFluid* f, int k )
{
	return k < f->activeCount ? k : f->shell[k - f->activeCount];
}

// Each particle's pressure and near pressure, from its own liquid's settings, scale with its size squared
static void DensityTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	bool springy = f->springy;
	if ( UsesUniformPasses( f ) )
	{
		for ( int k = start; k < end; ++k )
		{
			int i = DensityParticle( f, k );
			b2fPair* pairs = f->pairs.data + B2F_PAIRS_PER_PARTICLE * k;
			UniformDensity( f, i, pairs, f->rho + i, f->rhoNear + i );
			if ( springy )
			{
				OfferSprings( f, i, pairs );
			}
			if ( f->asleep[i] != 0 && ShellWakes( f, i, pairs ) )
			{
				f->wake[i] |= B2F_WAKE_NOW;
			}
		}
		return;
	}

	float rest = f->restDensity;
	for ( int k = start; k < end; ++k )
	{
		int i = DensityParticle( f, k );
		b2fPair* pairs = f->pairs.data + B2F_PAIRS_PER_PARTICLE * k;
		Density( f, i, pairs, f->rho + i, f->rhoNear + i );
		const b2LiquidDef* def = &f->liquids[f->liquid[i]].def;
		float hi = 2.0f * b2fLatticeSize( f->size[i] ), hi2 = hi * hi;
		f->pressure[i] = def->tuning.stiffness * hi2 * TensileClamp( f->rho[i] - rest, def->tuning.tensilePressure );
		f->nearPressure[i] = def->tuning.nearStiffness * hi2 * f->rhoNear[i];
		if ( springy )
		{
			OfferSprings( f, i, pairs );
		}
		if ( f->asleep[i] != 0 && ShellWakes( f, i, pairs ) )
		{
			f->wake[i] |= B2F_WAKE_NOW;
		}
	}
}

// The move from pressure, near pressure and cohesion, and the change of velocity from viscosity. A pair takes its
// cohesion, viscosity and mass weight from the table for the two liquids, which keeps the pair's moves equal and
// opposite in momentum when the liquids differ in density.
static void AccumulateTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	if ( UsesUniformPasses( f ) )
	{
		UniformAccumulate( f, start, end );
		return;
	}

	float dt = f->h;
	for ( int i = start; i < end; ++i )
	{
		if ( f->asleep[i] != 0 )
		{
			continue;
		}
		b2fCoord x = f->pos[i].x, y = f->pos[i].y;
		float si = f->size[i], moveX = 0.0f, moveY = 0.0f;
		float pressure = f->pressure[i], nearPressure = f->nearPressure[i];
		float vx = f->vx[i], vy = f->vy[i], dvx = 0.0f, dvy = 0.0f;
		const b2fLiquidPair* row = f->liquidPairs[f->liquid[i]];
		// The density pass kept only neighbours this particle's liquid pushes on; a particle with too many to keep
		// scans the grid, and skips the others here
		uint32_t sees = f->liquidSees[f->liquid[i]];
		bool all = sees == UINT32_MAX;
#define B2F_PAIR( j, l )                                                                                             \
	{                                                                                                                  \
		const b2fLiquidPair* pair = row + f->liquid[j];                                                                \
		float sj = f->size[j], h = b2fLatticeSize( si + sj );                                                          \
		float q = 1.0f - l / h, mean = 0.5f * ( si + sj ), w = pair->weight * sj * sj / ( mean * mean );             \
		float pull = pair->cohesion * h * h * q * ( 1.0f - 2.0f * q );                                                 \
		float d = 0.5f * w * ( ( pressure + f->pressure[j] ) * q + ( nearPressure + f->nearPressure[j] ) * q * q - pull ) / ( h * l ); \
		moveX -= d * dx;                                                                                               \
		moveY -= d * dy;                                                                                               \
		float u = ( ( vx - f->vx[j] ) * dx + ( vy - f->vy[j] ) * dy ) / l;                                            \
		if ( u > 0.0f )                                                                                                \
		{                                                                                                              \
			float impulse = 0.5f * w * dt * q * ( pair->viscosity * u + pair->quadraticViscosity * u * u ) / l;          \
			dvx -= impulse * dx;                                                                                       \
			dvy -= impulse * dy;                                                                                       \
		}                                                                                                              \
	}
		int n = f->pairCount[i];
		if ( n >= 0 )
		{
			const b2fPair* pairs = f->pairs.data + B2F_PAIRS_PER_PARTICLE * i;
			for ( int p = 0; p < n; ++p )
			{
				int j = pairs[p].j;
				float dx = (float)( f->pos[j].x - x ), dy = (float)( f->pos[j].y - y ), d2 = dx * dx + dy * dy, l = pairs[p].l;
				if ( d2 > 1e-12f )
				{
					B2F_PAIR( j, l )
				}
			}
		}
		else
		{
			FOR_NEIGHBOURS( f, i, j )
			{
				float dx = (float)( f->pos[j].x - x ), dy = (float)( f->pos[j].y - y ), d2 = dx * dx + dy * dy, reach = b2fLatticeSize( si + f->size[j] );
				if ( d2 < reach * reach && d2 > 1e-12f && j != i && ( all || ( sees >> f->liquid[j] & 1u ) != 0 ) )
				{
					float l = sqrtf( d2 );
					B2F_PAIR( j, l )
				}
			}
		}
#undef B2F_PAIR
		f->dx[i] = moveX;
		f->dy[i] = moveY;
		f->viscX[i] = dvx;
		f->viscY[i] = dvy;
	}
}

static void ApplyTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	float dt = f->h;
	float restRatio = 1.0f + B2F_SQUEEZE_SURFACE, invRest = 1.0f / f->restDensity;
	for ( int i = start; i < end; ++i )
	{
		if ( f->asleep[i] != 0 )
		{
			continue;
		}
		float radius = 2.0f * b2fLatticeSize( f->size[i] ), limit = B2F_MOVE_LIMIT * radius;
		float d2 = f->dx[i] * f->dx[i] + f->dy[i] * f->dy[i];
		if ( d2 > limit * limit )
		{
			float scale = limit / sqrtf( d2 );
			f->dx[i] *= scale;
			f->dy[i] *= scale;
		}
		// Viscosity changes velocity, and the position follows it for the rest of the substep. Velocity itself is
		// derived from the change in position at the end of the substep, so only the position needs the change.
		f->pos[i].x += f->dx[i] + dt * f->viscX[i];
		f->pos[i].y += f->dy[i] + dt * f->viscY[i];

		// A particle weighs what the area it fills would weigh at rest. Fine particles deep down pack up to a third
		// tighter than coarse ones or ones near the surface, so by count the liquid would be heavier there, and a
		// body would float or sink by where it is. Prediction moved the particle by its full weight; this takes back
		// the share its packing adds. Looser than rest, as in spray, it keeps its full weight.
		float squeeze = f->rho[i] * invRest - restRatio;
		if ( squeeze > 0.0f )
		{
			float extra = B2F_FILL_SLOPE * squeeze;
			float lighter = extra / ( B2F_REST_FILL + extra ) * dt * dt * f->liquids[f->liquid[i]].def.gravityScale;
			f->pos[i].x -= lighter * f->worldGravity.x;
			f->pos[i].y -= lighter * f->worldGravity.y;
		}
		if ( f->contactCount[i] > 0 )
		{
			const b2LiquidDef* def = &f->liquids[f->liquid[i]].def;
			float excess = TensileClamp( f->rho[i] - f->restDensity, def->tuning.tensilePressure );
			ApplyWallMoves( f, i, def->tuning.stiffness * radius * excess, def->tuning.nearStiffness * radius * f->rhoNear[i], def->adhesion, radius );
		}
	}
}

static void PredictTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	float h = f->h;
	for ( int i = start; i < end; ++i )
	{
		// Asleep, it holds still, at zero velocity
		if ( f->asleep[i] != 0 )
		{
			f->prev[i] = f->pos[i];
			continue;
		}
		float scale = h * f->liquids[f->liquid[i]].def.gravityScale;
		f->vx[i] += scale * f->worldGravity.x;
		f->vy[i] += scale * f->worldGravity.y;
		f->prev[i].x = f->pos[i].x;
		f->prev[i].y = f->pos[i].y;
		f->pos[i].x += h * f->vx[i];
		f->pos[i].y += h * f->vy[i];
	}
}

// At the end of the substep the shell's particles that fell asleep in the last one are no longer new, and those to wake
// wake. One woken by touching an awake body (ContactTask) is awake already, and takes part from the next substep.
static void ShellEndTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	for ( int k = start; k < end; ++k )
	{
		int i = f->shell[k];
		if ( f->asleep[i] != 0 )
		{
			f->asleep[i] = (unsigned char)( f->asleep[i] & ~B2F_ASLEEP_NEW );
			if ( f->wake[i] & B2F_WAKE_NOW )
			{
				b2fWakeParticle( f, i );
			}
		}
	}
}

static void VelocityTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	float h = f->h, inv = 1.0f / h, limit = 0.5f * f->radius, limit2 = limit * limit;
	float sleep2 = B2F_SLEEP_SPEED * B2F_SLEEP_SPEED, smooth = h < B2F_CALM_TIME ? h / B2F_CALM_TIME : 1.0f;
	uint32_t springLiquids = f->springLiquids;
	bool sleepOn = f->sleepOn;
	for ( int i = start; i < end; ++i )
	{
		if ( f->asleep[i] != 0 )
		{
			f->asleep[i] = (unsigned char)( f->asleep[i] & ~B2F_ASLEEP_NEW );
			if ( f->wake[i] & B2F_WAKE_NOW )
			{
				b2fWakeParticle( f, i );
			}
			continue;
		}

		float dx = (float)( f->pos[i].x - f->prev[i].x ), dy = (float)( f->pos[i].y - f->prev[i].y ), d2 = dx * dx + dy * dy;
		if ( d2 > limit2 )
		{
			float scale = limit / sqrtf( d2 );
			dx *= scale;
			dy *= scale;
			f->pos[i].x = f->prev[i].x + dx;
			f->pos[i].y = f->prev[i].y + dy;
		}
		f->vx[i] = dx * inv;
		f->vy[i] = dy * inv;
		if ( f->springFormed != NULL )
		{
			f->springFormed[i] = true;
		}

		// Slow for B2_TIME_TO_SLEEP, by its velocity smoothed over B2F_CALM_TIME, and touching no awake body, it falls
		// asleep where it is, or with springs, once all its group is ready too (b2fSpringGroups)
		if ( sleepOn == false )
		{
			f->sleepTime[i] = 0.0f;
			continue;
		}
		b2Vec2 calm = f->calm[i];
		calm.x += smooth * ( f->vx[i] - calm.x );
		calm.y += smooth * ( f->vy[i] - calm.y );
		f->calm[i] = calm;
		f->sleepTime[i] = b2LengthSquared( calm ) < sleep2 ? f->sleepTime[i] + h : 0.0f;
		bool ready = f->sleepTime[i] >= B2_TIME_TO_SLEEP && ( f->wake[i] & B2F_WAKE_HELD ) == 0;
		if ( ready && ( springLiquids >> f->liquid[i] & 1u ) != 0 )
		{
			f->wake[i] |= B2F_WAKE_CALM;
		}
		else if ( ready )
		{
			f->asleep[i] = (unsigned char)( B2F_ASLEEP | B2F_ASLEEP_NEW | ( f->contactCount[i] > 0 ? B2F_ASLEEP_SHAPE : 0u ) );
			f->vx[i] = 0.0f;
			f->vy[i] = 0.0f;
			f->calm[i] = b2Vec2_zero;
		}
	}
}

// Springs, Clavet et al. 2005 sections 5.1 and 5.2, a spring at a time. Each end moves by its share of the spring's
// stretch, w_i / (w_a + w_b + 1 / (k dt^2)) as in one iteration of XPBD (Macklin et al. 2016), so a spring is as stiff
// at any substep count. w is the inverse of a particle's mass in finest particles, and k the liquid's spring stiffness
// times Clavet's 1 - L / h, which weakens springs as their rest length L nears the interaction radius h. Then the rest
// length follows the length by Clavet's equation 8, past the yield ratio, and a spring whose rest length reaches h
// goes, after pulling this once. One whose end has gone, or is of another liquid, or of one without springs, goes at
// once. Each task writes only its own springs, and the moves are summed per particle afterwards (SpringApplyTask).
//
// A spring with both ends asleep is skipped: neither moves, so neither its pull nor its rest length would change.
// Particles joined by springs fall asleep and wake together (b2fSpringGroups), so the ends of a spring are never one
// asleep and one awake; if they were, it would be skipped too.
static void SpringTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	float dt = f->h, fine = f->def.particleSpacing;
	b2fSpring* springs = f->springs.data;
	b2fSpringWork* work = f->springWork.data;
	for ( int s = start; s < end; ++s )
	{
		b2fSpring* spring = springs + s;
		int a = b2fFindParticle( f, spring->a ), b = b2fFindParticle( f, spring->b );
		const b2LiquidDef* def = a >= 0 ? &f->liquids[f->liquid[a]].def : NULL;
		if ( b < 0 || def == NULL || f->liquid[b] != f->liquid[a] || def->springStiffness <= 0.0f )
		{
			spring->rest = -1.0f;
			work[s] = ( b2fSpringWork ){ -1, -1, 0.0f, 0.0f };
			continue;
		}
		work[s] = ( b2fSpringWork ){ a, b, 0.0f, 0.0f };
		if ( f->asleep[a] != 0 || f->asleep[b] != 0 )
		{
			continue;
		}

		float sa = f->size[a], sb = f->size[b], h = b2fLatticeSize( sa + sb );
		float dx = (float)( f->pos[b].x - f->pos[a].x ), dy = (float)( f->pos[b].y - f->pos[a].y ), r = sqrtf( dx * dx + dy * dy );
		float L = spring->rest;
		float k = def->springStiffness * ( 1.0f - L / h );
		if ( k > 0.0f && r > 1e-6f )
		{
			float wa = fine * fine / ( sa * sa ), wb = fine * fine / ( sb * sb );
			float move = ( r - L ) / ( r * ( wa + wb + 1.0f / ( k * dt * dt ) ) );
			work[s].moveX = move * dx;
			work[s].moveY = move * dy;
		}

		float yield = def->yieldRatio * L;
		if ( r > L + yield )
		{
			L += dt * def->plasticity * ( r - L - yield );
		}
		else if ( r < L - yield )
		{
			L -= dt * def->plasticity * ( L - yield - r );
		}
		spring->rest = L < h ? L : -1.0f;
	}
}

// On one thread, after SpringTask: removes the springs that went from the table, keeping the order of the rest; counts
// the springs each particle takes part in, for the offers of the density pass; and lists, for each awake particle, the
// springs that pulled it in this substep, in table order, so that its moves are summed in an order that does not depend
// on worker count. The list counts into springStart shifted by two and is filled shifted by one, which leaves particle
// i's springs from springStart[i] to springStart[i + 1].
static void ListSprings( b2fFluid* f )
{
	int active = f->activeCount, count = f->springs.count;
	memset( f->springCount, 0, (size_t)f->count );
	b2fArray_ResizeGrow( f->springStart, active + 2 );
	int* start = f->springStart.data;
	memset( start, 0, (size_t)( active + 2 ) * sizeof( int ) );
	b2fSpring* springs = f->springs.data;
	const b2fSpringWork* work = f->springWork.data;
	int kept = 0;
	for ( int s = 0; s < count; ++s )
	{
		int a = work[s].a, b = work[s].b;
		if ( a < 0 )
		{
			continue;
		}
		if ( f->asleep[a] == 0 && f->asleep[b] == 0 )
		{
			start[a + 2] += a < active ? 1 : 0;
			start[b + 2] += b < active ? 1 : 0;
		}
		if ( springs[s].rest >= 0.0f )
		{
			springs[kept] = springs[s];
			kept += 1;
			f->springCount[a] += 1;
			f->springCount[b] += 1;
		}
	}
	f->springs.count = kept;

	for ( int i = 2; i < active + 2; ++i )
	{
		start[i] += start[i - 1];
	}
	b2fArray_ResizeGrow( f->springList, start[active + 1] );
	int* list = f->springList.data;
	for ( int s = 0; s < count; ++s )
	{
		int a = work[s].a, b = work[s].b;
		if ( a >= 0 && f->asleep[a] == 0 && f->asleep[b] == 0 )
		{
			if ( a < active )
			{
				list[start[a + 1]++] = s;
			}
			if ( b < active )
			{
				list[start[b + 1]++] = s;
			}
		}
	}
}

// Sums each particle's moves from its springs, its first end's move times its inverse mass and its second end's the
// opposite, and moves it by them, capped as density's moves are. Nothing here reads another particle's position.
static void SpringApplyTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	float fine = f->def.particleSpacing;
	const int* first = f->springStart.data;
	const int* list = f->springList.data;
	const b2fSpringWork* work = f->springWork.data;
	for ( int i = start; i < end; ++i )
	{
		if ( first[i] == first[i + 1] )
		{
			continue;
		}
		float si = f->size[i], wi = fine * fine / ( si * si ), moveX = 0.0f, moveY = 0.0f;
		for ( int e = first[i]; e < first[i + 1]; ++e )
		{
			const b2fSpringWork* w = work + list[e];
			float share = w->a == i ? wi : -wi;
			moveX += share * w->moveX;
			moveY += share * w->moveY;
		}
		float cap = B2F_MOVE_LIMIT * 2.0f * b2fLatticeSize( si );
		float d2 = moveX * moveX + moveY * moveY, scale = d2 > cap * cap ? cap / sqrtf( d2 ) : 1.0f;
		f->pos[i].x += scale * moveX;
		f->pos[i].y += scale * moveY;
	}
}

// Where each id is now, in a table at least twice the particle count, filled in index order
void b2fBuildIdMap( b2fFluid* f )
{
	int slots = 2, bits = 1;
	while ( slots < 2 * f->count )
	{
		slots *= 2;
		bits += 1;
	}
	f->idMapMask = slots - 1;
	f->idMapShift = 32 - bits;
	memset( f->idMap, 0xFF, (size_t)slots * sizeof( b2fIdSlot ) );
	for ( int i = 0; i < f->count; ++i )
	{
		unsigned int slot = b2fIdHash( f, f->id[i] );
		while ( f->idMap[slot].id != -1 )
		{
			slot = ( slot + 1 ) & (unsigned int)f->idMapMask;
		}
		f->idMap[slot] = ( b2fIdSlot ){ f->id[i], i };
	}
}

void b2fRelaxSprings( b2fFluid* f, b2SubStepContext* context )
{
	b2fBuildIdMap( f );
	// Particles that woke since the last substep, touching an awake body in the contact pass among them, wake their
	// groups before any spring is worked out
	if ( f->sleepOn || f->springAsleep != 0 )
	{
		uint64_t ticks = b2GetTicks();
		b2fSpringGroups( f, false );
		f->profile.sleep += b2GetMilliseconds( ticks );
	}
	b2fArray_ResizeGrow( f->springWork, f->springs.count );
	b2SubStep_ParallelFor( context, SpringTask, f->springs.count, 512, f );
	ListSprings( f );
	b2SubStep_ParallelFor( context, SpringApplyTask, f->activeCount, 1024, f );
}

void b2fPredict( b2fFluid* f, b2SubStepContext* context )
{
	// Just filed, the awake particles are not yet first
	b2SubStep_ParallelFor( context, PredictTask, f->filePending ? f->count : f->activeCount, 1024, f );
}

// Makes room for the pairs of the particles this call covers. Short of room, it frees the array, since nothing in it
// is kept, and makes one a quarter larger than needed so that a slowly growing liquid does not make a new one every few
// substeps, but never larger than the particles' room, B2F_PAIRS_PER_PARTICLE for each particle of capacity, which
// follows the room as b2fGrow grows it. The array never shrinks, as the particles' room does not.
static void GrowPairs( b2fFluid* f )
{
	int needed = B2F_PAIRS_PER_PARTICLE * ( f->activeCount + f->shellCount );
	if ( needed > f->pairs.capacity )
	{
		int room = B2F_PAIRS_PER_PARTICLE * f->capacity;
		int capacity = b2MaxInt( b2MinInt( needed + needed / 4, room ), needed );
		b2Free( f->pairs.data, (size_t)f->pairs.capacity * sizeof( b2fPair ) );
		f->pairs.data = b2Alloc( (size_t)capacity * sizeof( b2fPair ) );
		f->pairs.capacity = capacity;
	}
	f->pairs.count = needed;
}

// The pairs are scratch for this call, grown here on the thread that runs the substep, outside the parallel loops. Not
// on the world's arena: the substep runs inside the solver's tasks, while b2SplitIslandTask may be allocating from it on
// another thread.
void b2fRelaxDensity( b2fFluid* f, b2SubStepContext* context )
{
	GrowPairs( f );
	b2SubStep_ParallelFor( context, DensityTask, f->activeCount + f->shellCount, 256, f );
	if ( f->springy )
	{
		FormSprings( f );
	}
	b2SubStep_ParallelFor( context, AccumulateTask, f->activeCount, 256, f );
	b2SubStep_ParallelFor( context, ApplyTask, f->activeCount, 2048, f );
}

void b2fUpdateVelocities( b2fFluid* f, b2SubStepContext* context )
{
	b2SubStep_ParallelFor( context, VelocityTask, f->activeCount, 2048, f );
	uint64_t ticks = b2GetTicks();
	b2SubStep_ParallelFor( context, ShellEndTask, f->shellCount, 2048, f );
	// Groups whose particles are all ready fall asleep together, and those of which some woke in this substep wake
	// together, before the next substep works out their springs
	if ( f->springy && ( f->sleepOn || f->springAsleep != 0 ) )
	{
		b2fSpringGroups( f, true );
	}
	if ( f->sleepOn || f->asleepCount > 0 )
	{
		b2fCountAsleep( f );
	}
	f->profile.sleep += b2GetMilliseconds( ticks );
}
