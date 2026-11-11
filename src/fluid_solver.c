// SPDX-License-Identifier: MIT
// Double density relaxation (Clavet et al. 2005) with viscosity, run as parallel passes over sorted particles. Viscosity
// shares the move pass and the pairs the density pass found. Pressure can be kept from pulling, with cohesion as a
// separate pull. Liquids with springs get Clavet's springs and plasticity in a pass of their own before it.
#include "fluid_internal.h"

// The density excess that pressure acts on. Below rest density only the tensile share of it pulls.
static inline float TensileClamp( float excess, float tensile )
{
	return excess > 0.0f ? excess : tensile * excess;
}

// Densities and pressures for one particle use current positions. Also keeps each neighbour within reach for the move pass.
static inline void Density( b2fFluid* f, int i, float* density, float* nearDensity )
{
	float h = f->radius, h2 = h * h, inv = 1.0f / h;
	b2fCoord x = f->pos[i].x, y = f->pos[i].y;
	float a = 0.0f, b = 0.0f;
	b2fPair* pairs = f->pairs + B2F_PAIRS_PER_PARTICLE * i;
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

// Shapes cannot move out of the way, so the particle takes the whole of the move. Below rest
// density the pressure part is negative, but a wall does not pull on a particle. Adhesion does: Akinci et al. 2013's
// spline (Eq. 7), which pulls a particle between h/2 and h from the shape and peaks at 3h/4, scaled so that its peak
// is 1. Only shapes that forces do not move, static and kinematic, pull: at bodies the pull also lifted crates
// heavier than the liquid off the floor.
static inline void ApplyWallMoves( b2fFluid* f, int i, float pressure, float nearPressure, float adhesion, float h )
{
	float limit = 0.5f * f->size[i];
	b2fContact* contacts = f->contacts + B2F_CONTACTS_PER_PARTICLE * i;
	for ( int c = 0; c < f->contactCount[i]; ++c )
	{
		float move = pressure * contacts[c].pressureWeight + nearPressure * contacts[c].nearWeight;
		move = move < 0.0f ? 0.0f : ( move > limit ? limit : move );
		if ( adhesion > 0.0f && f->bodies[contacts[c].bodyIndex].adheres )
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

// With sizes that differ, a pair uses the mean size and weights the other particle by its area, which keeps
// moves equal and opposite in momentum. Each particle's own pressure scales with its own size squared.
static inline void AdaptiveDensity( b2fFluid* f, int i, float* density, float* nearDensity )
{
	b2fCoord x = f->pos[i].x, y = f->pos[i].y;
	float si = f->size[i], a = 0.0f, b = 0.0f;
	b2fPair* pairs = f->pairs + B2F_PAIRS_PER_PARTICLE * i;
	int n = 0;
	FOR_NEIGHBOURS( f, i, j )
	{
		float dx = (float)( f->pos[j].x - x ), dy = (float)( f->pos[j].y - y ), d2 = dx * dx + dy * dy, sj = f->size[j], h = si + sj;
		if ( d2 < h * h && j != i )
		{
			float mean = 0.5f * h, w = sj * sj / ( mean * mean );
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

static void AdaptiveAccumulate( b2fFluid* f, int start, int end )
{
	const b2LiquidDef* def = &f->liquids[0].def;
	float k = def->tuning.stiffness, kn = def->tuning.nearStiffness, rest = f->restDensity;
	float tensile = def->tuning.tensilePressure, cohesion = def->cohesion;
	float sigma = def->viscosity, beta = def->tuning.quadraticViscosity, dt = f->h;
	bool viscous = sigma > 0.0f || beta > 0.0f;
	for ( int i = start; i < end; ++i )
	{
		b2fCoord x = f->pos[i].x, y = f->pos[i].y;
		float si = f->size[i], moveX = 0.0f, moveY = 0.0f;
		float hi2 = 4.0f * si * si;
		float pressure = k * hi2 * TensileClamp( f->rho[i] - rest, tensile ), nearPressure = kn * hi2 * f->rhoNear[i];
		float vx = f->vx[i], vy = f->vy[i], dvx = 0.0f, dvy = 0.0f;
#define B2F_ADAPTIVE_PAIR( j, l )                                                                                    \
	{                                                                                                                  \
		float sj = f->size[j], h = si + sj;                                                                            \
		float q = 1.0f - l / h, mean = 0.5f * h, w = sj * sj / ( mean * mean ), hj2 = 4.0f * sj * sj;                \
		float pj = k * hj2 * TensileClamp( f->rho[j] - rest, tensile ), nj = kn * hj2 * f->rhoNear[j];                \
		float pull = cohesion * h * h * q * ( 1.0f - 2.0f * q );                                                       \
		float d = 0.5f * w * ( ( pressure + pj ) * q + ( nearPressure + nj ) * q * q - pull ) / ( h * l );            \
		moveX -= d * dx;                                                                                               \
		moveY -= d * dy;                                                                                               \
		float u = ( ( vx - f->vx[j] ) * dx + ( vy - f->vy[j] ) * dy ) / l;                                            \
		if ( viscous && u > 0.0f )                                                                                     \
		{                                                                                                              \
			float impulse = 0.5f * w * dt * q * ( sigma * u + beta * u * u ) / l;                                      \
			dvx -= impulse * dx;                                                                                       \
			dvy -= impulse * dy;                                                                                       \
		}                                                                                                              \
	}
		int n = f->pairCount[i];
		if ( n >= 0 )
		{
			const b2fPair* pairs = f->pairs + B2F_PAIRS_PER_PARTICLE * i;
			for ( int p = 0; p < n; ++p )
			{
				int j = pairs[p].j;
				float dx = (float)( f->pos[j].x - x ), dy = (float)( f->pos[j].y - y ), d2 = dx * dx + dy * dy, l = pairs[p].l;
				if ( d2 > 1e-12f )
				{
					B2F_ADAPTIVE_PAIR( j, l )
				}
			}
		}
		else
		{
			FOR_NEIGHBOURS( f, i, j )
			{
				float dx = (float)( f->pos[j].x - x ), dy = (float)( f->pos[j].y - y ), d2 = dx * dx + dy * dy, reach = si + f->size[j];
				if ( d2 < reach * reach && d2 > 1e-12f && j != i )
				{
					float l = sqrtf( d2 );
					B2F_ADAPTIVE_PAIR( j, l )
				}
			}
		}
#undef B2F_ADAPTIVE_PAIR
		f->dx[i] = moveX;
		f->dy[i] = moveY;
		f->viscX[i] = dvx;
		f->viscY[i] = dvy;
	}
}

// Several liquids. Pressures were stored by the density pass, each from its own liquid's settings. A pair takes its
// cohesion, viscosity and mass weight from the table for the two liquids, which keeps the pair's moves equal and
// opposite in momentum when the liquids differ in density. With one liquid the passes above give the same moves.
static void MultiAccumulate( b2fFluid* f, int start, int end )
{
	float dt = f->h;
	for ( int i = start; i < end; ++i )
	{
		b2fCoord x = f->pos[i].x, y = f->pos[i].y;
		float si = f->size[i], moveX = 0.0f, moveY = 0.0f;
		float pressure = f->pressure[i], nearPressure = f->nearPressure[i];
		float vx = f->vx[i], vy = f->vy[i], dvx = 0.0f, dvy = 0.0f;
		const b2fLiquidPair* row = f->liquidPairs[f->liquid[i]];
#define B2F_MULTI_PAIR( j, l )                                                                                       \
	{                                                                                                                  \
		const b2fLiquidPair* pair = row + f->liquid[j];                                                                \
		float sj = f->size[j], h = si + sj;                                                                            \
		float q = 1.0f - l / h, mean = 0.5f * h, w = pair->weight * sj * sj / ( mean * mean );                       \
		float pull = pair->cohesion * h * h * q * ( 1.0f - 2.0f * q );                                                 \
		float d = 0.5f * w * ( ( pressure + f->pressure[j] ) * q + ( nearPressure + f->nearPressure[j] ) * q * q - pull ) / ( h * l ); \
		moveX -= d * dx;                                                                                               \
		moveY -= d * dy;                                                                                               \
		float u = ( ( vx - f->vx[j] ) * dx + ( vy - f->vy[j] ) * dy ) / l;                                            \
		if ( u > 0.0f )                                                                                                \
		{                                                                                                              \
			float impulse = 0.5f * w * dt * q * ( pair->viscosity * u + pair->quadraticViscosity * u * u ) / l;  \
			dvx -= impulse * dx;                                                                                       \
			dvy -= impulse * dy;                                                                                       \
		}                                                                                                              \
	}
		int n = f->pairCount[i];
		if ( n >= 0 )
		{
			const b2fPair* pairs = f->pairs + B2F_PAIRS_PER_PARTICLE * i;
			for ( int p = 0; p < n; ++p )
			{
				int j = pairs[p].j;
				float dx = (float)( f->pos[j].x - x ), dy = (float)( f->pos[j].y - y ), d2 = dx * dx + dy * dy, l = pairs[p].l;
				if ( d2 > 1e-12f )
				{
					B2F_MULTI_PAIR( j, l )
				}
			}
		}
		else
		{
			FOR_NEIGHBOURS( f, i, j )
			{
				float dx = (float)( f->pos[j].x - x ), dy = (float)( f->pos[j].y - y ), d2 = dx * dx + dy * dy, reach = si + f->size[j];
				if ( d2 < reach * reach && d2 > 1e-12f && j != i )
				{
					float l = sqrtf( d2 );
					B2F_MULTI_PAIR( j, l )
				}
			}
		}
#undef B2F_MULTI_PAIR
		f->dx[i] = moveX;
		f->dy[i] = moveY;
		f->viscX[i] = dvx;
		f->viscY[i] = dvy;
	}
}

// After the density pass found a particle's neighbours, drops the springs the spring pass marked as gone, and adds one
// at the current distance to each neighbour of the same liquid it has none to. The neighbour does the same from its
// side. Clavet's Algorithm 4 starts a spring at the interaction radius, where it has no force until plasticity
// shortens it; starting at the distance instead lets an elastic liquid, with no plasticity, hold the shape it had.
static void AddSprings( b2fFluid* f, int i )
{
	b2fSprings* sp = f->springs + i;
	if ( f->liquids[f->liquid[i]].def.springStiffness <= 0.0f )
	{
		sp->count = 0;
		return;
	}

	int kept = 0;
	for ( int s = 0; s < sp->count; ++s )
	{
		if ( sp->rest[s] >= 0.0f )
		{
			sp->id[kept] = sp->id[s];
			sp->index[kept] = sp->index[s];
			sp->rest[kept] = sp->rest[s];
			kept += 1;
		}
	}
	sp->count = kept;

	int n = f->pairCount[i];
	const b2fPair* pairs = f->pairs + B2F_PAIRS_PER_PARTICLE * i;
	for ( int p = 0; p < n && sp->count < B2F_SPRINGS_PER_PARTICLE; ++p )
	{
		int j = pairs[p].j, id = f->id[j];
		if ( f->liquid[j] != f->liquid[i] )
		{
			continue;
		}

		bool found = false;
		for ( int s = 0; s < sp->count && found == false; ++s )
		{
			found = sp->id[s] == id;
		}
		if ( found == false )
		{
			sp->id[sp->count] = id;
			sp->index[sp->count] = j;
			sp->rest[sp->count] = pairs[p].l;
			sp->count += 1;
		}
	}
}

static void DensityTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	bool springy = f->springy;
	if ( f->liquidCount > 1 )
	{
		float rest = f->restDensity;
		for ( int i = start; i < end; ++i )
		{
			AdaptiveDensity( f, i, f->rho + i, f->rhoNear + i );
			const b2LiquidDef* def = &f->liquids[f->liquid[i]].def;
			float hi2 = 4.0f * f->size[i] * f->size[i];
			f->pressure[i] = def->tuning.stiffness * hi2 * TensileClamp( f->rho[i] - rest, def->tuning.tensilePressure );
			f->nearPressure[i] = def->tuning.nearStiffness * hi2 * f->rhoNear[i];
			if ( springy )
			{
				AddSprings( f, i );
			}
		}
		return;
	}

	if ( f->adaptive )
	{
		for ( int i = start; i < end; ++i )
		{
			AdaptiveDensity( f, i, f->rho + i, f->rhoNear + i );
			if ( springy )
			{
				AddSprings( f, i );
			}
		}
		return;
	}

	for ( int i = start; i < end; ++i )
	{
		Density( f, i, f->rho + i, f->rhoNear + i );
		if ( springy )
		{
			AddSprings( f, i );
		}
	}
}

static void AccumulateTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	if ( f->liquidCount > 1 )
	{
		MultiAccumulate( f, start, end );
		return;
	}

	if ( f->adaptive )
	{
		AdaptiveAccumulate( f, start, end );
		return;
	}

	const b2LiquidDef* def = &f->liquids[0].def;
	float h = f->radius, h2 = h * h, inv = 1.0f / h;
	float scale = def->tuning.stiffness * h, nearScale = def->tuning.nearStiffness * h, rest = 2.0f * f->restDensity;
	float tensile = def->tuning.tensilePressure, single = f->restDensity, pullScale = def->cohesion * h;
	bool clamped = tensile < 1.0f;
	float sigma = def->viscosity, beta = def->tuning.quadraticViscosity, dt = f->h;
	bool viscous = sigma > 0.0f || beta > 0.0f;
	for ( int i = start; i < end; ++i )
	{
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
			const b2fPair* pairs = f->pairs + B2F_PAIRS_PER_PARTICLE * i;
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

static void ApplyTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	float limit = f->moveLimit * f->radius, limit2 = limit * limit, dt = f->h;
	bool adaptive = f->adaptive;
	float restRatio = 1.0f + B2F_SQUEEZE_SURFACE, invRest = 1.0f / f->restDensity;
	for ( int i = start; i < end; ++i )
	{
		if ( adaptive )
		{
			limit = f->moveLimit * 2.0f * f->size[i];
			limit2 = limit * limit;
		}

		if ( limit > 0.0f )
		{
			float d2 = f->dx[i] * f->dx[i] + f->dy[i] * f->dy[i];
			if ( d2 > limit2 )
			{
				float scale = limit / sqrtf( d2 );
				f->dx[i] *= scale;
				f->dy[i] *= scale;
			}
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
			float h = adaptive ? 2.0f * f->size[i] : f->radius;
			float excess = TensileClamp( f->rho[i] - f->restDensity, def->tuning.tensilePressure );
			ApplyWallMoves( f, i, def->tuning.stiffness * h * excess, def->tuning.nearStiffness * h * f->rhoNear[i], def->adhesion, h );
		}
	}
}

static void PredictTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	float h = f->h, gx = h * f->gravityX, gy = h * f->gravityY;
	bool multi = f->liquidCount > 1;
	for ( int i = start; i < end; ++i )
	{
		if ( multi )
		{
			float scale = h * f->liquids[f->liquid[i]].def.gravityScale;
			gx = scale * f->worldGravity.x;
			gy = scale * f->worldGravity.y;
		}
		f->vx[i] += gx;
		f->vy[i] += gy;
		f->prev[i].x = f->pos[i].x;
		f->prev[i].y = f->pos[i].y;
		f->pos[i].x += h * f->vx[i];
		f->pos[i].y += h * f->vy[i];
	}
}

static void VelocityTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	float h = f->h, inv = 1.0f / h, limit = 0.5f * f->radius, limit2 = limit * limit;
	for ( int i = start; i < end; ++i )
	{
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
	}
}

// Springs, Clavet et al. 2005 sections 5.1 and 5.2. Each particle moves by its share of each spring's stretch,
// w_i / (w_i + w_j + 1 / (k dt^2)) as in one iteration of XPBD (Macklin et al. 2016), so a spring is as stiff at any
// substep count. w is the inverse of a particle's mass in finest particles, and k the liquid's spring stiffness
// times Clavet's 1 - L / h, which weakens springs as their rest length L nears the interaction radius h. Then the
// rest length follows the length by Clavet's equation 8, past the yield ratio, and a spring whose rest length
// reaches h is marked gone, as is one whose partner has gone, holds no spring back, or is now another liquid.
// Both ends read the same positions and rest length, so they work a spring out the same way.
static inline int FindParticle( const b2fFluid* f, int id )
{
	unsigned int slot = ( (unsigned int)id * 2654435761u ) & (unsigned int)f->idMapMask;
	while ( f->idMap[slot].id != -1 )
	{
		if ( f->idMap[slot].id == id )
		{
			return f->idMap[slot].index;
		}
		slot = ( slot + 1 ) & (unsigned int)f->idMapMask;
	}
	return -1;
}

static void SpringTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	float dt = f->h, spacing = f->spacing;
	bool adaptive = f->adaptive;
	int count = f->count;
	bool check = f->checkSprings;
	for ( int i = start; i < end; ++i )
	{
		float moveX = 0.0f, moveY = 0.0f;
		b2fSprings* sp = f->springs + i;
		const b2LiquidDef* def = &f->liquids[f->liquid[i]].def;
		if ( def->springStiffness > 0.0f )
		{
			b2fCoord x = f->pos[i].x, y = f->pos[i].y;
			float si = f->size[i], wi = spacing * spacing / ( si * si );
			int self = f->id[i];
			for ( int s = 0; s < sp->count; ++s )
			{
				float L = sp->rest[s];
				int j = -1;
				if ( L >= 0.0f )
				{
					int last = sp->index[s];
					j = last >= 0 && last < count ? f->newIndex[last] : -1;
					j = j >= 0 && j < count && f->id[j] == sp->id[s] ? j : FindParticle( f, sp->id[s] );
					sp->index[s] = j;
				}
				// Only the first substep of a step checks the partner holds the spring too, which costs as much as
				// the rest. A spring held at one end comes from a partner with no free slot, and lasts one step.
				bool mutual = j >= 0 && f->liquid[j] == f->liquid[i] && check == false;
				if ( j >= 0 && f->liquid[j] == f->liquid[i] && check )
				{
					const b2fSprings* other = f->springs + j;
					for ( int t = 0; t < other->count && mutual == false; ++t )
					{
						mutual = other->id[t] == self;
					}
				}
				if ( mutual == false )
				{
					sp->rest[s] = -1.0f;
					continue;
				}

				float sj = f->size[j], h = adaptive ? si + sj : f->radius;
				float dx = (float)( f->pos[j].x - x ), dy = (float)( f->pos[j].y - y ), r = sqrtf( dx * dx + dy * dy );
				float k = def->springStiffness * ( 1.0f - L / h );
				if ( k > 0.0f && r > 1e-6f )
				{
					float wj = spacing * spacing / ( sj * sj );
					float share = wi / ( wi + wj + 1.0f / ( k * dt * dt ) );
					float move = share * ( r - L ) / r;
					moveX += move * dx;
					moveY += move * dy;
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
				sp->rest[s] = L < h ? L : -1.0f;
			}
		}
		f->dx[i] = moveX;
		f->dy[i] = moveY;
	}
}

static void SpringApplyTask( int start, int end, int worker, void* context )
{
	(void)worker;
	b2fFluid* f = context;
	bool adaptive = f->adaptive;
	float limit = f->moveLimit * f->radius;
	for ( int i = start; i < end; ++i )
	{
		float cap = adaptive ? f->moveLimit * 2.0f * f->size[i] : limit;
		float d2 = f->dx[i] * f->dx[i] + f->dy[i] * f->dy[i], scale = d2 > cap * cap ? cap / sqrtf( d2 ) : 1.0f;
		f->pos[i].x += scale * f->dx[i];
		f->pos[i].y += scale * f->dy[i];
	}
}

void b2fRelaxSprings( b2fFluid* f, b2SubStepContext* context )
{
	// Where each id is now, in a table at least twice the particle count, filled in index order
	int slots = 1;
	while ( slots < 2 * f->count )
	{
		slots *= 2;
	}
	f->idMapMask = slots - 1;
	memset( f->idMap, 0xFF, (size_t)slots * sizeof( b2fIdSlot ) );
	for ( int i = 0; i < f->count; ++i )
	{
		unsigned int slot = ( (unsigned int)f->id[i] * 2654435761u ) & (unsigned int)f->idMapMask;
		while ( f->idMap[slot].id != -1 )
		{
			slot = ( slot + 1 ) & (unsigned int)f->idMapMask;
		}
		f->idMap[slot] = ( b2fIdSlot ){ f->id[i], i };
	}

	b2SubStep_ParallelFor( context, SpringTask, f->count, 512, f );
	b2SubStep_ParallelFor( context, SpringApplyTask, f->count, 2048, f );
}

void b2fPredict( b2fFluid* f, b2SubStepContext* context )
{
	b2SubStep_ParallelFor( context, PredictTask, f->count, 1024, f );
}

void b2fRelaxDensity( b2fFluid* f, b2SubStepContext* context )
{
	b2SubStep_ParallelFor( context, DensityTask, f->count, 512, f );
	b2SubStep_ParallelFor( context, AccumulateTask, f->count, 512, f );
	b2SubStep_ParallelFor( context, ApplyTask, f->count, 2048, f );
}

void b2fUpdateVelocities( b2fFluid* f, b2SubStepContext* context )
{
	b2SubStep_ParallelFor( context, VelocityTask, f->count, 2048, f );
}
