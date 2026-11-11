// SPDX-License-Identifier: MIT
#include "draw.h"
#include "sample.h"
#include "scene.h"

#include <imgui.h>

class SampleFluid : public Sample
{
public:
	SampleFluid( SampleContext* context, LabSceneType type )
		: Sample( context )
	{
		static bool first = true;
		if ( first )
		{
			// Each scene keeps its own settings, starting from the liquid it was made to show
			for ( int i = 0; i < lab_sceneCount; ++i )
			{
				s_scenes[i] = LabDefaultScene( (LabSceneType)i );
				s_defs[i] = b2DefaultFluidDef();
				s_defs[i].capacity = 2000000;
				LabSceneSettings( (LabSceneType)i, s_defs + i );
			}
			first = false;
		}

		m_scene = &s_scenes[type];
		m_def = &s_defs[type];
		if ( m_context->restart == false )
		{
			m_context->camera.center = { m_scene->viewCenter.x, m_scene->viewCenter.y };
			m_context->camera.zoom = m_scene->viewHalfWidth;
			// Only on opening the scene, so the Sub-steps slider still holds across restarts
			if ( m_scene->subSteps > 0 )
			{
				m_context->subStepCount = m_scene->subSteps;
			}
		}

		m_fluid = LabCreateScene( m_worldId, m_scene, m_def );
	}

	~SampleFluid() override
	{
		b2DestroyFluid( m_fluid );
	}

	void Step() override
	{
		if ( m_context->pause == false || m_context->singleStep > 0 )
		{
			LabStepScene( m_scene, m_fluid, 1.0f / m_context->hertz );
		}

		Sample::Step();

		int n = b2Fluid_GetParticleCount( m_fluid );
		const b2Pos* pos = b2Fluid_GetPositions( m_fluid );
		const float* vx = b2Fluid_GetVelocitiesX( m_fluid );
		const float* vy = b2Fluid_GetVelocitiesY( m_fluid );
		const float* size = b2Fluid_GetSizes( m_fluid );
		const unsigned char* liquid = b2Fluid_GetLiquids( m_fluid );

		// Liquid 0 is blue, then amber and red. Speed lightens each toward white.
		static const float tints[3][3] = { { 40.0f, 110.0f, 255.0f }, { 230.0f, 170.0f, 40.0f }, { 200.0f, 40.0f, 60.0f } };
		for ( int i = 0; i < n; ++i )
		{
			float scale = size[i] / m_def->particleSpacing;
			float speed = b2MinFloat( b2Length( { vx[i], vy[i] } ) / 8.0f, 1.0f );
			const float* tint = liquid[i] == 0 && m_scene->color[0] + m_scene->color[1] + m_scene->color[2] > 0.0f ? m_scene->color : tints[liquid[i] % 3];
			uint32_t red = (uint32_t)( tint[0] + ( 255.0f - tint[0] ) * speed );
			uint32_t green = (uint32_t)( tint[1] + ( 255.0f - tint[1] ) * speed );
			uint32_t blue = (uint32_t)( tint[2] + ( 255.0f - tint[2] ) * speed );
			if ( scale > 1.05f && liquid[i] == 0 )
			{
				green = b2MinInt( 255, (int)( (float)green + 60.0f * scale ) );
			}
			DrawPoint( m_context->draw, pos[i], m_pointSize * scale, b2HexColor( ( red << 16 ) | ( green << 8 ) | blue ) );
		}
	}

	bool DrawControls() override
	{
		ImGui::Text( "Particles %d", b2Fluid_GetParticleCount( m_fluid ) );
		b2FluidProfile p = b2Fluid_GetProfile( m_fluid );
		ImGui::Text( "Fluid %.2f ms", p.total );
		ImGui::Text( "sort %.2f", p.sort );
		ImGui::Text( "density %.2f collide %.2f", p.density, p.collide );

		bool changed = false;
		changed |= ImGui::SliderFloat( "Stiffness", &m_def->liquid.stiffness, 0.0f, 0.2f, "%.3f" );
		changed |= ImGui::SliderFloat( "Near", &m_def->liquid.nearStiffness, 0.0f, 0.5f, "%.3f" );
		changed |= ImGui::SliderFloat( "Tensile pressure", &m_def->liquid.tensilePressure, 0.0f, 1.0f, "%.2f" );
		changed |= ImGui::SliderFloat( "Cohesion", &m_def->liquid.cohesion, 0.0f, 0.5f, "%.3f" );
		changed |= ImGui::SliderFloat( "Adhesion", &m_def->liquid.adhesion, 0.0f, 0.5f, "%.3f" );
		changed |= ImGui::SliderFloat( "Viscosity", &m_def->liquid.linearViscosity, 0.0f, 20.0f, "%.2f" );
		changed |= ImGui::SliderFloat( "Quad viscosity", &m_def->liquid.quadraticViscosity, 0.0f, 5.0f, "%.2f" );
		changed |= ImGui::SliderFloat( "Adaptive ratio", &m_def->liquid.adaptiveRatio, 1.0f, 30.0f, "%.1f" );
		changed |= ImGui::SliderFloat( "Adaptive depth", &m_def->adaptiveDepth, 0.0f, 60.0f, "%.1f" );
		changed |= ImGui::SliderFloat( "Fast speed (0 off)", &m_def->liquid.adaptiveSpeed, 0.0f, 12.0f, "%.1f" );
		changed |= ImGui::Checkbox( "Respect depth limit", &m_def->liquid.adaptiveDepthLimit );
		ImGui::Text( "Size changes %d", p.sizeChanges );
		changed |= ImGui::SliderFloat( "Full box", &m_def->adaptiveFullBox, 0.5f, 1.2f, "%.2f" );
		changed |= ImGui::SliderFloat( "Size tolerance", &m_def->liquid.adaptiveTolerance, 0.0f, 0.45f, "%.2f" );
		changed |= ImGui::SliderFloat( "Split/merge limit", &m_def->liquid.adaptiveLimit, 1.5f, 4.0f, "%.2f" );
		changed |= ImGui::SliderFloat( "Exchange rate", &m_def->liquid.adaptiveRate, 0.01f, 1.0f, "%.2f", ImGuiSliderFlags_Logarithmic );
		ImGui::Text( "Measure %.2f ms", p.measure );
		changed |= ImGui::SliderFloat( "Contact pressure", &m_def->liquid.contactPressure, 0.0f, 2.0f, "%.2f" );
		changed |= ImGui::SliderFloat( "Friction", &m_def->liquid.friction, 0.0f, 1.0f, "%.2f" );
		changed |= ImGui::SliderFloat( "Added mass", &m_def->liquid.addedMass, 0.0f, 1.0f, "%.2f" );
		changed |= ImGui::SliderFloat( "Spring stiffness", &m_def->liquid.springStiffness, 0.0f, 100000.0f, "%.0f", ImGuiSliderFlags_Logarithmic );
		changed |= ImGui::SliderFloat( "Plasticity", &m_def->liquid.plasticity, 0.0f, 5.0f, "%.2f" );
		changed |= ImGui::SliderFloat( "Yield ratio", &m_def->liquid.yieldRatio, 0.0f, 0.5f, "%.2f" );
		if ( changed )
		{
			b2Fluid_SetDef( m_fluid, m_def );
		}

		ImGui::SliderFloat( "Dot size", &m_pointSize, 1.0f, 12.0f, "%.1f" );

		// These need a new scene
		bool rebuild = ImGui::SliderFloat( "Spacing", &m_def->particleSpacing, 0.02f, 0.2f, "%.3f" );
		rebuild |= ImGui::SliderFloat( "Liquid density", &m_def->liquid.massDensity, 0.25f, 4.0f, "%.2f" );
		if ( m_scene->type == lab_tank )
		{
			rebuild |= ImGui::SliderInt( "Crates", &m_scene->crateCount, 0, 40 );
		}
		if ( m_scene->controlLabel != nullptr )
		{
			// A range over two decades, such as the ball's density, reads better on a log scale
			bool wide = m_scene->controlMin > 0.0f && m_scene->controlMax >= 100.0f * m_scene->controlMin;
			bool moved = ImGui::SliderFloat( m_scene->controlLabel, &m_scene->control, m_scene->controlMin, m_scene->controlMax, wide ? "%.3f" : "%.2f",
											 wide ? ImGuiSliderFlags_Logarithmic : 0 );

			// The hole and the ball are built into the scene
			rebuild |= moved && ( m_scene->type == lab_hole || m_scene->type == lab_tap || m_scene->type == lab_space || m_scene->type == lab_splash || m_scene->type == lab_drain || m_scene->type == lab_pool );
		}
		if ( rebuild )
		{
			m_context->restart = true;
		}
		return false;
	}

	template <LabSceneType type>
	static Sample* Create( SampleContext* context )
	{
		return new SampleFluid( context, type );
	}

	static inline b2FluidDef s_defs[lab_sceneCount];
	static inline LabScene s_scenes[lab_sceneCount];

	LabScene* m_scene;
	b2FluidDef* m_def;
	b2FluidId m_fluid;
	float m_pointSize = 4.0f;
};

// Scenes that show the liquid off are under Fluid. Fluid Tests holds the ones built to stress or measure it.
static int sampleTank = RegisterSample( "Fluid", "Tank", SampleFluid::Create<lab_tank> );
static int sampleHole = RegisterSample( "Fluid", "Hole And Slopes", SampleFluid::Create<lab_hole> );
static int sampleBoat = RegisterSample( "Fluid", "Paddle Boat", SampleFluid::Create<lab_boat> );
static int sampleDrum = RegisterSample( "Fluid", "Rotating Tank", SampleFluid::Create<lab_drum> );
static int sampleWheel = RegisterSample( "Fluid", "Water Wheel", SampleFluid::Create<lab_wheel> );
static int sampleWaves = RegisterSample( "Fluid", "Wave Maker", SampleFluid::Create<lab_waves> );
static int sampleSplash = RegisterSample( "Fluid", "Splash", SampleFluid::Create<lab_splash> );
static int sampleDrain = RegisterSample( "Fluid Tests", "Deep Drain", SampleFluid::Create<lab_drain> );
static int samplePool = RegisterSample( "Fluid Tests", "Deep Pool", SampleFluid::Create<lab_pool> );
static int sampleDrop = RegisterSample( "Fluid", "Drop", SampleFluid::Create<lab_drop> );
static int sampleLayers = RegisterSample( "Fluid", "Three Liquids", SampleFluid::Create<lab_layers> );
static int sampleChain = RegisterSample( "Fluid Tests", "Chain Bowl", SampleFluid::Create<lab_chain> );
static int sampleTap = RegisterSample( "Fluid", "Dripping Tap", SampleFluid::Create<lab_tap> );
static int sampleHoney = RegisterSample( "Fluid", "Honey Race", SampleFluid::Create<lab_honey> );
static int sampleLava = RegisterSample( "Fluid", "Lava Lamp", SampleFluid::Create<lab_lava> );
static int sampleSpace = RegisterSample( "Fluid", "Zero-G Drops", SampleFluid::Create<lab_space> );
static int samplePlunge = RegisterSample( "Fluid", "Drop Into Pool", SampleFluid::Create<lab_plunge> );
static int sampleMercury = RegisterSample( "Fluid", "Quicksilver", SampleFluid::Create<lab_mercury> );
static int sampleSlime = RegisterSample( "Fluid", "Slime Stairs", SampleFluid::Create<lab_slime> );
static int sampleJelly = RegisterSample( "Fluid", "Jelly And Dough", SampleFluid::Create<lab_jelly> );
