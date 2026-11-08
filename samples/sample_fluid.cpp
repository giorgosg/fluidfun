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
			s_def = b2fDefaultFluidDef();
			s_def.capacity = 2000000;
			for ( int i = 0; i < lab_sceneCount; ++i )
			{
				s_scenes[i] = LabDefaultScene( (LabSceneType)i );
			}
			first = false;
		}

		m_scene = &s_scenes[type];
		if ( m_context->restart == false )
		{
			m_context->camera.center = { m_scene->viewCenter.x, m_scene->viewCenter.y };
			m_context->camera.zoom = m_scene->viewHalfWidth;
		}

		m_fluid = LabCreateScene( m_worldId, m_scene, &s_def );
	}

	~SampleFluid() override
	{
		b2fDestroyFluid( m_fluid );
	}

	void Step() override
	{
		if ( m_context->pause == false || m_context->singleStep > 0 )
		{
			LabStepScene( m_scene, m_fluid, 1.0f / m_context->hertz );
			b2fFluid_Prepare( m_fluid );
		}

		Sample::Step();

		int n = b2fFluid_GetParticleCount( m_fluid );
		const b2Pos* pos = b2fFluid_GetPositions( m_fluid );
		const float* vx = b2fFluid_GetVelocitiesX( m_fluid );
		const float* vy = b2fFluid_GetVelocitiesY( m_fluid );
		const float* size = b2fFluid_GetSizes( m_fluid );
		for ( int i = 0; i < n; ++i )
		{
			float scale = size[i] / s_def.particleSpacing;
			float speed = b2MinFloat( b2Length( { vx[i], vy[i] } ) / 8.0f, 1.0f );
			uint32_t red = (uint32_t)( 40.0f + 215.0f * speed );
			uint32_t green = (uint32_t)( 110.0f + 145.0f * speed );
			if ( scale > 1.05f )
			{
				green = b2MinInt( 255, (int)( (float)green + 60.0f * scale ) );
			}
			DrawPoint( m_context->draw, pos[i], m_pointSize * scale, b2HexColor( ( red << 16 ) | ( green << 8 ) | 0xFF ) );
		}
	}

	bool DrawControls() override
	{
		ImGui::Text( "Particles %d", b2fFluid_GetParticleCount( m_fluid ) );
		b2fProfile p = b2fFluid_GetProfile( m_fluid );
		ImGui::Text( "Fluid %.2f ms", p.total );
		ImGui::Text( "sort %.2f", p.sort );
		ImGui::Text( "density %.2f collide %.2f", p.density, p.collide );

		bool changed = false;
		changed |= ImGui::SliderFloat( "Stiffness", &s_def.stiffness, 0.0f, 0.2f, "%.3f" );
		changed |= ImGui::SliderFloat( "Near", &s_def.nearStiffness, 0.0f, 0.5f, "%.3f" );
		changed |= ImGui::SliderFloat( "Viscosity", &s_def.linearViscosity, 0.0f, 20.0f, "%.2f" );
		changed |= ImGui::SliderFloat( "Quad viscosity", &s_def.quadraticViscosity, 0.0f, 5.0f, "%.2f" );
		changed |= ImGui::SliderFloat( "Adaptive ratio", &s_def.adaptiveRatio, 1.0f, 30.0f, "%.1f" );
		changed |= ImGui::SliderFloat( "Adaptive depth", &s_def.adaptiveDepth, 0.0f, 60.0f, "%.1f" );
		changed |= ImGui::SliderFloat( "Fast speed (0 off)", &s_def.adaptiveSpeed, 0.0f, 12.0f, "%.1f" );
		changed |= ImGui::Checkbox( "Respect depth limit", &s_def.adaptiveDepthLimit );
		ImGui::Text( "Size changes %d", p.sizeChanges );
		changed |= ImGui::SliderFloat( "Full box", &s_def.adaptiveFullBox, 0.5f, 1.2f, "%.2f" );
		changed |= ImGui::SliderFloat( "Size tolerance", &s_def.adaptiveTolerance, 0.0f, 0.45f, "%.2f" );
		changed |= ImGui::SliderFloat( "Split/merge limit", &s_def.adaptiveLimit, 1.5f, 4.0f, "%.2f" );
		changed |= ImGui::SliderFloat( "Exchange rate", &s_def.adaptiveRate, 0.01f, 1.0f, "%.2f", ImGuiSliderFlags_Logarithmic );
		ImGui::Text( "Measure %.2f ms", p.measure );
		changed |= ImGui::SliderFloat( "Contact pressure", &s_def.contactPressure, 0.0f, 2.0f, "%.2f" );
		changed |= ImGui::SliderFloat( "Friction", &s_def.friction, 0.0f, 1.0f, "%.2f" );
		if ( changed )
		{
			b2fFluid_SetDef( m_fluid, &s_def );
		}

		ImGui::SliderFloat( "Dot size", &m_pointSize, 1.0f, 12.0f, "%.1f" );

		// These need a new scene
		bool rebuild = ImGui::SliderFloat( "Spacing", &s_def.particleSpacing, 0.02f, 0.2f, "%.3f" );
		rebuild |= ImGui::SliderFloat( "Liquid density", &s_def.massDensity, 0.25f, 4.0f, "%.2f" );
		if ( m_scene->type == lab_tank )
		{
			rebuild |= ImGui::SliderInt( "Crates", &m_scene->crateCount, 0, 40 );
		}
		if ( m_scene->controlLabel != nullptr )
		{
			bool moved = ImGui::SliderFloat( m_scene->controlLabel, &m_scene->control, m_scene->controlMin, m_scene->controlMax, "%.2f" );

			// The hole and the ball are built into the scene
			rebuild |= moved && ( m_scene->type == lab_hole || m_scene->type == lab_splash || m_scene->type == lab_drain || m_scene->type == lab_pool );
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

	static inline b2fFluidDef s_def;
	static inline LabScene s_scenes[lab_sceneCount];

	LabScene* m_scene;
	b2fFluid* m_fluid;
	float m_pointSize = 4.0f;
};

static int sampleTank = RegisterSample( "Fluid", "Tank", SampleFluid::Create<lab_tank> );
static int sampleHole = RegisterSample( "Fluid", "Hole And Slopes", SampleFluid::Create<lab_hole> );
static int sampleBoat = RegisterSample( "Fluid", "Paddle Boat", SampleFluid::Create<lab_boat> );
static int sampleDrum = RegisterSample( "Fluid", "Rotating Tank", SampleFluid::Create<lab_drum> );
static int sampleWheel = RegisterSample( "Fluid", "Water Wheel", SampleFluid::Create<lab_wheel> );
static int sampleWaves = RegisterSample( "Fluid", "Wave Maker", SampleFluid::Create<lab_waves> );
static int sampleSplash = RegisterSample( "Fluid", "Splash", SampleFluid::Create<lab_splash> );
static int sampleDrain = RegisterSample( "Fluid", "Deep Drain", SampleFluid::Create<lab_drain> );
static int samplePool = RegisterSample( "Fluid", "Deep Pool", SampleFluid::Create<lab_pool> );
