#include "simulation.hpp"

using namespace cgp;

void simulate(std::vector<particle_structure>& particles, float dt_argument, std::vector<wall_bounds> const& walls)
{
	vec3 const g = { 0,0,-9.81f };
	size_t const N = particles.size();
	size_t const N_substep = 10;
	float const dt = dt_argument / N_substep;
	for (size_t k_substep = 0; k_substep < N_substep; ++k_substep)
	{
		// Update velocity along gravity
		for (size_t k = 0; k < N; ++k)
		{
			particle_structure& particle = particles[k];
			vec3 const f = particle.m * g;

			particle.v = (1 - 0.9f * dt) * particle.v + dt * f / particle.m;
			// particle.p = particle.p + dt * particle.v;
		}

		// **************************************** //
		// To do :
		//  Handle collision ...
		// **************************************** //

		// Handle collisions with other spheres
		const float v_alpha = 1.0f;
		for (size_t i = 0; i < N; ++i)
		{
			particle_structure& particle = particles[i];
			for (size_t j = i + 1; j < N; ++j)
			{
				particle_structure& other = particles[j];
				vec3 const d = particle.p - other.p;
				float const l = norm(d);
				if (l <= particle.r + other.r)
				{
					vec3 const u = d / l;
					float const j = 2 * (particle.m * other.m) / (particle.m + other.m) * dot(other.v - particle.v, u);
					// particle.v = v_alpha * (particle.v + j / particle.m * u);
					// other.v = v_alpha * (other.v - j / other.m * u);
					// particle.p = particle.p + 0.5f * (particle.r + other.r - l) * u;
					// other.p = other.p - 0.5f * (particle.r + other.r - l) * u;
					if (dot(particle.v - other.v, u) < 0)
					{
						particle.v = v_alpha * (particle.v + j / particle.m * u);
						other.v = v_alpha * (other.v - j / other.m * u);
					}
				}
			}
		}

		const float alpha = 1.0f;
		const float beta = 1.0f;
		for (size_t i = 0; i < N; ++i)
		{
			particle_structure& particle = particles[i];
			for (size_t w = 0; w < walls.size(); w++)
			{
				// if (dot((particle.p - walls[w].a), walls[w].n) <= particle.r)
				// {
				// 	vec3 v_ort = dot(particle.v, walls[w].n) * walls[w].n;
				// 	vec3 v_paral = particle.v - dot(particle.v, walls[w].n) * walls[w].n;
				// 	particle.v = alpha * v_paral - beta * v_ort;

				// 	//position based
				// 	float d = particle.r - dot(particle.p - walls[w].a, walls[w].n);
				// 	particle.p = particle.p + d * walls[w].n;

				// }		
				if (dot((particle.p - walls[w].a), walls[w].n) <= particle.r && dot(particle.v, walls[w].n) < 0)
				{
					vec3 const v_ort = dot(particle.v, walls[w].n) * walls[w].n;
					vec3 const v_par = particle.v - v_ort;
					particle.v = alpha * v_par - beta * v_ort;
				}		
			}
		}

		// Update position from velocity
		for (size_t k = 0; k < N; ++k)
		{
			particle_structure& particle = particles[k];
			particle.p = particle.p + dt * particle.v;
		}
	}
}
