#include "particles.hpp"

using namespace cgp;


particle_bubble::particle_bubble(float creation_time)
{
	t0 = creation_time;
	float const theta = rand_uniform(0.0f, 2 * Pi);
	float const radius_position = rand_uniform(0.0f, 0.7f);
	p0 = vec3(-0.5f + rand_uniform(0.0f, 1.0f), 0.0f, -0.5f + rand_uniform(0.0f, 1.0f));
	// p0 = radius_position * vec3(std::cos(theta), 0.25f, std::sin(theta));
	radius = std::sqrt(rand_uniform(0, 0.08f*0.08f));
	color = { 0.5f + rand_uniform(0,0.2f),0.6f + rand_uniform(0,0.2f),1.0f - rand_uniform(0,0.2f) };
}

vec3 particle_bubble::evaluate_position(float absolute_time) const
{
	float t = absolute_time - t0;

	return {p0.x + radius*std::sin(3*t), t, p0.z + radius*std::cos(3*t)};

}

particle_billboard::particle_billboard(float creation_time)
{
	float const theta = rand_uniform(0, 2 * Pi);
	t0 = creation_time;
	p0 = { 0.5f *std::sin(theta), 0.3, 0.5f *std::cos(theta) };
	v0 = { 0.5f * std::sin(theta), 0.7, 0.5f * std::cos(theta)};
}

vec3 particle_billboard::evaluate_position(float absolute_time) const
{
	vec3 const g = { 0, -1.0f, 0 };      // gravity constant
	float const t = absolute_time - t0; // local time elapsed since the particle creation
	
	vec3 p = 0.5f * g * t * t + v0 * t + p0;
	return p;
}


template <typename T>
static void remove_old_element(std::vector<T>& container, float current_time, float max_time)
{
	// Loop over all active particles
	for (auto it = container.begin(); it != container.end();)
	{
		// if a particle is too old, remove it
		if (current_time - it->t0 > max_time)
			it = container.erase(it);

		// Go to the next particle if we are not already on the last one
		if (it != container.end())
			++it;
	}
}


void particle_system_structure::remove_old_particles(float t)
{
	remove_old_element(bubbles, t, 3.0f);
	remove_old_element(billboards, t, 3.0f);
}