#include "particles.hpp"

using namespace cgp;

particle_structure::particle_structure(float creation_time)
{
	float const theta = rand_uniform(0, 2 * Pi);
	p0 = { 0,0,0 };
	v0 = { 0.8f * std::sin(theta), 0.8f * std::cos(theta), rand_uniform(3, 5)};
	t0 = creation_time;
	bounced = false;
}

vec3 particle_structure::evaluate_position(float absolute_time)
{
	vec3 const g = { 0,0,-9.81f };      // gravity constant
	float const t = absolute_time - t0; // local time elapsed since the particle creation

	float const ti = (-2.0 * v0.z) / g.z;
	vec3 p;

	if (t < ti)
	{
		p = 0.5f * g * t * t + v0 * t + p0; // currently only models the first parabola
	}
	else
	{
		if (!bounced)
		{
			bounce_p0 = 0.5f * g * ti * ti + v0 * ti + p0;
			bounced = true;
		}
		
		bounce_t0 = t - ti;
		float bounce_factor = 0.6f;
		
		p = 0.5f * g * bounce_t0 * bounce_t0 + bounce_factor * v0 * bounce_t0 + bounce_p0;
	}
	return p;
}

void particle_system_structure::create_new_particle(float t0)
{
	particles.push_back(particle_structure(t0));
	trajectory.push_back(trajectory_drawable(20));
}

void particle_system_structure::remove_old_particles(float t)
{
	assert_cgp_no_msg(trajectory.size() == particles.size());
	float const max_time = 3.0f;

	// Loop over all active particles
	auto it_trajectory = trajectory.begin();
	for (auto it = particles.begin(); it != particles.end();)
	{
		// if a particle is too old, remove it
		if (t - it->t0 > max_time) {

			// remove particle
			it = particles.erase(it);

			// clear and remove associated trajectory
			it_trajectory->clear();
			trajectory.erase(it_trajectory);
		}

		// Go to the next particle if we are not already on the last one
		if (it != particles.end()) {
			++it;
			++it_trajectory;
		}
	}
}