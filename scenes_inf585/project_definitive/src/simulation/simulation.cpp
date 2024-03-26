#include "simulation.hpp"

using namespace cgp;

float minimum(float a, float b)
{
	return a < b ? a : b;
}

float maximum(float a, float b)
{
	return a > b ? a : b;
}

float distance_point_to_line_segment(vec3 p, vec3 p1, vec3 p2) {
	float t = dot(p - p1, p2 - p1) / norm(p2 - p1);

	if (t < 0.0f) {
		return norm(p - p1);
	}
	else if (t > 1.0f) {
		return norm(p - p2);
	}
	else {
		vec3 closest_point = p1 + t * (p2 - p1);
		return norm(p - closest_point);
	}
}


bool collision_sphere_triangle(vec3 p, float r, vec3 p1, vec3 p2, vec3 p3)
{
	const float offset = 0.01f;
	vec3 n = normalize(cross(p2 - p1, p3 - p1));
	vec3 proj_p = p - dot(p - p1, n) * n;
	if (dot(cross(p2 - p1, proj_p - p1), n) > 0 && dot(cross(p3 - p2, proj_p - p2), n) > 0 && dot(cross(p1 - p3, proj_p - p3), n) > 0)
	{
		return norm(proj_p - p) <= r + offset;
	}

	std::vector<float> edge_distances = {
		distance_point_to_line_segment(p, p1, p2),
		distance_point_to_line_segment(p, p2, p3),
		distance_point_to_line_segment(p, p3, p1)
	};

	return *std::min_element(edge_distances.begin(), edge_distances.end()) <= r + offset;

}

bool collision_sphere_sphere(particle_structure& particle_1, particle_structure& particle_2)
{
	vec3& p1 = particle_1.p;
	vec3& p2 = particle_2.p;
	vec3& v1 = particle_1.v;
	vec3& v2 = particle_2.v;
	float const r1 = particle_1.r;
	float const r2 = particle_2.r;

	float const alpha = 0.95f;

	vec3 const p12 = p1 - p2;
	float const d12 = norm(p12);

	if (d12 < r1 + r2)
	{
		vec3 const u12 = p12 / d12;
		
		float velocity_component = dot(v1-v2, p1-p2);
		if(velocity_component<0) {

			float const j = dot(v1 - v2, u12);
			v1 = v1 - alpha * j * u12;
			v2 = v2 + alpha * j * u12;
			return true;
		}
	}
	return false;
}

bool cancel_colliding_velocity(particle_structure& particle_1, particle_structure& particle_2)
{
	vec3& p1 = particle_1.p;
	vec3& p2 = particle_2.p;
	vec3& v1 = particle_1.v;
	vec3& v2 = particle_2.v;

	float const r1 = particle_1.r;
	float const r2 = particle_2.r;

	vec3 const p12 = p1 - p2;
	float const d12 = norm(p12);

	if (d12 < r1 + r2)
	{
		vec3 const u12 = p12 / d12;

		float velocity_component = dot(v1-v2, u12);
		if(velocity_component<0) {
			v1 = v1-dot(v1,u12)*u12;
			v2 = v2-dot(v2,u12)*u12;
			return true;
		}
	}
	return false;
}

bool cancel_sphere_triangle_velocity(particle_structure& particle, cgp::vec3 const& p1, cgp::vec3 const& p2, cgp::vec3 const& p3)
{
	if (collision_sphere_triangle(particle.p, particle.r, p1, p2, p3))
	{
		vec3 const n = normalize(cross(p2 - p1, p3 - p1));
		if (dot(particle.v, n) < 0)
		{
			particle.v = particle.v-dot(particle.v, n)*n;
			return true;
		}
	}
	return false;
}

vec3 compute_face_normal(cgp::vec3 const& p1, cgp::vec3 const& p2, cgp::vec3 const& p3)
{
	return normalize(cross(p2 - p1, p3 - p1));
}

void simulate(std::vector<particle_structure>& particles, float dt_argument, slide_structure slide)
{
	vec3 const g = { 0,0,-9.81f };
	size_t const N = particles.size();
	size_t const N_substep = 10;
	float const dt = dt_argument / N_substep;

	const float alpha = 1.0f;
	const float vt_coef = 1.0f;
	const float vn_coef = 0.1f;
	for (size_t k_substep = 0; k_substep < N_substep; ++k_substep)
	{
		// Update velocity along gravity
		for (size_t k = 0; k < N; ++k)
		{
			particle_structure& particle = particles[k];
			vec3 const f = particle.m * g;

			particle.v = (1 - 0.9f * dt) * particle.v + dt * f / particle.m;
		}

		// Handle collisions with other spheres
		for (size_t k1 = 0; k1 < N; ++k1)
		{
			for (size_t k2 = k1 + 1; k2 < N; ++k2)
			{
				if (collision_sphere_sphere(particles[k1], particles[k2]))
					break;
			}

			particle_structure& particle = particles[k1];
			for (size_t f = 0; f < slide.slide_faces.size(); f++)
			{
				const vec3 p1 = slide.slide_vertices[slide.slide_faces[f][0][0]];
				const vec3 p2 = slide.slide_vertices[slide.slide_faces[f][1][0]];
				const vec3 p3 = slide.slide_vertices[slide.slide_faces[f][2][0]];
				if (collision_sphere_triangle(particle.p, particle.r, p1, p2, p3))
				{
					vec3 n = normalize(cross(p2 - p1, p3 - p1));
					if (dot(particle.v, n) < 0)
					{
						vec3 const vn = dot(particle.v, n) * n;
						vec3 const vt = particle.v - vn;
						particle.v = alpha*(vt_coef * vt - vn_coef * vn);

						// Fixes slight compenetration with the slide but adds jitering (when dividing by 10 not so noticeable)
						particle.p = particle.p + dot(particle.p - p1, n) * n / 10;
						break;
					}
				}
			}

			for (size_t k2 = k1 + 1; k2 < N; ++k2)
			{
				if (cancel_colliding_velocity(particles[k1], particles[k2]))
					break;
			}

			for (size_t k_face = 0; k_face < slide.slide_faces.size(); ++k_face)
			{
				if (cancel_sphere_triangle_velocity(particles[k1], slide.slide_vertices[slide.slide_faces[k_face][0][0]], slide.slide_vertices[slide.slide_faces[k_face][1][0]], slide.slide_vertices[slide.slide_faces[k_face][2][0]]))
				 	break;
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

cgp::vec3 compute_spring_force_p1_p2(vec3 p1, vec3 p2, float K, float L0) {
	cgp::vec3 force = -K * (norm(p1 - p2) - L0) * (p1 - p2) / norm(p1 - p2);
	return force;
}

bool vertex_in_grid(int ku, int kv, int N) {
	if (ku < 0 || ku >= N || kv < 0 || kv >= N)
		return false;
	else
		return true;
}

cgp::vec3 compute_wind_force(int ku, int kv, float L0, cgp::vec3 n, cgp::vec3 w, float m) {
	float area = L0 * L0;
	cgp::vec3 force = m * area * dot(n, w) * n;
	return force;
}

void simulation_compute_force(cloth_structure& cloth, simulation_parameters const& parameters, std::vector<particle_structure>& particles)
{
	// Direct access to the variables
	//  Note: A grid_2D is a structure you can access using its 2d-local index coordinates as grid_2d(k1,k2)
	//   The index corresponding to grid_2d(k1,k2) is k1 + N1*k2, with N1 the first dimension of the grid.
	//   
	grid_2D<vec3>& force = cloth.force;  // Storage for the forces exerted on each vertex

	grid_2D<vec3> const& position = cloth.position;  // Storage for the positions of the vertices
	grid_2D<vec3> const& velocity = cloth.velocity;  // Storage for the normals of the vertices
	grid_2D<vec3> const& normal = cloth.normal;      // Storage for the velocity of the vertices


	size_t const N_total = cloth.position.size();       // total number of vertices
	size_t const N = cloth.N_samples();                 // number of vertices in one dimension of the grid

	// Retrieve simulation parameter
	//  The default value of the simulation parameters are defined in simulation.hpp
	float const K = parameters.K;              // spring stifness
	float const m = parameters.mass_total / N_total; // mass of a particle
	float const mu = parameters.mu;            // damping/friction coefficient
	float const	L0 = 1.0f / (N - 1.0f);        // rest length between two direct neighboring particle
	float const wind_magnitude = parameters.wind.magnitude;
	cgp::vec3 const wind_direction = parameters.wind.direction;

	// Gravity
	const vec3 g = { 0,0,-9.81f };
	for (int ku = 0; ku < N; ++ku)
		for (int kv = 0; kv < N; ++kv)
			force(ku, kv) = m * g;

	// Drag (= friction)
	for (int ku = 0; ku < N; ++ku)
		for (int kv = 0; kv < N; ++kv)
			force(ku, kv) += -mu * m * velocity(ku, kv);

	for (size_t k = 0; k < particles.size(); ++k)
	{
		particle_structure& particle = particles[k];
		for (int ku = 0; ku < N; ++ku)
			for (int kv = 0; kv < N; ++kv)
				if (norm(cloth.position(ku, kv) - particle.p) < particle.r + 0.5*1e-1) {
					force(ku, kv) += dot(cloth.normal(ku, kv), particle.v) * cloth.normal(ku, kv) * particle.m;
				}
	}

	for (int ku = 0; ku < N; ++ku) {
		for (int kv = 0; kv < N; ++kv) {

			if (vertex_in_grid(ku - 1, kv - 1, N))
				force(ku, kv) += compute_spring_force_p1_p2(position(ku, kv), position(ku - 1, kv - 1), K, L0 * sqrt(2));
			if (vertex_in_grid(ku - 1, kv, N))
				force(ku, kv) += compute_spring_force_p1_p2(position(ku, kv), position(ku - 1, kv), K, L0);
			if (vertex_in_grid(ku - 1, kv + 1, N))
				force(ku, kv) += compute_spring_force_p1_p2(position(ku, kv), position(ku - 1, kv + 1), K, L0 * sqrt(2));
			if (vertex_in_grid(ku, kv - 1, N))
				force(ku, kv) += compute_spring_force_p1_p2(position(ku, kv), position(ku, kv - 1), K, L0);
			if (vertex_in_grid(ku, kv + 1, N))
				force(ku, kv) += compute_spring_force_p1_p2(position(ku, kv), position(ku, kv + 1), K, L0);
			if (vertex_in_grid(ku + 1, kv - 1, N))
				force(ku, kv) += compute_spring_force_p1_p2(position(ku, kv), position(ku + 1, kv - 1), K, L0 * sqrt(2));
			if (vertex_in_grid(ku + 1, kv, N))
				force(ku, kv) += compute_spring_force_p1_p2(position(ku, kv), position(ku + 1, kv), K, L0);
			if (vertex_in_grid(ku + 1, kv + 1, N))
				force(ku, kv) += compute_spring_force_p1_p2(position(ku, kv), position(ku + 1, kv + 1), K, L0 * sqrt(2));
			if (vertex_in_grid(ku - 2, kv, N))
				force(ku, kv) += compute_spring_force_p1_p2(position(ku, kv), position(ku - 2, kv), K, L0 * 2);
			if (vertex_in_grid(ku + 2, kv, N))
				force(ku, kv) += compute_spring_force_p1_p2(position(ku, kv), position(ku + 2, kv), K, L0 * 2);
			if (vertex_in_grid(ku, kv - 2, N))
				force(ku, kv) += compute_spring_force_p1_p2(position(ku, kv), position(ku, kv - 2), K, L0 * 2);
			if (vertex_in_grid(ku, kv + 2, N))
				force(ku, kv) += compute_spring_force_p1_p2(position(ku, kv), position(ku, kv + 2), K, L0 * 2);
		}
	}

	for (int ku = 0; ku < N; ++ku) {
		for (int kv = 0; kv < N; ++kv)
		{
			force(ku, kv) += compute_wind_force(ku, kv, L0, normal(ku, kv), wind_direction, wind_magnitude);
		}
	}
}

void simulation_numerical_integration(cloth_structure& cloth, simulation_parameters const& parameters, float dt)
{
	int const N = cloth.N_samples();
	int const N_total = cloth.position.size();
	float const m = parameters.mass_total / static_cast<float>(N_total);

	for (int ku = 0; ku < N; ++ku) {
		for (int kv = 0; kv < N; ++kv) {
			vec3& v = cloth.velocity(ku, kv);
			vec3& p = cloth.position(ku, kv);
			vec3 const& f = cloth.force(ku, kv);

			// Standard semi-implicit numerical integration
			v = v + dt * f / m;
			p = p + dt * v;
		}
	}

}

void simulation_apply_constraints(cloth_structure& cloth, constraint_structure const& constraint, std::vector<particle_structure>& particles)
{
	// Fixed positions of the cloth
	for (auto const& it : constraint.fixed_sample) {
		position_contraint c = it.second;
		cloth.position(c.ku, c.kv) = c.position; // set the position to the fixed one
	}
}



bool simulation_detect_divergence(cloth_structure const& cloth)
{
	bool simulation_diverged = false;
	const size_t N = cloth.position.size();
	for (size_t k = 0; simulation_diverged == false && k < N; ++k)
	{
		const float f = norm(cloth.force.data.at_unsafe(k));
		const vec3& p = cloth.position.data.at_unsafe(k);

		if (std::isnan(f)) // detect NaN in force
		{
			std::cout << "\n **** NaN detected in forces" << std::endl;
			simulation_diverged = true;
		}

		if (f > 600.0f) // detect strong force magnitude
		{
			std::cout << "\n **** Warning : Strong force magnitude detected " << f << " at vertex " << k << " ****" << std::endl;
			simulation_diverged = true;
		}

		if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) // detect NaN in position
		{
			std::cout << "\n **** NaN detected in positions" << std::endl;
			simulation_diverged = true;
		}
	}

	return simulation_diverged;
}
