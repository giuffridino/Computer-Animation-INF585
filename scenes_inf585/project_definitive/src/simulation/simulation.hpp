#pragma once

#include "cgp/cgp.hpp"
#include "../cloth/cloth.hpp"
#include "../constraint/constraint.hpp"

struct particle_structure
{
    cgp::vec3 p; // Position
    cgp::vec3 v; // Speed

    cgp::vec3 c; // Color
    float r;     // Radius
    float m;     // mass
};

struct slide_structure
{
	cgp::mesh_drawable slide_mesh;
	std::vector<cgp::vec3> slide_vertices;
	cgp::numarray<cgp::numarray<cgp::int3>> slide_faces;
};

struct simulation_parameters
{
    float dt = 0.005f;        // time step for the numerical integration
    float mass_total = 5.0f; // total mass of the cloth
    float K = 150.0f;         // stiffness parameter
    float mu = 15.0f;        // damping parameter

    //  Wind magnitude and direction
    struct {
        float magnitude = 17.0f;
        cgp::vec3 direction = { 0,-1,0 };
    } wind;
};

// Fill the forces in the cloth given the position and velocity
void simulation_compute_force(cloth_structure& cloth, simulation_parameters const& parameters, std::vector<particle_structure>& particles);

// Perform 1 step of a semi-implicit integration with time step dt
void simulation_numerical_integration(cloth_structure& cloth, simulation_parameters const& parameters, float dt);

// Apply the constraints (fixed position, obstacles) on the cloth position and velocity
void simulation_apply_constraints(cloth_structure& cloth, constraint_structure const& constraint, std::vector<particle_structure>& particles);

// Helper function that tries to detect if the simulation diverged 
bool simulation_detect_divergence(cloth_structure const& cloth);

void simulate(std::vector<particle_structure>& particles, float dt_argument, slide_structure slide);

