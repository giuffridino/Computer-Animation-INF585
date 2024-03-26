#pragma once

#include "cgp/cgp.hpp"


struct particle_structure
{
    cgp::vec3 p; // Position
    cgp::vec3 v; // Speed

    cgp::vec3 c; // Color
    float r;     // Radius
    float m;     // mass
};

struct wall_bounds
{
    cgp::vec3 a;
    cgp::vec3 n;
};

void simulate(std::vector<particle_structure>& particles, float dt_argument, std::vector<wall_bounds> const& walls);

