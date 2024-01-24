#include "ffd.hpp"

using namespace cgp;

// Helper to compute permutation(n, k) - k among n
//   avoids the recursive formula (too costly)
int binomial_coeff(int n, int k)
{
    int res = 1;
    if(k>n-k)
        k = n - k;
    for(int i=0; i<k; ++i) {
        res *= (n-i);
        res /= (i+1);
    }
    return res;
}

float b(float u, int i, int n)
{
	return binomial_coeff(n, i) * pow(u, i) * pow(1 - u, n - i);
}

// Precompute the FFD elements that depends only on the initial position of the shape (its barycentric coordinates in the grid)
numarray<grid_3D<float> > precompute_weights(numarray<vec3>& position, int Nx, int Ny, int Nz)
{
	numarray<grid_3D<float>> weights;
	weights.resize(position.size());
	for (unsigned int i = 0; i < weights.size(); i++)
	{
		std::cout << position[i] << "\n";
		weights[i].resize(Nx, Ny, Nz);
		for (int kx = 0; kx < Nx; kx++)
		{
			for (int ky = 0; ky < Ny; ky++)
			{
				for (int kz = 0; kz < Nz; kz++)
				{
					weights[i](kx, ky, kz) = b(position[i].x, kx, Nx - 1) * b(position[i].y, ky, Ny - 1) * b(position[i].z, kz, Nz - 1);
				}
			}
		}
	}
	return weights;
}


// Computation of the FFD deformation on the position with respect to the grid
void ffd_deform(numarray<vec3>& position, grid_3D<vec3> const& grid, numarray<grid_3D<float> > const& weights)
{
	int const Nx = grid.dimension.x;
	int const Ny = grid.dimension.y;
	int const Nz = grid.dimension.z;

	int const N_vertex = position.size();

	// TO DO: compute the deformed position
	// General formulation:
	// For all position k of the shape to be deformed
	//     position[k] = sum_{x,y,z} weights[k] * grid(x,y,z)
	for (unsigned int i = 0; i < position.size(); i++)
	{
		vec3 deformed_position = {0.0f, 0.0f, 0.0f};
		for (int kx = 0; kx < Nx; kx++)
		{
			for (int ky = 0; ky < Ny; ky++)
			{
				for (int kz = 0; kz < Nz; kz++)
				{
					deformed_position += weights[i](kx, ky, kz) * grid(kx, ky, kz);
					// position[i] += 100 * weights[i](kx, ky, kz) * grid(position[i].x, position[i].y, position[i].z);
				}
			}
		}
		position[i] = deformed_position;
	}
}
