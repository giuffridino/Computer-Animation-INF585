#pragma once

#include "cgp/cgp.hpp"
#include "../skeleton/skeleton.hpp"

struct rigged_model_structure {
    cgp::mesh mesh_bind_pose;  // Bind pose (/un-deformed) mesh
    cgp::mesh mesh_deformed;   // Deformed mesh
    cgp::numarray<cgp::numarray<float> > skinning_weight; //skinning_weight[k_vertex][k_joint]
};


struct animated_model_structure {
    rigged_model_structure rigged_mesh;
    skeleton_structure skeleton;

    // Compute the Linear Blend Skinning deformation
    //  Once computed, the rigged_mesh contains the updated deformed meshes
    void skinning_lbs();

    // Compute Dual Quaternion Skinning deformation
    //  Once computed, the rigged_mesh should contain the updated deformed meshes
    void skinning_dqs();
};

namespace cgp
{
    struct dual_quaternion
    {
        quaternion real;
        quaternion dual;

        dual_quaternion(quaternion q, vec3 t)
        {
            real = q;
            dual = quaternion(t, 0.0f) * q * 0.5f;
        }
        dual_quaternion(quaternion r, quaternion d)
        {
            real = r;
            dual = d;
        }
        dual_quaternion operator +(dual_quaternion const& q)
        {
            return dual_quaternion(real + q.real, dual + q.dual);
        }
        dual_quaternion operator +=(dual_quaternion const& q)
        {
            real += q.real;
            dual += q.dual;
            return *this;
        }
        dual_quaternion operator *(dual_quaternion const& q)
        {
            return dual_quaternion(real * q.real, real * q.dual + dual * q.real);
        }
        dual_quaternion operator *(float const& f)
        {
            return dual_quaternion(real * f, dual * f);
        }
        static dual_quaternion normalize(dual_quaternion const& q)
        {
            return dual_quaternion(q.real / norm(q.real), q.dual / norm(q.real));
        }
        dual_quaternion normalize()
        {
            return dual_quaternion(real / norm(real), dual / norm(real));
        }
    };
}