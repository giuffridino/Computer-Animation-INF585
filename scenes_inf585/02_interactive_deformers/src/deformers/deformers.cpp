#include "deformers.hpp"



using namespace cgp;

void apply_deformation(mesh& shape, numarray<vec3> const& position_before_deformation, vec2 const& translate_screen, vec3 const& picked_position, vec3 const& picked_normal, rotation_transform const& camera_orientation, deformer_parameters_structure const& deformer_parameters)
{

	/** 
		shape:  The position of shape are the one to be deformed
		position_before_deformation:  Reference position of the mesh before applying this deformation
		translate_screen:   Input translation of the user in the 2D-screen coordinates - tr must be converted into a 3D transformation applied to the positions of shape
		picked_position:    3D Position of the picked vertex
		picked_normal:      Normal of the surface at the picked vertex position
		camera_orientation: Current camera orientation - allows to convert the 2D-screen coordinates into 3D coordinates
	*/


	float const r = deformer_parameters.falloff; // radius of influence of the deformation
	size_t const N = shape.position.size();
	for (size_t k = 0; k < N; ++k)
	{
		vec3& p_shape = shape.position[k];                             // position to deform
		vec3 const& p_shape_original = position_before_deformation[k]; // reference position before deformation
		vec3 const& p_clicked = picked_position;                       // 3D position of picked position
		vec3 const& n_clicked = picked_normal;                         // normal of the surface (before deformation) at the picked position

		float const dist = norm(p_clicked - p_shape_original);         // distance between the picked position and the vertex before deformation

		// TO DO: Implement the deformation models
		// **************************************************************** //
		// ...
		if (deformer_parameters.type == deform_translate) // Case of translation
		{
			if (deformer_parameters.direction == direction_view_space)
			{
				vec3 const translation = camera_orientation * vec3(translate_screen, 0.0f);
				float const w = exp(-(dist/r) * (dist/r));
				p_shape = p_shape_original + w * translation;
			}
			else
			{
				vec3 const t3D = camera_orientation * vec3(translate_screen, 0.0f);
				float gesture = dot(t3D, n_clicked);
				float const w = exp(-(dist/r) * (dist/r)) / ( 3 * Pi);
				p_shape = p_shape + w * gesture * n_clicked;
			}
			
			// Hint: You can convert the 2D translation in screen space into a 3D translation in the view plane in multiplying 
			//       camera_orientation * vec3(translate_screen, 0)

			// Fake deformation (linear translation in the screen space) 
			//   the following lines should be modified to get the expected smooth deformation
			// if (dist < r)
			// 	p_shape = p_shape_original + (1 - dist / r) * translation;

		}
		if (deformer_parameters.type == deform_twist)
		{
			vec3 axis;
			if (deformer_parameters.direction == direction_view_space)
			{
				axis = normalize(camera_orientation * vec3(0.0, 0.0, 1.0));
			}
			else
			{
				axis = n_clicked;
			}
			
			float magnitude = translate_screen.x; 
			float const w = exp(-(dist * dist)/ (r * r));
			rotation_transform R =  rotation_axis_angle(axis, magnitude * w / 1.0);
			p_shape = R * (p_shape_original - p_clicked) + p_clicked;
		}
		if (deformer_parameters.type == deform_scale)
		{
			float const w = exp(-(dist * dist)/ (r * r));
			mat3 S = mat3(vec3(w * translate_screen.x + 1.0, 0.0, 0.0), vec3(0.0, w * translate_screen.x + 1.0, 0.0), vec3(0.0, 0.0, w * translate_screen.x + 1.0));
			p_shape = S * (p_shape_original - p_clicked) + p_clicked;
		}
		if (deformer_parameters.type == deform_perlin)
		{
			float const w = exp(- (dist * dist) / r);
			// float const w = exp(-(dist * dist)/ (r * r));
			p_shape = p_shape_original + vec3(0.0, 0.0, 1.0) * w * 10 * noise_perlin(p_shape_original, 8) * translate_screen.y;
			// std::cout << "noise_perlin:" << noise_perlin(p_shape_original, 8) << "\n";
			// p_shape = p_shape_original + p_shape_original * vec3(0.0, 0.0, 1.0) * noise_perlin(p_shape_original.x, p_shape_original.y) * translate_screen.y;
		}
	}
}

// float snoise_abs(vec2 v, int octaves)
// {
// 	float res = 0.0;
// 	float scale = 1.0;
// 	for(int i=0; i<8; i++) {
// 		if(i >= octaves) break;
// 		res += abs(smooth_snoise(v)) * scale;
// 		v *= vec2(2.0, 2.0);
// 		scale *= 0.5;
// 	}
// 	return res;
// }

// float smooth_snoise(vec2 v){
//   const vec4 C = vec4(0.211324865405187,
//                       0.366025403784439, 
//                      -0.577350269189626,
//                       0.024390243902439);
// // First corner
//   vec2 i = floor(v + dot(v, C.yy) );
//   vec2 x0 = v - i + dot(i, C.xx);

// // Other corners
//   vec2 i1;
//   i1 = (x0.x > x0.y) ? vec2(1.0, 0.0) : vec2(0.0, 1.0);
//   vec4 x12 = x0.xyxy + C.xxzz;
//   x12.xy -= i1;

// // Permutations
//   i = mod289(i);
//   vec3 p = permute(permute( i.y + vec3(0.0, i1.y, 1.0 ))
// + i.x + vec3(0.0, i1.x, 1.0 ));

//   vec3 m = max(0.5 - vec3(dot(x0,x0), dot(x12.xy,x12.xy), dot(x12.zw,x12.zw)), 0.0);
//   m = m*m ;
//   m = m*m ;

//   vec3 x = 2.0 * fract(p * C.www) - 1.0;
//   vec3 h = abs(x) - 0.5;
//   vec3 ox = floor(x + 0.5);
//   vec3 a0 = x - ox;

//   m *= 1.79284291400159 - 0.85373472095314 * ( a0*a0 + h*h );

//   vec3 g;
//   g.x = a0.x * x0.x + h.x * x0.y;
//   g.yz = a0.yz * x12.xz + h.yz * x12.yw;
//   return 130.0 * dot(m, g);
// }