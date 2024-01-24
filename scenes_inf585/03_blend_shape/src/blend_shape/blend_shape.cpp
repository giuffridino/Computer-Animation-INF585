#include "blend_shape.hpp"

#include "../environment.hpp"

using namespace cgp;

void blend_shape_structure::update()
{
	// To Do:
	//  Compute here the new face considering the current blend shape weights
	//  Note:
	//    - This function is called every time one of the slider is modified. You should avoid creating new mesh_drawable: Try to only update existing ones.
	//  Help: 
	//    - weights[k_face] - referes to the blend shape weights for the pose k_face
	//    - face.vbo_position.update( [buffer of new positions] ) - allows to update the displayed position 
	//    - face.vbo_normal.update( normal_per_vertex([vertices], [connectivity]) ) - can be used to automatically compute new normals
	//    - If you use numarray to store the vertices, you can use the operator + (or +=) between two numarray of same size to add their values. You can also multiply a numarray with a scalar value.

	numarray<vec3> vertices = faces_storage[0].position;
	for (unsigned int i = 0; i < faces_storage[0].position.size(); i++)
	{
		vec3 initial_pos = faces_storage[0].position[i];
		vec3 current_pos = vec3(0.0f, 0.0f, 0.0f);
		for (unsigned int j = 0; j < faces_storage.size(); j++)
		{
			current_pos += weights[j] * (faces_storage[j].position[i] - initial_pos);
		}
		vertices[i] = initial_pos + current_pos;
	}
	face.vbo_position.update(vertices);
	face.vbo_normal.update(normal_per_vertex(vertices, faces_storage[0].connectivity));

	// std::cout << "Called update_blend_shape with weights values: " << weights << std::endl; // line to remove once you have seen its action
}

void blend_shape_structure::initialize()
{
	// Load faces
	std::cout << " Load faces ... " << std::endl;
	std::cout << "\t Load faces 00 " << std::endl;
	faces_storage.push_back(mesh_load_file_obj(project::path + "assets/face_00.obj"));
	std::cout << "\t Load faces 01 " << std::endl;
	faces_storage.push_back(mesh_load_file_obj(project::path + "assets/face_01.obj"));
	std::cout << "\t Load faces 02 " << std::endl;
	faces_storage.push_back(mesh_load_file_obj(project::path + "assets/face_02.obj"));
	std::cout << "\t Load faces 03 " << std::endl;
	faces_storage.push_back(mesh_load_file_obj(project::path + "assets/face_03.obj"));
	std::cout << "\t Load faces 04 " << std::endl;
	faces_storage.push_back(mesh_load_file_obj(project::path + "assets/face_04.obj"));
	std::cout << "\t Load faces 05 " << std::endl;
	faces_storage.push_back(mesh_load_file_obj(project::path + "assets/face_05.obj"));
	face.initialize_data_on_gpu(faces_storage[0]);


	size_t N_face = faces_storage.size();
	weights.resize(N_face);

	// To Do:
	//  You can add here any precomputation you need to do. For instance, precomputing the difference between a pose and the reference one.
	//  Help: 
	//    faces_storage[0].position[k] - refers to the k-th vertex position of the reference pose
	//    faces_storage[k_face].position[k] - refers to the k-th vertex of the pose k_face (for k_face>0)
}