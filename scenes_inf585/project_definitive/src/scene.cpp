#include "scene.hpp"


using namespace cgp;


// Compute a new cloth in its initial position (can be called multiple times)
void scene_structure::initialize_cloth(int N_sample)
{
	cloth.initialize(N_sample);
	cloth_drawable.initialize(N_sample);
	cloth_drawable.drawable.texture = cloth_texture;
	cloth_drawable.drawable.material.texture_settings.two_sided = true;

	constraint.fixed_sample.clear();
	constraint.add_fixed_position(0, 0, cloth);
	constraint.add_fixed_position(0, N_sample - 1, cloth);
}


void scene_structure::initialize()
{
	camera_control.initialize(inputs, window); // Give access to the inputs and window global state to the camera controler
	camera_control.set_rotation_axis_z();
	camera_control.look_at({ 27.0f, -13.0f, 2.5f }, { 3.0f, -12.0f, -7.0f }, {0,0,1});

	timer.event_period = 1.0f;
	timer.scale = 2.0f;

	sphere.initialize_data_on_gpu(mesh_primitive_sphere());

	// Access the data of the slide
	slide.slide_mesh.initialize_data_on_gpu(mesh_load_file_obj("assets/slide-funnel.obj"));
	slide.slide_vertices = cgp::loader::obj_read_positions("assets/slide-funnel.obj");
	slide.slide_faces = cgp::loader::obj_read_faces("assets/slide-funnel.obj", cgp::loader::obj_type::vertex_texture_normal);

	cloth_texture.load_and_initialize_texture_2d_on_gpu(project::path + "assets/cloth.jpg");
	initialize_cloth(gui.N_sample_edge);
}

void scene_structure::display_frame()
{
	// Set the light to the current position of the camera
	environment.light = camera_control.camera_model.position();
	
	timer.update();

	// Call the simulation of the particle system
	float const dt = 0.01f * timer.scale;
	simulate(particles, dt, slide);

	// Display the result
	sphere_display();
	draw(slide.slide_mesh, environment);

	// Simulation of the cloth
	int const N_step = 1; // Adapt here the number of intermediate simulation steps (ex. 5 intermediate steps per frame)
	for (int k_step = 0; simulation_running == true && k_step < N_step; ++k_step)
	{
		simulation_compute_force(cloth, gui.cloth_parameters, particles);

		simulation_numerical_integration(cloth, gui.cloth_parameters, gui.cloth_parameters.dt);

		simulation_apply_constraints(cloth, constraint, particles);

		bool const simulation_diverged = simulation_detect_divergence(cloth);
		if (simulation_diverged) {
			std::cout << "\n *** Simulation has diverged ***" << std::endl;
			std::cout << " > The simulation is stoped" << std::endl;
			simulation_running = false;
		}
	}

	// Prepare to display the updated cloth
	cloth.update_normal();        // compute the new normals
	cloth_drawable.update(cloth); // update the positions on the GPU
	update_wind_magnitude();

	// Display the cloth
	draw(cloth_drawable, environment);

}

void scene_structure::sphere_display()
{
	// Display the particles as spheres
	size_t const N = particles.size();
	for (size_t k = 0; k < N; ++k)
	{
		particle_structure const& particle = particles[k];
		sphere.material.color = particle.c;
		sphere.model.translation = particle.p;
		sphere.model.scaling = particle.r;

		draw(sphere, environment);
	}
}

void scene_structure::start_race()
{
	particles.clear();
	emit_particle(gui.sphere_count);	
}

void scene_structure::update_wind_magnitude()
{
	wind_counter++;
	if (wind_counter % 200 == 0)
	{
		gui.cloth_parameters.wind.magnitude = (gui.cloth_parameters.wind.magnitude == 17.0f) ? 3.0f : 17.0f;
	}
}

void scene_structure::emit_particle(int count)
{
	// Emit count particles with random position
	static numarray<vec3> const color_lut = { {1,0,0},{0,1,0},{0,0,1},{1,1,0},{1,0,1},{0,1,1} };
	particle_structure particle;
	rand_initialize_generator();
	for (int i = 0; i < count; i++)
	{
		const float px = rand_uniform(-0.05f, 0.05f);
		const float py = rand_uniform(-0.05f, 0.05f);
		const float theta = rand_uniform(0, 2 * 3.14f);
		particle.p = { cos(theta) * gui.radius + gui.px, sin(theta) * gui.radius + gui.py, 3.0 };
		particle.r = 0.08f;
		particle.c = color_lut[int(rand_uniform() * color_lut.size())];
		particle.v = {0, 0, 0};
		particle.m = 5.0f; 

		particles.push_back(particle);
	}
}


void scene_structure::display_gui()
{
	ImGui::SliderFloat("Time scale", &timer.scale, 0.05f, 2.0f, "%.2f s");
	ImGui::SliderInt("Number of balls", &gui.sphere_count, 1,10);	
	if(ImGui::Button("Start Race (SPACE)")) 
	{
		start_race();
	}

}

void scene_structure::mouse_move_event()
{
	if (!inputs.keyboard.shift)
		camera_control.action_mouse_move(environment.camera_view);
}
void scene_structure::mouse_click_event()
{
	camera_control.action_mouse_click(environment.camera_view);
}
void scene_structure::keyboard_event()
{
	camera_control.action_keyboard(environment.camera_view);
	if(inputs.keyboard.last_action.is_pressed(GLFW_KEY_SPACE)) 
	{
		start_race();
	}
}
void scene_structure::idle_frame()
{
	camera_control.idle_frame(environment.camera_view);
}

