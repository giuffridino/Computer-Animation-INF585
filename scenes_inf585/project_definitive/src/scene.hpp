#pragma once


#include "cgp/cgp.hpp"
#include "environment.hpp"

#include "simulation/simulation.hpp"

using cgp::mesh_drawable;


struct gui_parameters {
	bool display_frame = false;
	bool add_sphere = true;
	int sphere_count = 4;
	float radius = 0.6f;
	int N_sample_edge = 20;
	simulation_parameters cloth_parameters;
	float px = 0.0f;
	float py = 0.0f;
};

// The structure of the custom scene
struct scene_structure : scene_inputs_generic {
	
	// ****************************** //
	// Elements and shapes of the scene
	// ****************************** //
	camera_controller_first_person_euler camera_control;
	// camera_controller_orbit_euler camera_control;
	camera_projection_perspective camera_projection;
	window_structure window;

	environment_structure environment;   // Standard environment controler
	input_devices inputs;                // Storage for inputs status (mouse, keyboard, window dimension)
	gui_parameters gui;                  // Standard GUI element storage
	
	// ****************************** //
	// Elements and shapes of the scene
	// ****************************** //
	cgp::timer_event_periodic timer;
	std::vector<particle_structure> particles;
	cgp::mesh_drawable sphere;

	slide_structure slide;

	// Cloth related structures
	cloth_structure cloth;                     // The values of the position, velocity, forces, etc, stored as a 2D grid
	cloth_structure_drawable cloth_drawable;   // Helper structure to display the cloth as a mesh
	// simulation_parameters parameters;          // Stores the parameters of the simulation (stiffness, mass, damping, time step, etc)
	constraint_structure constraint;         // Handle the parameters of the constraints (fixed vertices, floor and sphere)
	int wind_counter = 0;
	float wind_magnitude = 7.0f;

	// Helper variables
	bool simulation_running = true;   // Boolean indicating if the simulation should be computed
	cgp::opengl_texture_image_structure cloth_texture;             // Storage of the texture ID used for the cloth

	void initialize_cloth(int N_sample); // Recompute the cloth from scratch

	// ****************************** //
	// Functions
	// ****************************** //

	void initialize();    // Standard initialization to be called before the animation loop
	void display_frame(); // The frame display to be called within the animation loop
	void display_gui();   // The display of the GUI, also called within the animation loop

	void mouse_move_event();
	void mouse_click_event();
	void keyboard_event();
	void idle_frame();

	void emit_particle(int count);
	void sphere_display();
	void start_race();
	void update_wind_magnitude();
};





