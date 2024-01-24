#pragma once


#include "cgp/cgp.hpp"
#include "environment.hpp"


using cgp::mesh_drawable;
using cgp::curve_drawable;
using cgp::mesh;
using cgp::numarray;
using cgp::vec3;



struct gui_parameters {
	bool display_frame = true;
	bool display_wireframe = false;
};

// The structure of the custom scene
struct scene_structure : cgp::scene_inputs_generic {
	
	// ****************************** //
	// Elements and shapes of the scene
	// ****************************** //
	camera_controller_orbit_euler camera_control;
	camera_projection_perspective camera_projection;
	window_structure window;

	mesh_drawable global_frame;          // The standard global frame
	environment_structure environment;   // Standard environment controler
	input_devices inputs;                // Storage for inputs status (mouse, keyboard, window dimension)
	gui_parameters gui;                  // Standard GUI element storage
	
	// ****************************** //
	// Elements and shapes of the scene
	// ****************************** //

	cgp::timer_basic timer;

	mesh_drawable cube;
	mesh_drawable ground;
	mesh_drawable cylinder;
	mesh_drawable sphere;

	mesh shape;
	numarray<vec3> initial_position;
	mesh_drawable shape_visual;

	curve_drawable curve;    // A set of vertices displayed as a curve


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

	void evolve_shape(float time);

};





