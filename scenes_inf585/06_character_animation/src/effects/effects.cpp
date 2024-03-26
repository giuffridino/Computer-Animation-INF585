#include "effects.hpp"

using namespace cgp;

//#undef SOLUTION

// Update a transition_structure before starting a new transition
void effect_transition_start(effect_transition_structure& effect_transition, character_structure& character, std::string const& destination_anim)
{
	effect_transition.source_anim = character.current_animation_name;
	effect_transition.timer_source_anim = character.timer;

	effect_transition.destination_anim = destination_anim;
	effect_transition.timer_destination_anim.event_period = character.animated_model.animation.at(destination_anim).time_max;
	effect_transition.timer_destination_anim.t_periodic=0;
	effect_transition.timer_destination_anim.start();

	effect_transition.timer_completion.t=0;
	effect_transition.timer_completion.scale = 1.0f/effect_transition.transition_time; // force the timer to go from 0 to 1 during the transition_time
	effect_transition.timer_completion.start();
	effect_transition.active = true;

	character.current_animation_name = destination_anim;
}

// Check if a transition need to be de-activated if it is fully completed
void effect_transition_stop_if_completed(effect_transition_structure& transition, character_structure& character) {
	if(transition.timer_completion.t >= 1.0f) {
		character.set_current_animation(transition.destination_anim);
		character.timer = transition.timer_destination_anim;
		transition.active = false;
	}
}


void effect_transition_compute(effect_transition_structure& effect_transition, character_structure& character)
{

	effect_transition_structure& transition = effect_transition;
	animated_model_structure& model = character.animated_model;

	// Compute the skeleton from the source animation
	model.set_skeleton_from_animation(transition.source_anim, transition.timer_source_anim.t_periodic);
	numarray<mat4> joint_source_anim_local = model.skeleton.joint_matrix_local;
	numarray<mat4> joint_source_anim_global = model.skeleton.joint_matrix_local;

	// Compute the skeleton from the destination animation
	model.set_skeleton_from_animation(transition.destination_anim, transition.timer_destination_anim.t_periodic);
	numarray<mat4> joint_destination_anim_local = model.skeleton.joint_matrix_local;
	numarray<mat4> joint_destination_anim_global = model.skeleton.joint_matrix_global;

	float alpha_completion = transition.timer_completion.t;

	/** TO DO : Compute a smooth transition between two animation 
	 *  The objective is to fill "model.skeleton.joint_matrix_local and global"
	 *     with the correct interpolated matrices corresponding to the blending between the source and destination animation 
	 * 
	 * 	
	 * 
	 *  Notes:
	 *    - model.skeleton.joint_matrix_local is a numarray<mat4>. Its size corresponds to the number of joints.
	 *    - joint_source_anim contains the (local) matrices of the source animation
	 *    - joint_destination_anim contains the (local) matrices of the destination animation
	 * 	  - alpha_completion contains the ratio of completion.
	 *       e.g.: alpha_completion=0 => we have the source anim
	 *             alpha_completion=1 => we have the destination anim
	 *             alpha_completion=0.5 => we have "0.5 source anim + 0.5 destination anim"
	 *    - if model.skeleton.joint_matrix_local is filled, you can update the global matrices using "model.skeleton.update_joint_matrix_local_to_global();"
	 *    - if model.skeleton.joint_matrix_global is filled, you can update the global matrices using "model.skeleton.update_joint_matrix_global_to_local();"
	 * 
	 *  Help:
	 *    - You can convert a matrix M to its translation t and rotation r (or quaternion q) representation using the following syntax:
	 *      affine_rt a = affine_rt::from_matrix(M);
	 *      vec3 t = a.translation;
	 *      rotation_transform r = a.rotation;
	 *      quaternion q = r.get_quaternion();
	 *    - An affine_rt structure is a container for a rotation_transform and a translation. Its corresponding 4x4 matrix can be obtained
	 *        with {affine_rt}.matrix();
	 * 
	 * 
	 */
	for (unsigned int i = 0; i < model.skeleton.joint_matrix_local.size(); i++)
	{
		affine_rt source_rt = (affine_rt::from_matrix(joint_source_anim_local[i]));
        affine_rt destination_rt = affine_rt::from_matrix(joint_destination_anim_local[i]);
		vec3 interpolated_translation = (1 - alpha_completion) * source_rt.translation + alpha_completion * destination_rt.translation;
		quaternion source_rotation = normalize(source_rt.rotation.data);
        quaternion destination_rotation = normalize(destination_rt.rotation.data);
        quaternion interpolated_rotation = slerp(source_rotation, destination_rotation, alpha_completion);
		affine_rt interpolated_rt = affine_rt(interpolated_rotation, interpolated_translation);
		model.skeleton.joint_matrix_local[i] = interpolated_rt.matrix();
	}
	model.skeleton.update_joint_matrix_local_to_global();


	// Update the timers
	transition.timer_source_anim.update();
	transition.timer_destination_anim.update();
	transition.timer_completion.update();
	character.timer = transition.timer_destination_anim;
}




// This function implements the change of animation when we press or release the UP key during a walk effect
void effect_walking_keyboard_event(effect_transition_structure& effect_transition, character_structure& character, cgp::input_devices const& inputs, effect_walking_structure const& effect_walking)
{
	// If we just press the key up (or W), start the Walk cycle animation
	if(inputs.keyboard.last_action.is_pressed(GLFW_KEY_UP) || inputs.keyboard.last_action.is_pressed(GLFW_KEY_W)) {
		effect_transition_start(effect_transition, character, effect_walking.walk_anim_name);
	}
	// If we just release the key up (or W), start the Idle cycle animation
	if(inputs.keyboard.last_action.is_released(GLFW_KEY_UP) || inputs.keyboard.last_action.is_released(GLFW_KEY_W)) {
		effect_transition_start(effect_transition, character, effect_walking.idle_anim_name);
	}

}


// This function implements the change of position of the character when a directional key is pressed
// This function is called at every frame when the walk effect is active
void effect_walking(effect_walking_structure& effect_walking,  character_structure& character, cgp::input_devices const& inputs, effect_transition_structure const& effect_transition)
{

	/** TO DO : An displacement of the character along the direction of the walk.
	 *   The character should be able to move straight, and turn, based on the user command.
	 *  
	 *   Inputs: 
	 *      - The effect_walking structure storing the current position and angle of the character (root joint position and angle)
	 *      - The skeleton to be modified by the walk effect
	 *      - The current state of the keyboard input (e.g. inputs.keyboard.is_pressed(...))
	 *   Output: 
	 * 		- A modified effect_walking structure and skeleton corresponding to the expected displacement
	 * 		  
	 * 
	 *   Help:
	 *      - The following syntax allows to check if a specific key is currently pressed:
	 *        if( inputs.keyboard.is_pressed({KeyName}) ) {...} , with
	 * 		   {KeyName} being: GLFW_KEY_UP, GLFW_KEY_RIGHT, GLFW_KEY_LEFT, GLFW_KEY_DOWN for the arrows
	 *                      or: GLFW_KEY_W, GLFW_KEY_A, GLFW_KEY_D, GLFW_KEY_S using for instance the letters
	 *      - Given a mat4 structure representing an affine transformation, you can apply the following block operations
	 *        - {mat4}.get/set_block_translation({vec3}) : get/set the vec3 block corresponding to the translation part
	 *        - {mat4}.get/set_block_linear({mat3}) : get/set the mat3 block corresponding to the linear part
	 *      - A rotation with a given axis and angle can be created with the syntax
	 *         rotation_transform r = rotation_axis_angle({vec3 axis}, {float angle});
	 *         The associated 3x3 matrix can be obtained using r.matrix();  
	 *      - An internal timer is available in effect_walking.timer
	 *        The timer can be updated via effect_walking.timer.update() and the elapsed time since the last update is returned.
	 *        This can be used to enforce a constant speed independently of the frame rate
	 * 
	 * 
	 * 	 Algorithm:     
	 * 		If(press UP)
	 *        Advance straight and update effect_walking.root_position
	 *      If(press Left/Right)
	 *        Rotate and update effect_walking.root_angle
	 *      Update the root joint of the skeleton
	 * 
	 * 
	 */
	// std::cout << "Testing, effect_walking.active: " << effect_walking.active << "\n";
	float speed = 0.05f;
	if(inputs.keyboard.is_pressed(GLFW_KEY_UP) || inputs.keyboard.is_pressed(GLFW_KEY_W)) 
	{
		vec3 forward_direction = vec3(sin(effect_walking.root_angle), 0.0f, cos(effect_walking.root_angle));
		vec3 delta_position = forward_direction * speed;
		effect_walking.root_position += delta_position;
	}
	if(inputs.keyboard.is_pressed(GLFW_KEY_DOWN) || inputs.keyboard.is_pressed(GLFW_KEY_S)) 
	{
		vec3 forward_direction = vec3(sin(effect_walking.root_angle), 0.0f, cos(effect_walking.root_angle));
		vec3 delta_position = forward_direction * speed;
		effect_walking.root_position -= delta_position;
	}

	float angleSpeed = 0.1f;
	if(inputs.keyboard.is_pressed(GLFW_KEY_RIGHT) || inputs.keyboard.is_pressed(GLFW_KEY_D)) 
	{
		effect_walking.root_angle -= angleSpeed;
	}
	if(inputs.keyboard.is_pressed(GLFW_KEY_LEFT) || inputs.keyboard.is_pressed(GLFW_KEY_A)) 
	{
		effect_walking.root_angle += angleSpeed;
	}

	// Set position after calculating the root_position and angle
	rotation_transform r = rotation_axis_angle(vec3(0.0f, 1.0f, 0.0f), effect_walking.root_angle);
	quaternion q = normalize(r.quat());
	mat3 r_mat = rotation_transform::convert_quaternion_to_matrix(q);
	character.animated_model.skeleton.joint_matrix_global[0].set_block_translation(effect_walking.root_position);
	character.animated_model.skeleton.joint_matrix_global[0].set_block_linear(r_mat);
	for (unsigned int i = 1; i < character.animated_model.skeleton.joint_matrix_local.size(); i++)
	{
		int parent_index = character.animated_model.skeleton.parent_index[i];
		character.animated_model.skeleton.joint_matrix_global[i] = character.animated_model.skeleton.joint_matrix_global[parent_index] * character.animated_model.skeleton.joint_matrix_local[i];
	}
	character.animated_model.skeleton.update_joint_matrix_global_to_local();
}


// Rotate the joint of the head to look toward the given objective position (if possible)
void effect_rotate_head_toward_objective_position(skeleton_structure& skeleton, int joint_head_index, vec3 const& objective_position)
{
	/** TO DO : Change the direction of the joint of the head such that it points toward the objective position given in parameters
	 *  
	 *  Notes:
	 *    - the index of the joint corresponding to the head is supposed to be given in parameter in the variable joint_head_index
	 *      The associated matrix is accessible via "skeleton.joint_matrix_local/global[joint_head_index]"
	 *    - Given a mat4 structure representing an affine transformation, you can apply the following block operations
	 *      - {mat4}.get_block_linear() : returns the mat3 corresponding to the linear part (typically the rotation)
	 *      - {mat4}.get_block_translation() : returns the vec3 corresponding to the translation part
	 *      - {mat4}.apply_transform_to_block_linear({mat3}) : Applies the mat3 matrix to the linear part of the mat4 matrix (doesn't modify the translation part)
	 *    - The transpose of a matrix M (mat3 or mat4) can be obtained with transpose(M), or transpose(M)
	 *    - A rotation with a given axis and angle can be created with the syntax
	 *      rotation_transform r = rotation_axis_angle({vec3 axis}, {float angle});
	 *      The associated 3x3 matrix can be obtained using r.matrix();
	 * 
	 *  skeleton.joint_matrix_local/global[joint_head_index] = ... // something that depends on the objective_position
	 * 
	 */

	vec3 head_position = skeleton.joint_matrix_local[joint_head_index].get_block_translation();
	vec3 direction = normalize(objective_position - head_position);
	mat3 inverse_body_rotation = inverse(skeleton.joint_matrix_local[0].get_block_linear());
	vec3 adjusted_direction = inverse_body_rotation * direction;
	float max_angle_yaw = 0.6f;
	float max_angle_pitch = 0.8f;
	float yaw = atan2(adjusted_direction.x, adjusted_direction.z);
	float pitch = atan2(-adjusted_direction.y, sqrt(adjusted_direction.x * adjusted_direction.x + adjusted_direction.z * adjusted_direction.z));
	yaw = clamp(yaw, -max_angle_yaw, max_angle_yaw);
	pitch = clamp(pitch, -max_angle_pitch, max_angle_pitch);
	rotation_transform rY = rotation_axis_angle({0.0f, 1.0f, 0.0f}, {yaw});
	rotation_transform rX = rotation_axis_angle({1.0f, 0.0f, 0.0f}, {pitch});
	rotation_transform r = rY * rX;
	skeleton.joint_matrix_local[joint_head_index].apply_linear_transform(r.matrix());
	skeleton.update_joint_matrix_local_to_global();

	// vec3 head_position = skeleton.joint_matrix_local[joint_head_index].get_block_translation();
	// mat3 current_direction = skeleton.joint_matrix_local[joint_head_index].get_block_linear();
	// vec3 direction = normalize(objective_position - head_position);
	
	// float max_angle_yaw = 0.6f;
	// float max_angle_pitch = 0.8f;
	// float yaw = atan2(direction.x, direction.z);
	// float pitch = atan2(-direction.y, sqrt(direction.x * direction.x + direction.z * direction.z));
	// // std::cout << "yaw: " << yaw << "\n";
	// yaw = clamp(yaw, -max_angle_yaw, max_angle_yaw);
	// pitch = clamp(pitch, -max_angle_pitch, max_angle_pitch);
	// rotation_transform rY = rotation_axis_angle({0.0f, 1.0f, 0.0f}, {yaw});
	// rotation_transform rX = rotation_axis_angle({1.0f, 0.0f, 0.0f}, {pitch});
	// rotation_transform r = rY * rX;
	// skeleton.joint_matrix_local[joint_head_index].apply_linear_transform(r.matrix());
	// skeleton.update_joint_matrix_local_to_global();
}



// Update the current position of the IK constraint to match at its start the current joint position
void effect_ik_start(effect_ik_structure& effect_ik, skeleton_structure& skeleton, int joint_end)
{
	// get current position of the joint at index joint_end
	vec3 current_position = skeleton.joint_matrix_global[joint_end].get_block_translation();
	// set this position as the default objective for the IK
	effect_ik.target_position = current_position;
	effect_ik.target_offset = {0,0,0};
}

// Function that makes sure not to normalize if distance is too little (avoid division by 0)
vec3 normalize_direction(vec3 direction)
{
	float norm_direction = norm(direction);
	if (norm_direction <= 1e-3)
	{
		return direction;
	}
	else
	{
		return direction / norm_direction;
	}
}


void effect_ik_compute(effect_ik_structure const& effect_ik, skeleton_structure& skeleton)
{
	/** TO DO : Implement an Inverse Kinematic on the skeleton
	 *  
	 *   Inputs: 
	 *      - The skeleton to be modified by the IK
	 *      - The index of the joint constrained by the IK : effect_ik.joint_end
	 *      - The index of the joint at the beginning of the skeleton chain : effect_ik.joint_start
	 *        (effect_ik.joint_start should be a parent of effect_ik.joint_end, otherwise the start of the chain can be considered to be the skeleton root)
	 *      - The position associated to constrained joint: 
	 *          p_constrained = effect_ik.objective_position + objective_offset.effect_ik
	 *        Rem. this position is split between an initial position and an offset that can be manipulated via the GUI.
	 *   Output: 
	 * 		- A modified joint position of the skeleton satisfying at best the constraint.
	 * 
	 *   Rem. 
	 *      - effect_ik.joint_start should be a parent of effect_ik.joint_end such that the set of joints between [joint_start - joint_end] is a kinematic chain, subset of the skeleton hierarchy. If it is not the case, we may assume that the start of the chain is at the skeleton root.
	 * 
	 *   Help:
	 *      - A rotation R transforming a vector v1 to a vector v2 (with ||v1||=||v2||=1) can be computed using
	 *        rotation_transform R = rotation_transform::from_vector_transform(v1, v2);
	 *      - Considering a mat4 representing an affine transform:
	 *         - {mat4}.get_block_translation() gives the vec3 corresponding to the translation part
	 *         - {mat4}.set_block_translation({vec3}) set the block to the mat4 corresponding to the translation with the vec3 passed as argument
	 *         - {mat4}.apply_transform_to_block_linear({mat3}) : Applies the mat3 matrix to the linear part of the mat4 matrix (doesn't modify the translation part)
	 *         - mat4 M_inv = {mat4}.inverse_assuming_rigid_transform() : gives the 4x4 matrix corresponding to the inverse of the {mat4}
	 *      - mat4().set_identity() generate a 4x4 identity matrix
	 *      
	 */

	//Check that the joint_end is a child of the joint_start
	int current_joint = effect_ik.joint_target;
	int root_joint = effect_ik.joint_root_ik;
	numarray<int> child_joints;
	numarray<float> joint_lengths;
	child_joints.push_back(current_joint);
	while (true)
	{
		if (skeleton.parent_index[current_joint] == effect_ik.joint_root_ik)
		{
			joint_lengths.push_back(norm(skeleton.joint_matrix_global[current_joint].get_block_translation() - skeleton.joint_matrix_global[skeleton.parent_index[current_joint]].get_block_translation()));
			child_joints.push_back(root_joint);
			break;
		}
		if (skeleton.parent_index[current_joint] == -1)
		{
			root_joint = 0;
			break;
		}
		int parent_joint = skeleton.parent_index[current_joint];
		joint_lengths.push_back(norm(skeleton.joint_matrix_global[current_joint].get_block_translation() - skeleton.joint_matrix_global[parent_joint].get_block_translation()));
		current_joint = parent_joint;
		child_joints.push_back(current_joint);
	}
	// std::cout << "joint_lengths: " << joint_lengths << "\n";
	// std::cout << "finished printing joint_lengths\n";
	//Use root_joint now and not effect_ik.joint_root_ik
	// std::cout << "child_joints: " << child_joints << "\n";
	int num_c_joints = child_joints.size();
	float min_dist = 0.1f;
	int count = 0;
	while (norm(skeleton.joint_matrix_global[effect_ik.joint_target].get_block_translation() - (effect_ik.target_position + effect_ik.target_offset)) > min_dist) 
	{
		vec3 root_pos = skeleton.joint_matrix_global[root_joint].get_block_translation();
		vec3 current_target_position = effect_ik.target_position + effect_ik.target_offset;
		//Backward phase
		std::cout << "num_c_joints: " << num_c_joints << "\n";
		// std::cout << "backward phase starting\n";
		numarray<vec3> back_positions;
		back_positions.push_back(current_target_position);
		for(int i = 0; i < num_c_joints - 1; i++)
		{
			int joint = child_joints[i];
			int parent_joint = skeleton.parent_index[joint];
			// if (joint == 26)
			// {
			// 	std::cout << "skel.joint_matrix_global[joint]: " << skeleton.joint_matrix_global[joint] << "\n";
			// 	std::cout << "skel.joint_matrix_global[parent_joint]: " << skeleton.joint_matrix_global[parent_joint] << "\n";
			// }
			
			vec3 v1 = skeleton.joint_matrix_global[joint].get_block_translation() - skeleton.joint_matrix_global[parent_joint].get_block_translation();
			float joint_length = norm(v1);
			// std::cout << "joint_length bw: " << joint_length << "\n";
			vec3 v2 = current_target_position - skeleton.joint_matrix_global[parent_joint].get_block_translation();
			vec3 direction = normalize_direction(-v2);
			current_target_position = current_target_position + normalize(direction) * joint_lengths[i];
			// current_target_position = current_target_position + normalize(direction) * joint_length;
			back_positions.push_back(current_target_position);
		}
		// std::cout << "back_positions: " << back_positions << "\n";
		// std::cout << "backward phase completed\n";
		// std::cout << "forward phase starting\n";
		numarray<vec3> forward_positions;
		forward_positions.push_back(root_pos);
		current_target_position = root_pos;
		for (int i = num_c_joints - 1; i >= 1; i--)
		{
			vec3 v1 = back_positions[i - 1] - current_target_position;
			float joint_length = norm(back_positions[i - 1] - back_positions[i]);
			// std::cout << "joint_length fw: " << joint_length << "\n";
			vec3 direction = normalize_direction(v1);
			current_target_position = current_target_position + normalize(direction) * joint_lengths[i - 1];
			// current_target_position = current_target_position + normalize(direction) * joint_length;
			forward_positions.push_back(current_target_position);
		}
		for (int i = 0; i < num_c_joints - 2; i++)
		{
			int joint = child_joints[i];
			int parent_joint = child_joints[i + 1];
			vec3 v1 = skeleton.joint_matrix_global[joint].get_block_translation() - skeleton.joint_matrix_global[parent_joint].get_block_translation();
			vec3 v2 = forward_positions[num_c_joints - 1 - i] - forward_positions[num_c_joints - 2 - i];
			rotation_transform R = rotation_transform::from_vector_transform(normalize(v1), normalize(v2));
			mat4 M_global = skeleton.joint_matrix_global[joint];
			M_global.apply_linear_transform(R.matrix());
			M_global.set_block_translation(forward_positions[num_c_joints - 1 - i]);
			skeleton.joint_matrix_local[joint] = skeleton.joint_matrix_global[parent_joint].inverse_assuming_rigid_transform() * M_global;
			// skeleton.joint_matrix_global[joint].apply_linear_transform(R.matrix());
			// skeleton.joint_matrix_global[joint].set_block_translation(forward_positions[num_c_joints - 1 - i]);
		}
		skeleton.update_joint_matrix_local_to_global();
		// skeleton.update_joint_matrix_global_to_local();
		std::cout << "forward phase completed\n";
		count++;
		if (count > 10)
		{
			break;
		}		
	}
}


