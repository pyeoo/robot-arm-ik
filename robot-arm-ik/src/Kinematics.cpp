/*
*
* File name: Kinematics.cpp
*
* Implement the forward and inverse kinematics computation
*
*/

#include "arm/Kinematics.h"
#include <cmath>
#include <algorithm>

namespace {
	// Helper function to compute normalized angle
	float NormalizeAngle(float angle) {
		while (angle > 3.14159265f) angle -= 2.0f * 3.14159265f;
		while (angle < -3.14159265f) angle += 2.0f * 3.14159265f;
		return angle;
	}

	// Helper function to compute distance between 2 Vec2 objects
	float Distance(Vec2 a, Vec2 b) {
		return std::sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y));
	}
}

float Kinematics::EaseAngle(float current_angle, float target_angel, float ease_speed, float dt) {
	float t = 1.0f - std::exp(-ease_speed * dt);
	return current_angle + NormalizeAngle(target_angel - current_angle) * t;
}

std::vector<Vec2> Kinematics::ComputeJointPositions(ArmChain const& chain) {
	std::vector<Vec2> joint_positions{};
	joint_positions.reserve(chain.JointCount() + 1);
	joint_positions.push_back({ chain.BaseX(), chain.BaseY() });	// Start from base position (push base position into the joints position container)
	Vec2 current_pos{}; current_pos.x = chain.BaseX(); current_pos.y = chain.BaseY();	// Store the base position, to be added.
	float cumulative_angle{};	// Stores cumulative angles

	// Iterate through all joints 
	for (size_t i = 0; i < chain.JointCount(); i++) {

		// Accumulate angle
		Joint const& joint = chain.GetJoint(i);
		cumulative_angle += joint.angle;

		// Increment step by lenth of each link and its direction
		current_pos.x += joint.length * std::cos(cumulative_angle);
		current_pos.y += joint.length * std::sin(cumulative_angle);

		joint_positions.push_back(current_pos);
	}
	return joint_positions;
}

float Kinematics::SolveCCD(ArmChain& chain, Vec2 const target, int max_passes, float tolerance) {
	// Get all joint positions
	std::vector<Vec2> joint_positions{};
	joint_positions.reserve(chain.JointCount() + 1);
	joint_positions = ComputeJointPositions(chain);
	float dist_end_effector_to_target{};

	// Iterate through specified pass count
	for (size_t pass = 1; pass <= max_passes; pass++) {
		Vec2 current_joint_pos, end_effector_pos;
		Vec2 to_end_effector, to_target;
		// Iterate through all joints
		for (size_t i = chain.JointCount(); i-- > 0; ) {
			Joint const& joint = chain.GetJoint(i);

			current_joint_pos = joint_positions[i];
			end_effector_pos = joint_positions[chain.JointCount()];

			to_end_effector = end_effector_pos - current_joint_pos;
			to_target = target - current_joint_pos;

			float angle_to_turn = std::atan2(to_target.y, to_target.x) - std::atan2(to_end_effector.y, to_end_effector.x);
			angle_to_turn = NormalizeAngle(angle_to_turn);

			chain.SetJointAngle(i, chain.GetJoint(i).angle + angle_to_turn);
			joint_positions = ComputeJointPositions(chain);
		}
		dist_end_effector_to_target = Distance(joint_positions.back(), target);
		if (dist_end_effector_to_target < tolerance) break;
	}
	return dist_end_effector_to_target;
}