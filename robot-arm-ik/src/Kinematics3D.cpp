/*
*
* File name: Kinematics3D.cpp
*
* Implement the forward and inverse kinematics computation in 3D space
*
*/

#include "arm/Kinematics3D.h"
#include <cmath>
#include <iostream>

namespace {
	// Helper function to compute normalized angle
	float NormalizeAngle(float angle) {
		while (angle > 3.14159265f) angle -= 2.0f * 3.14159265f;
		while (angle < -3.14159265f) angle += 2.0f * 3.14159265f;
		return angle;
	}
}

float Kinematics3D::EaseAngle(float current_angle, float target_angel, float ease_speed, float dt) {
	float t = 1.0f - std::exp(-ease_speed * dt);
	return current_angle + NormalizeAngle(target_angel - current_angle) * t;
}

std::vector<Eigen::Vector3f> Kinematics3D::ComputeJointPositions(ArmChain3D const& chain) {
	std::vector<Eigen::Vector3f> joint_positions{};
	joint_positions.reserve(chain.JointCount() + 1);
	joint_positions.push_back(chain.Base());
	Eigen::Vector3f current_pos = chain.Base();
	
	// We use quaternions for angles here as it avoids Gimbal Lock and can support
	//spherical interpolation to transition smoothly between 2 orientations 
	Eigen::Quaternionf cumulative_orientation = Eigen::Quaternionf::Identity();

	// Iterate through all joints
	for (size_t i = 0; i < chain.JointCount(); i++) {
		Joint3D const& joint = chain.GetJoint(i);
		
		// The key difference here is that since this is in 3D, we must take into account 
		//the axis by which we are rotating each joint
		// Luckily, Eigen has a very useful type AngleAxis that constructs this transformation
		Eigen::AngleAxisf local_rotation(joint.angle, joint.axis);								// Set up the local rotation
		cumulative_orientation = cumulative_orientation * local_rotation;						// Compose cumulative orientation with local rotation
		Eigen::Vector3f step = cumulative_orientation * Eigen::Vector3f(joint.length, 0, 0);	// Get the step orientation by length (along local +ve X axis)
		current_pos = current_pos + step;			// Step along local +ve X axis;
		joint_positions.push_back(current_pos);		// Push into joint_positions
	}
	return joint_positions;
}

std::vector<Eigen::Quaternionf> Kinematics3D::ComputeJointOrientation(ArmChain3D const& chain) {
	std::vector<Eigen::Quaternionf> orientations;
	orientations.reserve(chain.JointCount());
	Eigen::Quaternionf cumulative_orientation = Eigen::Quaternionf::Identity();

	// Iterate through all joints
	for (size_t i = 0; i < chain.JointCount(); i++) {
		orientations.push_back(cumulative_orientation);	// Push back FIRST as joint 0 should not need/have any rotation
		Joint3D const& joint = chain.GetJoint(i);
		Eigen::AngleAxisf local_rotation(joint.angle, joint.axis);
		cumulative_orientation = cumulative_orientation * local_rotation;
	}
	return orientations;
}

std::vector<Kinematics3D::LinkTransform> Kinematics3D::ComputeLinkTransforms(ArmChain3D const& chain) {
	std::vector<Kinematics3D::LinkTransform> transforms;
	transforms.reserve(chain.JointCount());

	// Similar to ComputeJointPosition, start current_pos at base and set up the cumulative orientation
	Eigen::Vector3f current_pos = chain.Base();
	Eigen::Quaternionf cumulative_orientation = Eigen::Quaternionf::Identity();

	for (size_t i = 0; i < chain.JointCount(); i++) {
		Joint3D const& joint = chain.GetJoint(i);
		Eigen::AngleAxisf local_rotation(joint.angle, joint.axis);
		cumulative_orientation = cumulative_orientation * local_rotation;

		// Main difference here is that we take each iteration of position, cumulative orientation and length, starting
		//right before the step happens. This is crucial as we need to start with the initial chain base and identity cumulative orienation
		//so that subsequent constructed links will be rendered with the correct orientation and length.
		transforms.push_back({current_pos, cumulative_orientation, joint.length});

		Eigen::Vector3f step = cumulative_orientation * Eigen::Vector3f(joint.length, 0, 0);
		current_pos = current_pos + step;
	}
	return transforms;
}

Eigen::Matrix4f Kinematics3D::BuildLinkModelMatrix(Kinematics3D::LinkTransform const& lt, float thickness) {
	Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
	Eigen::Matrix3f rotation = lt.orientation.toRotationMatrix();
	Eigen::Matrix3f scale = Eigen::Matrix3f::Identity();
	scale(0, 0) = lt.length;
	scale(1, 1) = thickness;
	scale(2, 2) = thickness;
	model.block<3, 3>(0, 0) = rotation * scale;		// scale -> rotate, in local space
	model.block<3, 1>(0, 3) = lt.position;			// translate
	return model;
}

Eigen::Matrix4f Kinematics3D::BuildMarkerModelMatrix(Eigen::Vector3f position, float size) {
	Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
	Eigen::Matrix3f scale = Eigen::Matrix3f::Identity() * size;
	model.block<3, 3>(0, 0) = scale;
	model.block<3, 1>(0, 3) = position - Eigen::Vector3f(size * 0.5f, 0, 0);
	return model;
}

Eigen::Vector3f Kinematics3D::project_perpendicular(Eigen::Vector3f v, Eigen::Vector3f axis) {
	return v - v.dot(axis) * axis;   // removes the component along axis, leaves only the perpendicular part
}

float Kinematics3D::SolveCCD_3D(ArmChain3D& chain, Eigen::Vector3f target, int max_passes, float tolerance) {
	std::vector<Eigen::Vector3f> joint_positions;
	std::vector<Eigen::Quaternionf> joint_orienations;
	float dist{};

	// Iterate through specified pass count 
	for (int pass = 0; pass < max_passes; pass++) {
		Eigen::Vector3f current_joint_pos{}, end_effector_pos{};
		Eigen::Vector3f to_end_effector{}, to_target{};

		// Get positions and orienataions of all joints
		joint_positions = ComputeJointPositions(chain);
		joint_orienations = ComputeJointOrientation(chain);

		// Iterate through all joints starting from the last 
		for (size_t i = chain.JointCount(); i-- > 0;) {
			Joint3D const& joint = chain.GetJoint(i);
			Eigen::Vector3f world_axis = (joint_orienations[i] * joint.axis).normalized();

			// Get current joint position
			current_joint_pos = joint_positions[i];

			// Get end effector position
			end_effector_pos = joint_positions[chain.JointCount()];

			// Vector of joint position to end effector
			to_end_effector = project_perpendicular(end_effector_pos - current_joint_pos, world_axis);

			// Vector of joint position to target
			to_target = project_perpendicular(target - current_joint_pos, world_axis);

			// Degnereate case: skip if to_end_effector or to_target comes out to near 0 as
			//this means that the vector is (nearly) parallel to the joint axis, so no rotation can
			//occur.
			constexpr float EPSILON = 1e-5f;
			if (to_end_effector.squaredNorm() < EPSILON || to_target.squaredNorm() < EPSILON) {
				continue;
			}

			float angle_to_turn = std::atan2(world_axis.dot(to_end_effector.cross(to_target)), to_end_effector.dot(to_target));
			chain.SetJointAngle(i, chain.GetJoint(i).angle + angle_to_turn);

			// Refresh positions and orienataions of all joints
			joint_positions = ComputeJointPositions(chain);
			joint_orienations = ComputeJointOrientation(chain);
		}
		dist = (joint_positions.back() - target).norm();
		if (dist < tolerance) break;
	}
	return dist;
}