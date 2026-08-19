/*
*
* File name: Kinematics3D.cpp
*
* Implement the forward and inverse kinematics computation in 3D space
*
*/

#include "arm/Kinematics3D.h"

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
		Eigen::AngleAxisf local_rotation(joint.angle, joint.axis);								// Set up the local rotation
		cumulative_orientation = cumulative_orientation * local_rotation;						// Compose it with local_rotation
		Eigen::Vector3f step = cumulative_orientation * Eigen::Vector3f(joint.length, 0, 0);	// Get the step orientation by length (along local +ve X axis)
		current_pos = current_pos + step;			// Step along local +ve X axis;
		joint_positions.push_back(current_pos);		// Push into joint_positions
	}
	return joint_positions;
}