/*
*
* File name: ArmChain3D.cpp
*
* Implements 3D Arm Chain properties
*
*/

#include <cassert>
#include <algorithm>
#include "arm\ArmChain3D.h"

ArmChain3D::ArmChain3D(std::vector<Joint3D> joints, Eigen::Vector3f base) : joints_{ std::move(joints) }, base_{ base } {}

void ArmChain3D::SetJointAngle(size_t index, float angle) {
	assert(index < joints_.size());	// Bounds check
	joints_[index].angle = std::clamp(angle, joints_[index].min_angle, joints_[index].max_angle);
}

void ArmChain3D::RotateJoint(size_t index, float d_angle) {
	assert(index < joints_.size());
	float target = joints_[index].angle + d_angle;
	joints_[index].angle = std::clamp(target, joints_[index].min_angle, joints_[index].max_angle);
}