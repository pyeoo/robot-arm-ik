/*
*
* File name: ArmChain.cpp
*
* Implements SetJointAngle and RotateJoint as defined in ArmChain.h
*
*/

#include <cassert>
#include <algorithm>
#include "arm\ArmChain.h"

ArmChain::ArmChain(std::vector<Joint> joints, float baseX, float baseY) : joints_{ std::move(joints) }, baseX_ { baseX }, baseY_{ baseY } {}

void ArmChain::SetJointAngle(size_t index, float angle) {
	assert(index < joints_.size());	// Bounds check on index
	joints_[index].angle = std::clamp(angle, joints_[index].min_angle, joints_[index].max_angle);	// Clamp angle
}

void ArmChain::RotateJoint(size_t index, float d_angle) {
	assert(index < joints_.size());
	float target = joints_[index].angle + d_angle;
	joints_[index].angle = std::clamp(target, joints_[index].min_angle, joints_[index].max_angle);	// Clamp angle
}