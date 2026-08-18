/*
*
* File name: ArmChain.h
*
* Stores minimum information needed to compute link lengths (fixed) and joint angles (changes every frame)
*
*/
#pragma once
#include <vector>

struct Joint {
	float angle;		// Joint angle, relative to previous link's orientation (in radians)
	float length;		// Fixed length of link that this joint controls
	float min_angle;	// Joint angle limit (lower bound)
	float max_angle;	// Joint angle limit (upper bound)
};


class ArmChain {
public:
	// ctor
	explicit ArmChain(std::vector<Joint> joints, float baseX = 0.f, float baseY = 0.f);

	// Mutators
	void SetJointAngle(size_t index, float angle);
	void RotateJoint(size_t index, float angle);

	// Accessors
	Joint const& GetJoint(size_t index) const { return joints_[index]; }
	size_t JointCount() const { return joints_.size(); }
	float BaseX() const { return baseX_; }
	float BaseY() const { return baseY_; }

private:
	std::vector<Joint> joints_;
	float baseX_, baseY_;
};