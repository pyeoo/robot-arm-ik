/*
*
* File name: ArmChain.h
*
* Stores minimum information needed to compute link lengths (fixed) and joint angles (changes every frame), 
*this time for a 3D arm
*
*/
#pragma once
#include <vector>
#include <Eigen/Dense>

struct Joint3D {
	float angle;			// Joint angle, relative to previous link's orientation (in radians)
	float length;			// Fixed length of link that this joint controls
	float min_angle;		// Joint angle limit (lower bound)
	float max_angle;		// Joint angle limit (upper bound)
	Eigen::Vector3f axis;	// Fixed hinge axis
};


class ArmChain3D {
public:
	// ctor
	explicit ArmChain3D(std::vector<Joint3D>, Eigen::Vector3f base = Eigen::Vector3f::Zero());

	// Mutators
	void SetJointAngle(size_t index, float angle);
	void RotateJoint(size_t index, float d_angle);

	// Accessors
	Joint3D const& GetJoint(size_t index) const { return joints_[index]; }
	size_t JointCount() const { return joints_.size(); }
	Eigen::Vector3f Base() const { return base_; }

private:
	std::vector<Joint3D> joints_;
	Eigen::Vector3f base_;
};