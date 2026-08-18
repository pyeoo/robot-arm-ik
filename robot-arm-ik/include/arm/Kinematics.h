/*
*
* File name: Kinematics.h
*
* Computes the forward and inverse kinematics 
*
*/
#pragma once
#include <vector>
#include <cmath>
#include "arm/ArmChain.h"


struct Vec2 {
    float x, y;

    Vec2 operator+(Vec2 const& other) const {
        return Vec2{ x + other.x, y + other.y };
    }
    Vec2 operator-(Vec2 const& other) const {
        return Vec2{ x - other.x, y - other.y };
    }
    Vec2 operator*(float scalar) const {
        return Vec2{ x * scalar, y * scalar };
    }
};

namespace Kinematics {
    // Computes every joint position in world space from the chain's current angles.
    // Returned vector size == chain.JointCount() + 1
    //   index 0         = base position
    //   index i (i>0)   = position after joint i-1's link is applied
    // Last element is the end-effector.
    std::vector<Vec2> ComputeJointPositions(ArmChain const& chain);

    // Computes the rotation needed for each join such that it gives the closest distance
    //between the target and the end-effector
    // 
    // For me: This function uses the Cyclic Coordinate Descent algorithm. 
    // Joints are rotated one at a time starting from the last joint, such that the distance
    // between the end effector and the target eventually reaches 0. 
    // 
    // In other words, for each joint
    // starting from the last, it asks "how much should I rotate this joint such that it gives the closest distance
    // between the end effector and the target?" It finds the answer for this question at every joint by "max_passes" times
    // Until the end-effector eventually meets the target
    float SolveCCD(ArmChain& chain, Vec2 const target, int max_passes, float tolerance);
}