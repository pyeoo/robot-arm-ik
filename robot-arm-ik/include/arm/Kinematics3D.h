/*
*
* File name: Kinematics3D.h
*
* Computes the forward and inverse kinematics in 3D space
*
*/
#pragma once

#include <vector>
#include <Eigen/Dense>
#include <arm/ArmChain3D.h>

namespace Kinematics3D {
    // Now we are essentially using 3D cubes instead of lines. Thus, we need more information to define
    //a link using a cube. The idea behind this LinkTransform is that it encapsulates all necessary data of ONE link
    //and treats it as ONE so-called 'line'
    struct LinkTransform {
        Eigen::Vector3f position;       // the link's starting joint position
        Eigen::Quaternionf orientation;  // the link's direction, as rotation from local +X
        float length;
    };

    // Computes every joint position in world space from the chain's current angles.
    // Same index convention as the 2D version: index 0 = base, last index = end-effector.
    std::vector<Eigen::Vector3f> ComputeJointPositions(ArmChain3D const& chain);

    // Computes every joint's orientation in parent frame
    std::vector<Eigen::Quaternionf> ComputeJointOrientation(ArmChain3D const& chain);

    // Computes the actual transformation properties of all links
    std::vector<LinkTransform> ComputeLinkTransforms(ArmChain3D const& chain);

    // Compute the model transformation for each link
    Eigen::Matrix4f BuildLinkModelMatrix(LinkTransform const& lt, float thickenss);

    // Compute the model transformation for the target object
    Eigen::Matrix4f BuildMarkerModelMatrix(Eigen::Vector3f position, float size);

    // Project both vectors (end-effector to target and joint to target) onto a plane perpendicular to the axis first 
    // This helps us to easily compute the angle to turn betwen the two vectors about a given axis
    Eigen::Vector3f project_perpendicular(Eigen::Vector3f v, Eigen::Vector3f axis);

    // Comptues the rotation needed for each joint about an axis such that it gives the closest distance
    //betweeen the target and the end effector
    float SolveCCD_3D(ArmChain3D& chain, Eigen::Vector3f target, int max_passes, float tolerance);

    // Helper function to compute easing
    float EaseAngle(float current_angle, float target_angel, float ease_speed, float dt);
}