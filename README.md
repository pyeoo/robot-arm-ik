# Robot F/IK: 2D & 3D Robotic Arm Kinematics Simulator

A real-time simulator of an articulated robotic arm, built to explore forward and
inverse kinematics from first principles, first in 2D, then extended into a fully
3D solid-body simulation with lighting. Drag a target and the arm solves, joint by
joint, how to reach it, smoothly easing into position while respecting real joint
angle limits.

## What it does

The project ships two parallel simulators sharing the same core ideas:

- **2D arm**: three joints, forward and inverse kinematics, live manual control via
  sliders, and a target-seeking mode that continuously re-solves and eases toward
  a draggable target.
- **3D arm**: the same forward/inverse kinematics problem extended into full 3D,
  with per-joint hinge axes, quaternion-based rotation, and the arm and target
  rendered as solid, lit boxes rather than lines, with real depth and camera
  perspective.

Both arms support two complementary modes of control over the same underlying
state:

- **Forward kinematics (manual mode)**: drag any joint's angle slider directly and
  watch the rest of the arm follow.
- **Inverse kinematics (target-seeking mode)**: drag the target and the arm
  automatically solves for the joint angles needed to reach it, re-solving every
  frame so the motion reads as a natural, springy reach rather than an instant
  snap.

## How the inverse kinematics works

Both arms use **Cyclic Coordinate Descent (CCD)**, an iterative method also used
in real robotics and character animation rigs. Rather than solving for every
joint's angle at once, it processes one joint per step, working backward from the
end-effector toward the base:

1. From the current joint, measure the direction to the end-effector and the
   direction to the target.
2. Rotate that joint by the angle between those two directions, closing the gap.
3. Move to the next joint back toward the base and repeat.
4. Run several full passes over all joints; the end-effector converges toward the
   target across passes rather than jumping there in one step.

**In 3D**, this needed a real extension rather than a direct port: each joint can
only rotate about one fixed axis (a hinge joint), so before computing the angle to
turn, both direction vectors are projected onto the plane perpendicular to that
joint's world-space axis, and the signed angle around that specific axis is
recovered using a cross-product/dot-product formulation, since a plain
vector-angle calculation has no concept of a specific rotation axis and can't
produce a signed, axis-relative result on its own.

## Smooth motion via decoupled easing

Rather than rendering CCD's raw, partial per-frame convergence directly (which
looked mechanical), the displayed arm and the IK solver are two separate pieces of
state: a solver chain re-converges toward the target almost instantly every frame,
acting as a stable goal pose, while the displayed chain eases its joint angles
toward that goal using frame-rate-independent exponential smoothing. Manual slider
input bypasses easing entirely so direct control stays responsive, while
target-seeking motion reads as a natural, decelerating reach.

## The 3D extension

Going from 2D to 3D wasn't a straightforward port, three ideas from the 2D
solution had to be rebuilt around genuinely different math:

- **Rotations are composed with quaternions, not angle addition.** 2D rotations
  commute (order doesn't matter), so cumulative rotation was just adding angles.
  3D rotations don't commute, so each joint's rotation is built with
  `Eigen::AngleAxisf` from its own fixed axis and angle, then composed onto the
  chain's accumulated orientation via quaternion multiplication.
- **Every joint is a hinge with an explicit axis.** Choosing that axis matters: an
  axis parallel to the joint's own link direction produces a joint that only rolls
  in place with no bending at all, and axes shared between two consecutive joints
  confine the whole sub-chain to a single bending plane. Getting genuinely
  non-planar 3D motion required each joint's axis to be distinct from both its own
  link direction and its neighbor's effective axis.
- **The renderer needed real depth, not just a 3D-looking projection.** A
  perspective camera with a proper view and projection matrix was necessary but
  not sufficient, a bare wireframe line still reads as ambiguous depth-wise
  without additional cues. Distance-based fog and, later, actual solid box
  geometry (24 vertices per cube with one normal per face, lit with a simple
  diffuse model) were what actually made the arm look three-dimensional rather
  than just mathematically-technically-3D.

## Architecture

The project is split into layers that don't know more about each other than they
strictly need to, this split was deliberately mirrored between the 2D and 3D
implementations rather than trying to unify them under one generic system:

- **`ArmChain` / `ArmChain3D`**, pure data. Link lengths, current joint angles,
  joint limits, and (in 3D) each joint's hinge axis. All mutation goes through
  bounds-checked, clamped setters, so invalid state can't be written from anywhere
  else in the codebase.
- **`Kinematics` / `Kinematics3D`**, pure math. Forward kinematics derives joint
  positions from a chain's current angles; `SolveCCD` iteratively adjusts a
  chain's angles to reach a target. Neither touches OpenGL or holds state, both
  are plain, testable transformations of data.
- **`Renderer` / `Renderer3D`**, pure presentation. Draws whatever it's handed,
  points and lines in 2D, lit solid boxes in 3D, with no awareness of kinematics,
  chains, or joints.

Positions are never cached or stored; they're recomputed fresh from each chain's
current angles every frame. This was a deliberate choice from the very first
version of the project: it trades a small amount of redundant computation for
eliminating an entire category of "stale position" bugs where rendered state could
drift out of sync with the actual joint angles, and it's exactly what makes
per-frame easing and re-solving work correctly with no extra bookkeeping.

## Tech stack

- **C++17**
- **OpenGL 3.3 (Core Profile)** via GLFW (windowing/input) and GLAD (function
  loading)
- **Eigen** for all 3D linear algebra: quaternions, view/projection matrices, and
  the per-link model matrices used to position and orient each box
- **Dear ImGui** for live joint-angle sliders and target input, separate panels for
  the 2D and 3D arms
- Custom GLSL shaders: a 2D orthographic pipeline for the original arm, and a
  separate 3D perspective pipeline with per-vertex normals, distance-based fog,
  and basic diffuse lighting for the box-rendered arm

## What I'd extend next

- **Camera controls.** The 3D scene currently uses a fixed camera position; adding
  mouse-drag orbit controls would let a viewer see the arm's depth and rotation
  interactively rather than from one fixed angle.
- **Ball joints.** Every joint in the current 3D arm is a single-axis hinge; a
  shoulder-like joint with two or three degrees of freedom would need a genuinely
  different representation (a full quaternion per joint rather than a scalar angle
  plus fixed axis) and would be the point where slerp-based easing actually
  becomes necessary, rather than the simple scalar easing that's sufficient for
  hinge joints.
- **Index-buffer sharing between links**, to reduce redundant vertex uploads if
  the arm grows to many more joints.
- **Obstacle/floor constraints**, so the arm treats certain regions as off-limits
  in addition to its per-joint angle limits.
