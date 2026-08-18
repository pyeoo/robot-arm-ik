# Robot F/IK — 2D Robotic Arm Kinematics Simulator

A real-time simulator of a 3-joint robotic arm, built to explore forward and inverse
kinematics from first principles. Drag a target on screen and the arm solves, joint
by joint, how to reach it, while respecting real joint angle limits.

## What it does

The simulator supports two complementary modes of control over the same arm:

- **Forward kinematics (manual mode):** drag any joint's angle slider directly and
  watch the rest of the arm follow, a direct demonstration of how joint angles
  compose into an end-effector position.
- **Inverse kinematics (target-seeking mode):** drag the target point and the arm
  automatically solves for the joint angles needed to reach it, re-solving
  continuously every frame so the motion reads as a natural, springy reach rather
  than an instant snap.

Both modes run simultaneously on the same underlying arm state, there's no mode
switch, the IK solver and the manual sliders are just two different ways of setting
the same joint angles.

## How the inverse kinematics works

The solver uses **Cyclic Coordinate Descent (CCD)**, an iterative method also used
in real robotics and character animation rigs. Rather than solving for every joint's
angle at once, it processes one joint per step, working backward from the
end-effector toward the base:

1. From the current joint, measure the direction to the end-effector and the
   direction to the target.
2. Rotate that joint by the angle between those two directions, closing the gap.
3. Move to the next joint back toward the base and repeat.
4. Run several full passes over all joints; the end-effector converges toward the
   target across passes rather than jumping there in one step.

Because the solver is called once per frame rather than run to full convergence in
a single call, a target that requires many corrective passes visibly animates
toward the arm over a few frames, which is what produces the reaching motion you
see rather than an instant teleport.

Joint angle limits are enforced throughout, if a target would require a joint to
rotate past its limit, that joint clamps at the boundary and the arm reaches as
close as its geometry allows rather than bending unnaturally.

## Architecture

The project is split into layers that don't know about each other beyond what they
strictly need:

- **`ArmChain`** — pure data. Stores link lengths, current joint angles, and joint
  limits. Exposes only clamped, bounds-checked mutation (`SetJointAngle`,
  `RotateJoint`), so invalid state can't be written into it from anywhere else in
  the codebase.
- **`Kinematics`** — pure math. `ComputeJointPositions` derives every joint's world
  position from an `ArmChain`'s current angles (forward kinematics); `SolveCCD`
  iteratively adjusts an `ArmChain`'s angles to reach a target (inverse
  kinematics). Neither function touches OpenGL or holds any state itself, both are
  plain, testable transformations of data.
- **`Renderer`** — pure presentation. Draws whatever positions it's given as
  colored line segments and points, with no awareness of kinematics, chains, or
  joints. It only ever consumes the output of `Kinematics`.

Positions are never cached or stored, they're recomputed fresh from the chain's
current angles every single frame. This was a deliberate choice from the start:
it trades a small, irrelevant amount of redundant computation for eliminating an
entire category of "stale position" bugs where rendered state could drift out of
sync with the actual joint angles.

## Tech stack

- **C++17**
- **OpenGL 3.3 (Core Profile)** via GLFW (windowing/input) and GLAD (function loading)
- **Eigen** for the projection matrix (2D kinematics itself only needed scalar
  trigonometry, not full linear algebra, Eigen is used specifically where an
  actual 4×4 matrix is involved)
- **Dear ImGui** for live joint-angle sliders and target input
- Custom GLSL shaders for rendering, with per-link coloring and a background grid
  for spatial reference

## What I'd extend next

- **3D arm and rotation matrices/quaternions.** The current 2D implementation
  composes rotations by simple angle addition, which doesn't extend to 3D, a 3D
  version would be the natural next step and the point where Eigen's rotation
  types would actually be needed for the kinematics itself, not just the
  projection matrix.
- **Jacobian transpose IK**, as a comparison against CCD's convergence behavior
  and a natural use case for Eigen's matrix operations beyond the projection matrix.
- **Obstacle/floor constraints**, so the arm treats certain regions as off-limits
  in addition to its per-joint angle limits.
