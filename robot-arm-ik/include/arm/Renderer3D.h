/*
*
* File name: Renderer3D.h
*
* Initializes the shader program, vao and vbo and draws the robot arm in 3D space
*
*/
#pragma once
#include <vector>
#include <Eigen/Dense>
#include <arm/Kinematics3D.h>

class Renderer3D {
public:
	void Init(int window_width, int window_height);
	void OnResize(int width, int height);

	void DrawArm(std::vector<Eigen::Vector3f> const& jointPositions);
	void DrawPoint(Eigen::Vector3f position, float r, float g, float b);
	void DrawCube(Eigen::Matrix4f const& model, float r, float g, float b);
	void DrawArmBoxes(std::vector<Kinematics3D::LinkTransform> const& link_transforms, float thickness);
private:
	
	Eigen::Matrix4f view_;
	Eigen::Matrix4f projection_;
	Eigen::Matrix4f BuildViewMatrix(Eigen::Vector3f eye, Eigen::Vector3f tgt, Eigen::Vector3f worldUp);
	Eigen::Matrix4f BuildPerspectiveMatrix(float FOV, float ar, float z_near, float z_far);

	void InitCubeMesh();

	unsigned int shaderProgram = 0;
	unsigned int vao = 0, vbo = 0;
	unsigned int cube_vao = 0, cube_vbo = 0, cube_ebo = 0;
	unsigned int CompileShader(const char* source, unsigned int type);
	unsigned int LinkProgram(unsigned int vertShader, unsigned int fragShader);
	std::string LoadFile(const char* path);

	void RebuildProjection(int width, int height);
};