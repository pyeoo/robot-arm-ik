/*
*
* File name: Renderer.cpp
*
* Implements 3D Renderer
*
*/
#include <glad/glad.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <iostream>
#include "arm/Renderer3D.h"

constexpr float PI = 3.14159265f;

/* CUBE */
// 24 Vertices, 6 faces
// I decided to use 24 vertices instead of 8. This is because for the lighting equations later on, we need to use the normal
//of each face. If we were to use the traditional 8 vertices approach, certain edges are shared and the normals would becomes ambigious.
//We need to make sure we have ONE DISTINCT NORMAL PER FACE so that the lighting can act on the surface normal without any ambiguity.

static const float cubeVertices[] = {
	// +X face (normal: 1,0,0)
	1,-0.5f,-0.5f,  1,0,0,
	1, 0.5f,-0.5f,  1,0,0,
	1, 0.5f, 0.5f,  1,0,0,
	1,-0.5f, 0.5f,  1,0,0,

	// -X face (normal: -1,0,0)
	0,-0.5f, 0.5f,  -1,0,0,
	0, 0.5f, 0.5f,  -1,0,0,
	0, 0.5f,-0.5f,  -1,0,0,
	0,-0.5f,-0.5f,  -1,0,0,

	// +Y face (normal: 0,1,0)
	0, 0.5f,-0.5f,  0,1,0,
	0, 0.5f, 0.5f,  0,1,0,
	1, 0.5f, 0.5f,  0,1,0,
	1, 0.5f,-0.5f,  0,1,0,

	// -Y face (normal: 0,-1,0)
	0,-0.5f, 0.5f,  0,-1,0,
	0,-0.5f,-0.5f,  0,-1,0,
	1,-0.5f,-0.5f,  0,-1,0,
	1,-0.5f, 0.5f,  0,-1,0,

	// +Z face (normal: 0,0,1)
	0,-0.5f, 0.5f,  0,0,1,
	1,-0.5f, 0.5f,  0,0,1,
	1, 0.5f, 0.5f,  0,0,1,
	0, 0.5f, 0.5f,  0,0,1,

	// -Z face (normal: 0,0,-1)
	1,-0.5f,-0.5f,  0,0,-1,
	0,-0.5f,-0.5f,  0,0,-1,
	0, 0.5f,-0.5f,  0,0,-1,
	1, 0.5f,-0.5f,  0,0,-1,
};

// 6 faces * 2 triangles per face * 3 indices per triangle= 36 indices
static const unsigned int cubeIndices[] = {
	0,1,2,  0,2,3,       // +X
	4,5,6,  4,6,7,       // -X
	8,9,10, 8,10,11,     // +Y
	12,13,14, 12,14,15,  // -Y
	16,17,18, 16,18,19,  // +Z
	20,21,22, 20,22,23,  // -Z
};

void Renderer3D::InitCubeMesh() {
	glGenVertexArrays(1, &cube_vao);
	glBindVertexArray(cube_vao);

	glGenBuffers(1, &cube_vbo);
	glBindBuffer(GL_ARRAY_BUFFER, cube_vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(cubeVertices), cubeVertices, GL_STATIC_DRAW);

	glGenBuffers(1, &cube_ebo);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cube_ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(cubeIndices), cubeIndices, GL_STATIC_DRAW);

	// position: 3 floats, normal: 3 floats, stride = 6 floats total per vertex
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);
}

unsigned int Renderer3D::CompileShader(const char* source, unsigned int type) {
	unsigned int id = glCreateShader(type);
	glShaderSource(id, 1, &source, nullptr);
	glCompileShader(id);

	// Check compilation status and fetch logs if it fails
	int success;
	glGetShaderiv(id, GL_COMPILE_STATUS, &success);
	if (!success) {
		int length;
		glGetShaderiv(id, GL_INFO_LOG_LENGTH, &length);
		std::vector<char> message(length);
		glGetShaderInfoLog(id, length, &length, message.data());

		std::cerr << "Failed to compile 3D "
			<< (type == GL_VERTEX_SHADER ? "vertex" : "fragment")
			<< " shader!" << std::endl;
		std::cerr << message.data() << std::endl;

		glDeleteShader(id);
		return 0;
	}

	return id;
}

unsigned int Renderer3D::LinkProgram(unsigned int vertShader, unsigned int fragShader) {
	if (vertShader == 0 || fragShader == 0) {
		std::cerr << "Shader linking aborted: Invalid shader IDs provided." << std::endl;
		return 0;
	}

	unsigned int program = glCreateProgram();
	glAttachShader(program, vertShader);
	glAttachShader(program, fragShader);
	glLinkProgram(program);

	// Check program linking status and fetch logs if it fails
	int success;
	glGetProgramiv(program, GL_LINK_STATUS, &success);
	if (!success) {
		int length;
		glGetProgramiv(program, GL_INFO_LOG_LENGTH, &length);
		std::vector<char> message(length);
		glGetProgramInfoLog(program, length, &length, message.data());

		std::cerr << "Failed to link shader program!" << std::endl;
		std::cerr << message.data() << std::endl;

		glDeleteProgram(program);
		return 0;
	}

	return program;
}

std::string Renderer3D::LoadFile(const char* path) {
	// Get and open input file stream 
	std::ifstream file(path);
	if (!file.is_open()) {
		std::cerr << "Unable to open file: " << path << '\n';
		return "";
	}
	// Read and return as a string stream
	std::stringstream stream;
	stream << file.rdbuf();
	return stream.str();
}


Eigen::Matrix4f Renderer3D::BuildViewMatrix(Eigen::Vector3f eye, Eigen::Vector3f tgt, Eigen::Vector3f worldUp) {
	Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
	Eigen::Vector3f w = (tgt - eye).normalized();
	Eigen::Vector3f u = w.cross(worldUp).normalized();
	Eigen::Vector3f v = u.cross(w);

	view(0, 0) = u.x(); view(0, 1) = u.y(); view(0, 2) = u.z(); view(0, 3) = -u.dot(eye);
	view(1, 0) = v.x(); view(1, 1) = v.y(); view(1, 2) = v.z(); view(1, 3) = -v.dot(eye);
	view(2, 0) = -w.x(); view(2, 1) = -w.y(); view(2, 2) = -w.z(); view(2, 3) = w.dot(eye);

	return view;
}

Eigen::Matrix4f Renderer3D::BuildPerspectiveMatrix(float FOV_Y, float ar, float z_near, float z_far) {
	Eigen::Matrix4f proj = Eigen::Matrix4f::Zero();
	float f = 1 / std::tan(FOV_Y / 2);

	proj(0, 0) = f / ar;
	proj(1, 1) = f;
	proj(2, 2) = (z_far + z_near) / (z_near - z_far);
	proj(2, 3) = (2 * z_near * z_far) / (z_near - z_far);
	proj(3, 2) = -1;

	return proj;
}

void Renderer3D::Init(int window_width, int window_height) {
	// Load and compile shaders
	std::string vs_str = LoadFile("assets/shaders/basic3d.vert");
	std::string fs_str = LoadFile("assets/shaders/basic3d.frag");
	unsigned int vertShader = CompileShader(vs_str.c_str(), GL_VERTEX_SHADER);
	unsigned int fragShader = CompileShader(fs_str.c_str(), GL_FRAGMENT_SHADER);
	shaderProgram = LinkProgram(vertShader, fragShader);

	// Setup vao and vbo
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);
	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Eigen::Vector3f), (void*)0);
	glEnableVertexAttribArray(0);

	InitCubeMesh();

	float ar = static_cast<float>(window_width) / static_cast<float>(window_height);
	view_ = BuildViewMatrix(Eigen::Vector3f(300, 300, 300), Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 1, 0));	// Build view matrix (static camera for now)
	projection_ = BuildPerspectiveMatrix(45.0f * (PI / 180.0f), ar, 0.1f, 1000.f);	// Build projection matrix

	
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_PROGRAM_POINT_SIZE);
}

void Renderer3D::DrawArm(std::vector<Eigen::Vector3f> const& jointPositions) {
	glBindVertexArray(vao);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, jointPositions.size() * sizeof(Eigen::Vector3f), jointPositions.data(), GL_DYNAMIC_DRAW);

	glUseProgram(shaderProgram);
	Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();
	glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uModel"), 1, GL_FALSE, identity.data());
	glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uProjection"), 1, GL_FALSE, projection_.data());
	glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uView"), 1, GL_FALSE, view_.data());

	glUniform3f(glGetUniformLocation(shaderProgram, "uColor"), 0.7f, 0.7f, 0.7f);
	glDrawArrays(GL_LINE_STRIP, 0, static_cast<GLint>(jointPositions.size()));

	glUniform3f(glGetUniformLocation(shaderProgram, "uColor"), 1.0f, 0.3f, 0.3f);
	glDrawArrays(GL_POINTS, 0, static_cast<GLint>(jointPositions.size()));

	glUniform3f(glGetUniformLocation(shaderProgram, "uFogColor"), 0.15f, 0.15f, 0.2f);   // match your glClearColor
	glUniform1f(glGetUniformLocation(shaderProgram, "uFogNear"), 300.0f);
	glUniform1f(glGetUniformLocation(shaderProgram, "uFogFar"), 700.0f);
}

void Renderer3D::DrawCube(Eigen::Matrix4f const& model, float r, float g, float b) {
	glBindVertexArray(cube_vao);
	glUseProgram(shaderProgram);
	glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uProjection"), 1, GL_FALSE, projection_.data());
	glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uView"), 1, GL_FALSE, view_.data());
	glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uModel"), 1, GL_FALSE, model.data());
	glUniform3f(glGetUniformLocation(shaderProgram, "uColor"), r, g, b);
	glUniform3f(glGetUniformLocation(shaderProgram, "uLightDir"), 0.5f, 1.0f, 0.3f);   // NEW
	glUniform3f(glGetUniformLocation(shaderProgram, "uFogColor"), 0.15f, 0.15f, 0.2f); // also missing here, same reasoning as uLightDir
	glUniform1f(glGetUniformLocation(shaderProgram, "uFogNear"), 300.0f);
	glUniform1f(glGetUniformLocation(shaderProgram, "uFogFar"), 700.0f);
	glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
}

void Renderer3D::DrawArmBoxes(std::vector<Kinematics3D::LinkTransform> const& transforms, float thickness) {
	for (auto const& lt : transforms) {
		Eigen::Matrix4f model = BuildLinkModelMatrix(lt, thickness);
		DrawCube(model, 0.7f, 0.7f, 0.7f);
	}
}

void Renderer3D::DrawPoint(Eigen::Vector3f position, float r, float g, float b) {
	glBindVertexArray(vao);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Eigen::Vector3f), &position, GL_DYNAMIC_DRAW);

	glUseProgram(shaderProgram);
	Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();
	glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uModel"), 1, GL_FALSE, identity.data());
	glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uProjection"), 1, GL_FALSE, projection_.data());
	glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uView"), 1, GL_FALSE, view_.data());
	glUniform3f(glGetUniformLocation(shaderProgram, "uColor"), r, g, b);
	glDrawArrays(GL_POINTS, 0, 1);
}

