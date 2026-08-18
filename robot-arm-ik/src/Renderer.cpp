/*
*
* File name: Renderer.cpp
*
* Implements functions of Renderer.h
*
*/
#include <glad/glad.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <iostream>
#include "arm/Renderer.h"

unsigned int Renderer::CompileShader(const char* source, unsigned int type) {
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

        std::cerr << "Failed to compile "
            << (type == GL_VERTEX_SHADER ? "vertex" : "fragment")
            << " shader!" << std::endl;
        std::cerr << message.data() << std::endl;

        glDeleteShader(id);
        return 0;
    }

    return id;
}

unsigned int Renderer::LinkProgram(unsigned int vertShader, unsigned int fragShader) {
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

std::string Renderer::LoadFile(const char* path) {
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

void Renderer::RebuildProjection(int width, int height) {
    // Construct orthographic projection matrix
    Eigen::Matrix4f ortho = Eigen::Matrix4f::Zero();
    float ar = static_cast<float>(width) / static_cast<float>(height);
    float half_extent = 350.f;
    float left, right, bottom, top;
    float zNear = -1.0f;
    float zFar = 1.0f;

    if (ar >= 1.0f) {
        // w > h: expand left/right
        top = half_extent;
        bottom = -half_extent;
        right = half_extent * ar;
        left = -right;
    }
    else {
        // h > w: expand top/bottom
        right = half_extent;
        left = -half_extent;
        top = half_extent / ar;
        bottom = -top;
    }

    ortho(0, 0) = 2 / (right - left);
    ortho(1, 1) = 2 / (top - bottom);
    ortho(2, 2) = -2 / (zFar - zNear);
    ortho(0, 3) = -(right + left) / (right - left);
    ortho(1, 3) = -(top + bottom) / (top - bottom);
    ortho(2, 3) = -(zNear + zFar) / (zNear - zFar);
    ortho(3, 3) = 1.f;
    projection = ortho;
}

void Renderer::Init(int window_width, int window_height) {
    // Load and compile shaders
    std::string vs_str = LoadFile("assets/shaders/vs.vert");
    std::string fs_str = LoadFile("assets/shaders/fs.frag");
    unsigned int vertShader = CompileShader(vs_str.c_str(), GL_VERTEX_SHADER);
    unsigned int fragShader = CompileShader(fs_str.c_str(), GL_FRAGMENT_SHADER);
    shaderProgram = LinkProgram(vertShader, fragShader);

    // Setup vao/vbo
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);
    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Vec2), (void*)0);
    glEnableVertexAttribArray(0);
        
    RebuildProjection(window_width, window_height);
    glEnable(GL_PROGRAM_POINT_SIZE);
}

void Renderer::OnResize(int width, int height) {
    RebuildProjection(width, height);
}

void Renderer::DrawArm(std::vector<Vec2> const& jointPositions) {
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, jointPositions.size() * sizeof(Vec2), jointPositions.data(), GL_DYNAMIC_DRAW);

    glUseProgram(shaderProgram);

    // Upload projection matrix uniform
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uProjection"), 1, GL_FALSE, projection.data());

    // A small fixed palette, cycles if you ever have more links than colors
    static const float linkColors[][3] = {
        {0.37f, 0.78f, 0.85f},  // cyan
        {0.50f, 0.85f, 0.56f},  // green
        {1.00f, 0.71f, 0.33f},  // amber
    };
    int colorLoc = glGetUniformLocation(shaderProgram, "uColor");

    // one segment = vertices [i, i+1]; draw as GL_LINES with a 2-vertex slice
    for (size_t i = 0; i + 1 < jointPositions.size(); ++i) {
        const float* c = linkColors[i % 3];
        glUniform3f(colorLoc, c[0], c[1], c[2]);
        glDrawArrays(GL_LINES, static_cast<GLint>(i), 2);
    }
    // Draw Base
    glUniform3f(colorLoc, 0.6f, 0.6f, 0.6f); // gray base
    glDrawArrays(GL_POINTS, 0, 1);

    // Draw joints
    glUniform3f(glGetUniformLocation(shaderProgram, "uColor"), 1.f, 0.3f, 0.3f);
    glDrawArrays(GL_POINTS, 1, static_cast<GLsizei>(jointPositions.size() - 1));
}

void Renderer::DrawPoint(Vec2 position, float r, float g, float b) {
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vec2), &position, GL_DYNAMIC_DRAW);

    glUseProgram(shaderProgram);
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uProjection"), 1, GL_FALSE, projection.data());
    glUniform3f(glGetUniformLocation(shaderProgram, "uColor"), r, g, b);
    glDrawArrays(GL_POINTS, 0, 1);
}
void Renderer::DrawGrid(float halfExtent, float spacing) {
    std::vector<Vec2> gridVerts;
    for (float x = -halfExtent; x <= halfExtent; x += spacing) {
        gridVerts.push_back({ x, -halfExtent });
        gridVerts.push_back({ x,  halfExtent });
    }
    for (float y = -halfExtent; y <= halfExtent; y += spacing) {
        gridVerts.push_back({ -halfExtent, y });
        gridVerts.push_back({ halfExtent, y });
    }
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, gridVerts.size() * sizeof(Vec2), gridVerts.data(), GL_DYNAMIC_DRAW);

    glUseProgram(shaderProgram);
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "uProjection"), 1, GL_FALSE, projection.data());
    glUniform3f(glGetUniformLocation(shaderProgram, "uColor"), 0.25f, 0.28f, 0.27f); // faint, near-background

    glDrawArrays(GL_LINES, 0, static_cast<GLint>(gridVerts.size()));
}