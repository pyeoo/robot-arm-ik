/*
*
* File name: Renderer.h
*
* Initializes the shader program, vao and vbo and draws the robot arm
*
*/
#pragma once
#include <vector>
#include <Eigen/Dense>
#include "arm/Kinematics.h"

class Renderer {
public:
    void Init(int window_width, int window_height);
    void OnResize(int width, int height);
    void ShutDown();

    void DrawArm(std::vector<Vec2> const& jointPositions);
    void DrawPoint(Vec2 position, float r, float g, float b);
    void DrawGrid(float halfExtent, float spacing);

private:
    unsigned int shaderProgram = 0;
    unsigned int vao = 0;
    unsigned int vbo = 0;
    Eigen::Matrix4f projection;

    unsigned int CompileShader(const char* source, unsigned int type);
    unsigned int LinkProgram(unsigned int vertShader, unsigned int fragShader);
    std::string LoadFile(const char* path);

    void RebuildProjection(int width, int height);
};