/*
* 
* File name: main.cpp
* 
* Main entry point of the program. Requests OpenGL 3.3 core profile and initializes GLFW, window, GLAD.
* 
*/

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include "arm/ArmChain.h"
#include "arm/Kinematics.h"
#include "arm/Renderer.h"
#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

int main() {
    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Unable to initialize GLFW" << '\n';
        return -1;
    }

    float last_time = static_cast<float>(glfwGetTime());    // Get time

    constexpr int WINDOW_WIDTH = 1200;
    constexpr int WINDOW_HEIGHT = 800;

    // Request OpenGL 3.3 Core Profile
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Create a window
    GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Robot F/IK", nullptr, nullptr);
    if (!window) {
        std::cerr << "Unable to create window" << '\n'; 
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window); // Make this window the current context

    // Initialize GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Unabloe to initialize GLAD" << '\n';
        return -1;
    }

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    static_cast<void>(io);
    ImGui::StyleColorsDark();
    
    // Setup platform and renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true); // Connects imgui to GLFW window (so that imgui can read opengl inputs)
    ImGui_ImplOpenGL3_Init("#version 330"); // Initialize imgui's render backend for OpenGL

    /* TEST ROBOT ARM */
    // 3-joint arm: lengths 120, 100, 70. Limits wide open for now (-180 to 180 deg in radians).
    constexpr float PI = 3.14159265f;
    std::vector<Joint> joints = {
        { 30.0f * PI / 180.0f, 120.0f, -PI, PI },  
        { -45.0f * PI / 180.0f, 100.0f, -PI, PI }, 
        { 60.0f * PI / 180.0f,  70.0f, -PI, PI },  
    };

    ArmChain chain(joints, 0.0f, 0.0f);
    ArmChain solver_chain(joints, 0.f, 0.f);
    Vec2 target = { 150.0f, 100.0f };

    /* RENDERER */
    Renderer renderer{};
    renderer.Init(WINDOW_WIDTH, WINDOW_HEIGHT);

    glfwSetWindowUserPointer(window, &renderer);

    glfwSetFramebufferSizeCallback(window, [](GLFWwindow* win, int width, int height) {
        glViewport(0, 0, width, height);
        Renderer* r = static_cast<Renderer*>(glfwGetWindowUserPointer(win));
        r->OnResize(width, height);
        });

    // Main Loop
    while (!glfwWindowShouldClose(window)) {
        // Test input (to close window)
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
            glfwSetWindowShouldClose(window, true);
        }

        // Compute dt
        float current_time = static_cast<float>(glfwGetTime());
        float dt = current_time - last_time;
        last_time = current_time;

        // Render stuff
        glClearColor(0.15f, 0.15f, 0.2f, 1.f);
        glClear(GL_COLOR_BUFFER_BIT);

        /* DRAW GUI */
        // Start of frame 
        ImGui_ImplOpenGL3_NewFrame();   // Updates ImGui with render data
        ImGui_ImplGlfw_NewFrame();      // Updates ImGui with GLFW data (e.g. window size, mouse pos, mouse and key inputs)
        ImGui::NewFrame();

        // Build GUI Panel
        ImGui::Begin("Arm Control");
        
        // Sliders
        static float easing_speed = 8.0f; // Easing speed
        // Get each joint
        for (size_t i = 0; i < chain.JointCount(); i++) {
            Joint const& joint = chain.GetJoint(i);

            // Get angles
            float angle = joint.angle;      // Can stay radians, as ImGui::SliderAngle allows for radian input
            // ImGui::SliderAngle still uses degrees for min and max boundaries, so the min and max angle still need to be converted into degrees
            float min_angle_deg = joint.min_angle * (180 / PI);
            float max_angle_deg = joint.max_angle * (180 / PI);

            // Label
            std::string label = "Joint " + std::to_string(i);
            // Joint angle slider
            if (ImGui::SliderAngle(label.c_str(), &angle, min_angle_deg, max_angle_deg)) {
                chain.SetJointAngle(i, angle);
            }
        }

        // Easing speed slider
        std::string easing_speed_label = "Easing speed";
        ImGui::SliderFloat(easing_speed_label.c_str(), &easing_speed, 1.f, 100.f);

        ImGui::DragFloat2("Target", &target.x, 1.0f); // 1.0f = drag sensitivity
        ImGui::End();

        /* DRAW GRID ROBOT ARM AND TARGET */
        Kinematics::SolveCCD(solver_chain, target, 4, 2.0f);

        for (size_t i = 0; i < chain.JointCount(); i++) {
            float target_angle = solver_chain.GetJoint(i).angle;
            float current_angle = chain.GetJoint(i).angle;
            float eased = Kinematics::EaseAngle(current_angle, target_angle, easing_speed, dt);
            chain.SetJointAngle(i, eased);
        }

        std::vector<Vec2> positions = Kinematics::ComputeJointPositions(chain);
        renderer.DrawGrid(400.f, 40.f);
        renderer.DrawArm(positions);
        renderer.DrawPoint(target, 0.3f, 1.0f, 0.3f);

        /* END OF FRAME (Draw the GUI ON TOP of the arm */
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // Swap Buffer and Poll events
        glfwSwapBuffers(window);    // Swap front and back buffer
        glfwPollEvents();           // Process any input events
    }

    // Destrory imgui context
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwTerminate();
    return 0;
}