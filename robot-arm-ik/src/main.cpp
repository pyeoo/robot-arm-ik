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
#include "arm/ArmChain3D.h"
#include "arm/Kinematics.h"
#include "arm/Kinematics3D.h"
#include "arm/Renderer.h"
#include "arm/Renderer3D.h"
#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

std::vector<bool> DrawJointSliders3D(ArmChain3D& chain, float PI) {
    std::vector<bool> manuallyChanged(chain.JointCount(), false);
    for (size_t i = 0; i < chain.JointCount(); i++) {
        Joint3D const& joint = chain.GetJoint(i);
        float angle = joint.angle;
        float min_deg = joint.min_angle * (180 / PI);
        float max_deg = joint.max_angle * (180 / PI);
        std::string label = "Joint " + std::to_string(i);
        if (ImGui::SliderAngle(label.c_str(), &angle, min_deg, max_deg)) {
            chain.SetJointAngle(i, angle);
            manuallyChanged[i] = true;
        }
    }
    return manuallyChanged;
}

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

    constexpr float PI = 3.14159265f;

    /* TEST 3D ROBOT ARM */
    std::vector<Joint3D> joints3d = {
    { 30.0f * PI / 180.0f, 120.0f, -PI, PI, Eigen::Vector3f(0, 0, 1) },  // base, yaw
    { 40.0f * PI / 180.0f, 100.0f, -PI, PI, Eigen::Vector3f(0, 1, 0) },  // shoulder, pitch around Y
    { -50.0f * PI / 180.0f, 70.0f, -PI, PI, Eigen::Vector3f(0, 0, 1) },  // elbow, back to Z: perpendicular to local +X, and distinct from joint 1's Y
    };
    ArmChain3D chain3d(joints3d, Eigen::Vector3f(0, 0, 0));
    ArmChain3D solver_chain3d(joints3d, Eigen::Vector3f(0, 0, 0));
    Eigen::Vector3f target3d(150.0f, -100.0f, 80.0f);

    /* RENDERER */
    Renderer renderer{};
    renderer.Init(WINDOW_WIDTH, WINDOW_HEIGHT);

    Renderer3D renderer3d{};
    renderer3d.Init(WINDOW_WIDTH, WINDOW_HEIGHT);

    glfwSetWindowUserPointer(window, &renderer);

    glfwSetFramebufferSizeCallback(window, [](GLFWwindow* win, int width, int height) {
        glViewport(0, 0, width, height);
        Renderer* r = static_cast<Renderer*>(glfwGetWindowUserPointer(win));
        r->OnResize(width, height);
        });

    // Test view matrix
    
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
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        /* DRAW GUI */
        // Start of frame 
        ImGui_ImplOpenGL3_NewFrame();   // Updates ImGui with render data
        ImGui_ImplGlfw_NewFrame();      // Updates ImGui with GLFW data (e.g. window size, mouse pos, mouse and key inputs)
        ImGui::NewFrame();

        // Build GUI Panel for 3D
        ImGui::Begin("3D Arm Control");
        static float easing_speed_3d = 8.0f;
        auto manuallyChanged3d = DrawJointSliders3D(chain3d, PI);
        ImGui::SliderFloat("Easing Speed 3D", &easing_speed_3d, 1.f, 100.f);
        ImGui::DragFloat3("Target 3D", target3d.data(), 1.0f);
        ImGui::End();

        /* DRAW GRID, ROBOT ARM AND TARGET */
        Kinematics3D::SolveCCD_3D(solver_chain3d, target3d, 20, 0.5f);

        for (size_t i = 0; i < chain3d.JointCount(); i++) {
            if (manuallyChanged3d[i]) continue;   // If joint was manually changed, don't conflict with the manual change this frame
            float target_angle = solver_chain3d.GetJoint(i).angle;
            float current_angle = chain3d.GetJoint(i).angle;
            float eased = Kinematics::EaseAngle(current_angle, target_angle, easing_speed_3d, dt);
            chain3d.SetJointAngle(i, eased);
        }

        Eigen::Matrix4f targetModel = Kinematics3D::BuildMarkerModelMatrix(target3d, 15.0f);  // target
        renderer3d.DrawCube(targetModel, 0.3f, 1.0f, 0.3f);  // same green you used for DrawPoint
        renderer3d.DrawArmBoxes(Kinematics3D::ComputeLinkTransforms(chain3d), 15.0f);
        
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