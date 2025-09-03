
#include "mujoco/mujoco.h"
#include <iostream>
#include <thread>
#include "GLFW/glfw3.h"

char error[1000];
mjModel* m;
mjData* d;

mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;
GLFWwindow* window = nullptr;

// Mouse state
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

// Mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    glfwGetCursorPos(window, &lastx, &lasty);
}

// Mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
    if (!button_left && !button_middle && !button_right) {
        return;
    }

    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    bool shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                  glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    mjtMouse action;
    if (button_right) {
        action = shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (button_left) {
        action = shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else {
        action = mjMOUSE_ZOOM;
    }

    mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

// Scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, 0.05 * yoffset, &scn, &cam);
}

// Initialize GLFW and MuJoCo visualization
bool init_visualization() {
    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return false;
    }

    // Create window
    window = glfwCreateWindow(1200, 900, "MuJoCo Simulation", NULL, NULL);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // Set callbacks
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetScrollCallback(window, scroll);

    // Initialize MuJoCo visualization
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // Create scene and context
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    return true;
}

void render() {
    // Get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // Update scene
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);

    // Render scene
    mjr_render(viewport, &scn, &con);

    // Swap buffers
    glfwSwapBuffers(window);

    // Process events
    glfwPollEvents();
}

void cleanup() {
    // Free visualization resources
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // Free model and data
    if (d) mj_deleteData(d);
    if (m) mj_deleteModel(m);

    // Cleanup GLFW
    if (window) glfwDestroyWindow(window);
    glfwTerminate();
}


int main(void) {
    // load model from file and check for errors
    m = mj_loadXML("boston_dynamics_spot/scene_arm.xml", NULL, error, 1000);
    if (!m) {
        std::cout<<error<<std::endl;
        return 1;
    }

    // make data corresponding to model
    d = mj_makeData(m);


    // Initialize visualization
    if (!init_visualization()) {
        cleanup();
        return 1;
    }


    // Main simulation loop
    while (!glfwWindowShouldClose(window)) {
        // Advance simulation
        mj_step(m, d);

        // Render
        render();

        // Control simulation speed
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Cleanup
    cleanup();
    return 0;
}
