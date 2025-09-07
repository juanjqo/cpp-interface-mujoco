/**
(C) Copyright 2011-2025 DQ Robotics Developers

This file is based on DQ Robotics.

    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.

DQ Robotics website: dqrobotics.github.io

Contributors:
- Juan Jose Quiroz Omana
       - Responsible for the original implementation.

*/

#include <dqrobotics/interfaces/mujoco/DQ_MujocoInterface.h>
#include <thread>
#include "mujoco/mujoco.h"
#include "GLFW/glfw3.h"


[[maybe_unused]] static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}


class DQ_MujocoInterface::Impl
{
public:
    //std::thread render_thread_;
    //std::atomic<bool> render_loop_started_{false};
    VectorXd qpos_;
    VectorXd qvel_;
public:
    std::string xml_model_;
    mjModel* m_ = nullptr;  // MuJoCo model
    mjData* d_ = nullptr;   // MuJoCo data

    mjvCamera cam_;                      // abstract camera
    mjvOption opt_;                      // visualization options
    mjvScene scn_;                       // abstract scene
    mjrContext con_;                     // custom GPU context

    GLFWwindow* window_ = nullptr;

    Impl()
    {
        std::cerr<<"DQ_MujocoInterface::Impl ctor"<<std::endl;
        //render_loop_started_ = false;

    };
    ~Impl(){
        std::cerr<<"DQ_MujocoInterface::Impl destroyed"<<std::endl;
        // close GLFW, free visualization storage
        // Cleanup GLFW
        if (window_) glfwDestroyWindow(window_);
        glfwTerminate();
        mjv_freeScene(&scn_);
        mjr_freeContext(&con_);

        //free MuJoCo model and data
        if (d_) mj_deleteData(d_);
        if (m_) mj_deleteModel(m_);

    }
    void load_model(const std::string& xml_model)
    {
        if (xml_model.empty())
            throw std::invalid_argument("xml model path cannot be empty");
        xml_model_ = xml_model;
        // load model from file and check for errors
        char error[1000];
        m_ = mj_loadXML(xml_model.c_str(), NULL, error, 1000);
        if (!m_)
            throw std::runtime_error("Wrong xml model!");

         // make data corresponding to model
        d_ = mj_makeData(m_);
        qpos_ = Eigen::Map<Eigen::VectorXd>(d_->qpos, m_->nq);
        qvel_ = Eigen::Map<Eigen::VectorXd>(d_->qvel, m_->nv);
        initialize(1280, 920, "test");
    }

    void initialize(const int &width, const int &height, const std::string& title)
    {
        /*
        glfwSetErrorCallback(glfw_error_callback);
        if (!glfwInit())
            throw std::runtime_error("Error in glfwInit()");

// Decide GL+GLSL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
        // GL ES 2.0 + GLSL 100
        const char* glsl_version = "#version 100";
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
        glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(__APPLE__)
        // GL 3.2 + GLSL 150
        const char* glsl_version = "#version 150";
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
#else
        // GL 3.0 + GLSL 130
        const char* glsl_version = "#version 130";
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
#endif
*/
        if (!glfwInit())
            throw std::runtime_error("Error in glfwInit()");
        window_ =  glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
        if (window_ == nullptr)
            throw std::runtime_error("Error in window()");

        glfwMakeContextCurrent(window_);
        glfwSwapInterval(1); // Enable vsync

        // initialize visualization data structures
        mjv_defaultCamera(&cam_);
        mjv_defaultOption(&opt_);
        mjv_defaultScene(&scn_);
        mjr_defaultContext(&con_);

        // Create scene and context
        mjv_makeScene(m_, &scn_, 2000);
        mjr_makeContext(m_, &con_, mjFONTSCALE_150);
    }


    void render()
    {
        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(m_, d_, &opt_, NULL, &cam_, mjCAT_ALL, &scn_);
        mjr_render(viewport, &scn_, &con_);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window_);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
    }

    void step_simulation()
    {
        if (!m_ || !d_)
            throw std::runtime_error("Model or data not initialized");
        mj_step(m_, d_);
        d_->qpos[0] = 0.5 * sin(2.0 * d_->time);
        qpos_ = Eigen::Map<Eigen::VectorXd>(d_->qpos, m_->nq);
        qvel_ = Eigen::Map<Eigen::VectorXd>(d_->qvel, m_->nv);
    }


};

DQ_MujocoInterface::DQ_MujocoInterface()
{
    impl_ = std::make_shared<DQ_MujocoInterface::Impl>();

}



void DQ_MujocoInterface::load_model(const std::string &xml_model)
{
    impl_->load_model(xml_model);
}



void DQ_MujocoInterface::trigger_next_simulation_step() const
{
    impl_->step_simulation();
}

double DQ_MujocoInterface::get_simulation_time() const
{
    return impl_->d_->time;
}

bool DQ_MujocoInterface::should_close_window() const
{
    return glfwWindowShouldClose(impl_->window_);
}

void DQ_MujocoInterface::render()
{
    impl_->render();
}

VectorXd DQ_MujocoInterface::get_joint_positions() const
{
    return impl_->qpos_;
}
