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

#include <chrono>
#include <dqrobotics/interfaces/mujoco/DQ_MujocoInterface.h>
//#include <thread>
#include "mujoco/mujoco.h"
#include "GLFW/glfw3.h"


[[maybe_unused]] static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}


class DQ_MujocoInterface::Impl
{
public:
    // Static callback wrappers
    static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
        Impl* self = static_cast<Impl*>(glfwGetWindowUserPointer(window));
        if (self) {
            self->mouse_button(window, button, action, mods);
        }
    }

    static void mouse_move_callback(GLFWwindow* window, double xpos, double ypos) {
        Impl* self = static_cast<Impl*>(glfwGetWindowUserPointer(window));
        if (self) {
            self->mouse_move(window, xpos, ypos);
        }
    }

    static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
        Impl* self = static_cast<Impl*>(glfwGetWindowUserPointer(window));
        if (self) {
            self->scroll(window, yoffset);
        }
    }

    // Mouse button handler
    void mouse_button(GLFWwindow* window, int button, int action, int mods) {
        button_left_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
        button_middle_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
        button_right_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
        glfwGetCursorPos(window, &lastx_, &lasty_);
    }

    // Mouse move handler
    void mouse_move(GLFWwindow* window, double xpos, double ypos) {
        if (!button_left_ && !button_middle_ && !button_right_)
            return;
        double dx = xpos - lastx_;
        double dy = ypos - lasty_;
        lastx_ = xpos;
        lasty_ = ypos;

        int width, height;
        glfwGetWindowSize(window, &width, &height);

        bool shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

        mjtMouse action;
        if (button_right_) {
            action = shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
        } else if (button_left_) {
            action = shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
        } else {
            action = mjMOUSE_ZOOM;
        }
        mjv_moveCamera(m_, action, dx / height, dy / height, &scn_, &cam_);
    }

    // Scroll handler
    void scroll(GLFWwindow* window, double yoffset) {
        mjv_moveCamera(m_, mjMOUSE_ZOOM, 0, 0.05 * yoffset, &scn_, &cam_);
    }

public:
    //std::thread render_thread_;
    //std::atomic<bool> render_loop_started_{false};
    VectorXd qpos_;
    VectorXd qvel_;
    std::unordered_map<std::string, int> handles_map_; //

public:
    std::string xml_model_;
    mjModel* m_ = nullptr;  // MuJoCo model
    mjData* d_ = nullptr;   // MuJoCo data

    mjvCamera cam_;                      // abstract camera
    mjvOption opt_;                      // visualization options
    mjvScene scn_;                       // abstract scene
    mjrContext con_;                     // custom GPU context

    GLFWwindow* window_ = nullptr;

    // Mouse state
    bool button_left_ = false;
    bool button_middle_ = false;
    bool button_right_ = false;
    double lastx_{0};
    double lasty_{0};

    bool show_profiler_ = true;  // Control flag for showing profiler
    int frame_count_{0};
    double fps_{0};
    std::chrono::high_resolution_clock::time_point last_time_;
    double computation_time_{0};
    //mjrRect profiler_rect_;

    /**
     * @brief Impl
     */
    Impl()
    {
        std::cerr<<"DQ_MujocoInterface::Impl ctor"<<std::endl;
        //render_loop_started_ = false;

    };

    /**
     * @brief ~Impl() Destructor of the class
     */
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

    /**
     * @brief load_model
     * @param xml_model
     */
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

    /**
     * @brief initialize
     * @param width
     * @param height
     * @param title
     */
    void initialize(const int &width, const int &height, const std::string& title)
    {
        if (!glfwInit())
            throw std::runtime_error("Error in glfwInit()");
        window_ =  glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
        if (window_ == nullptr)
            throw std::runtime_error("Error in window()");

        // Store this pointer for callbacks
        glfwSetWindowUserPointer(window_, this);

        glfwMakeContextCurrent(window_);
        glfwSwapInterval(1); // Enable vsync

        // Set callbacks using static wrapper functions
        glfwSetMouseButtonCallback(window_, &Impl::mouse_button_callback);
        glfwSetCursorPosCallback(window_, &Impl::mouse_move_callback);
        glfwSetScrollCallback(window_, &Impl::scroll_callback);


        // initialize visualization data structures
        mjv_defaultCamera(&cam_);
        mjv_defaultOption(&opt_);
        mjv_defaultScene(&scn_);
        mjr_defaultContext(&con_);

        // Create scene and context
        mjv_makeScene(m_, &scn_, 2000);
        mjr_makeContext(m_, &con_, mjFONTSCALE_150);
        last_time_ = std::chrono::high_resolution_clock::now();

    }
    void update_fps()
    {
        frame_count_++;
        auto  now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = now - last_time_;
        if (elapsed.count() >= 0.1)
        {
            fps_ = frame_count_/elapsed.count();
            frame_count_ = 0;
            last_time_ = now;
        }
    }

    /**
     * @brief render
     */
    void render()
    {
        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(m_, d_, &opt_, NULL, &cam_, mjCAT_ALL, &scn_);
        mjr_render(viewport, &scn_, &con_);



        if (show_profiler_)
        {
            // Update profiler position based on current viewport size
            mjrRect profiler_rect;
            profiler_rect.left = viewport.width - 310;
            profiler_rect.bottom = viewport.height - 210;
            profiler_rect.width = 300;
            profiler_rect.height = 200;

            // Show MuJoCo's built-in profiler
            std::string simulation_time_text = "t: " + std::to_string(d_->time) + "s";
            mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, {viewport.width - 310, viewport.height - 210, 300, 200},
                        simulation_time_text.c_str(), nullptr, &con_);



            // Show MuJoCo's built-in profiler
            std::string text = "fps: "+std::to_string(fps_);
            mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, {viewport.width - 310, viewport.height - 250, 300, 200},
                        text.c_str(), nullptr, &con_);

            // Show MuJoCo's built-in profiler
            std::string text2 = "ct: "+std::to_string(fps_)+" us";
            mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, {viewport.width - 310, viewport.height - 290, 300, 200},
                        text2.c_str(), nullptr, &con_);


        }


        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window_);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();
        update_fps();
    }

    /**
     * @brief step_simulation
     */
    void step_simulation()
    {
        auto start = std::chrono::high_resolution_clock::now();
        //------------------------------------------------------------
        if (!m_ || !d_)
            throw std::runtime_error("Model or data not initialized");
        mj_step(m_, d_);
        //mj_forward(m_, d_);
        //d_->qpos[0] = 0.5 * sin(2.0 * d_->time);
        qpos_ = Eigen::Map<Eigen::VectorXd>(d_->qpos, m_->nq);
        qvel_ = Eigen::Map<Eigen::VectorXd>(d_->qvel, m_->nv);
        //------------------------------------------------------------

        auto end = std::chrono::high_resolution_clock::now();
        computation_time_ = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    }

    int _get_handle_from_map(const std::string& objectname)
    {
        auto search = handles_map_.find(objectname);
        // returns a tuple <bool, int>
        //If the handle is found in the map, returns <true, handle>.

        if (search != handles_map_.end())
        { // handle found in map
            return search->second;
        }
        else
        {   // handle not found in map. Therefore, it is taken from CoppeliaSim and the map
            // is updated;
            return get_object_handle(objectname);
        }
    }

    int get_object_handle(const std::string& objectname)
    {
        int handle;
        auto error_msg =

            " \n"
            + std::string("The object \"")
            + objectname + std::string("\"")
                         + std::string(" does not exist in the XML model. \n");
        try
        {
            handle = mj_name2id(m_, mjOBJ_JOINT, objectname.c_str());
            //If the handle is not included in the map, then the map is updated.
            if (handle == -1)
                throw std::runtime_error(error_msg);
            handles_map_.try_emplace(objectname, handle);
        }
        catch(const std::runtime_error& e)
        {
            throw std::runtime_error(std::string(e.what()) + error_msg);
        }
        return handle;
    }

    double   _get_joint_position(const std::string& jointname)
    {
        int qpos_adr = m_->jnt_qposadr[_get_handle_from_map(jointname)];
        return qpos_[qpos_adr];
    }

    void _set_joint_target_position(const std::string& jointname, const double& angle_rad)
    {
        int qpos_adr = m_->jnt_qposadr[_get_handle_from_map(jointname)];
        d_->qpos[qpos_adr] = angle_rad;
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

VectorXd DQ_MujocoInterface::get_joint_positions(const std::vector<std::string> &jointnames) const
{
    int n = jointnames.size();
    VectorXd joint_positions(n);
    for(auto i=0;i<n;i++)
        joint_positions(i)=impl_->_get_joint_position(jointnames.at(i));
    return joint_positions;
}

void DQ_MujocoInterface::set_joint_target_positions(const std::vector<std::string>& jointnames,
                                                    const VectorXd& angles_rad)
{
    for(std::size_t i=0;i<jointnames.size();i++)
        impl_->_set_joint_target_position(jointnames.at(i), angles_rad(i));
}


