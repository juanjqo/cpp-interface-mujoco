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
#include <unordered_map>
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

    static void scroll_callback(GLFWwindow* window, [[maybe_unused]] double xoffset, double yoffset) {
        Impl* self = static_cast<Impl*>(glfwGetWindowUserPointer(window));
        if (self) {
            self->scroll(window, yoffset);
        }
    }

    // Mouse button handler
    void mouse_button(GLFWwindow* window, [[maybe_unused]] int button, [[maybe_unused]] int action, [[maybe_unused]] int mods) {
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
    void scroll([[maybe_unused]] GLFWwindow* window, double yoffset) {
        mjv_moveCamera(m_, mjMOUSE_ZOOM, 0, 0.05 * yoffset, &scn_, &cam_);
    }

public:
    //std::thread render_thread_;
    //std::atomic<bool> render_loop_started_{false};
    VectorXd qpos_;
    VectorXd qvel_;
    std::unordered_map<std::string, int> handles_map_; //
    std::unordered_map<std::string, int> actuator_map_; // actuator handles

public:
    std::string xml_model_;
    bool model_loaded_{false};
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
    double avg_computation_time_microseconds_{50.0};
    double computation_time_samples_ = 1000;

    /**
     * @brief Impl
     */
    Impl()
    {
        std::cerr<<"DQ_MujocoInterface::Impl ctor"<<std::endl;
        model_loaded_ = false;
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
        model_loaded_ = true;
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




        // Reduce geometry quality
        //opt_.quality = mjQUALITY_LOW;       // Lowest quality settings


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
        if (elapsed.count() >= 1.0)
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
            /*
            mjrRect profiler_rect;
            profiler_rect.left = viewport.width - 310;
            profiler_rect.bottom = viewport.height - 210;
            profiler_rect.width = 300;
            profiler_rect.height = 200;
            */

            // Show MuJoCo's built-in profiler
            std::string simulation_time_text = "t: " + std::to_string(d_->time) + "s";
            mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, {viewport.width - 310, viewport.height - 210, 300, 200},
                        simulation_time_text.c_str(), nullptr, &con_);



            // Show MuJoCo's built-in profiler
            std::string text = "FPS: "+std::to_string(fps_);
            mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, {viewport.width - 310, viewport.height - 250, 300, 200},
                        text.c_str(), nullptr, &con_);

            // Show MuJoCo's built-in profiler
            std::string text2 = "ct: "+std::to_string(avg_computation_time_microseconds_)+" us";
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

        const double& N = computation_time_samples_;

        avg_computation_time_microseconds_ = (1.0/N)*computation_time_ + ((N-1.0)/N)*avg_computation_time_microseconds_ ;
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
        {
            // handle not found in map, get it from MuJoCo and update the map
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



    /**
     * @brief Get actuator handle from joint name
     * @param jointname Name of the joint
     * @return Actuator index in the range [0, m_->nu)
     * @throws std::runtime_error if joint has no actuator
     *
     * @note MuJoCo's actuator_trnid array stores the joint/body ID for each actuator.
     *       The mapping is: actuator_trnid[2*i] = joint_id for actuator i
     */
    int get_actuator_handle_from_joint(const std::string& jointname)
    {
        auto search = actuator_map_.find(jointname);
        if (search != actuator_map_.end()) {
            return search->second;
        }

        int joint_id = _get_handle_from_map(jointname);
        int actuator_id = -1;

        // Search for actuator associated with this joint
        for (int i = 0; i < m_->nu; i++) {
            if (m_->actuator_trnid[2 * i] == joint_id) {
                actuator_id = i;
                break;
            }
        }

        if (actuator_id == -1)
            throw std::runtime_error("Joint " + jointname + " does not have an actuator");

        actuator_map_.try_emplace(jointname, actuator_id);
        return actuator_id;
    }


    /**
     * @brief _get_joint_position
     * @param jointname Name of the joint to query
     * @return Current joint position in radians (revolute) or meters (prismatic)
     *
     * Returns the actual joint position from the simulation state (d_->qpos).
     * This reflects the current physical position after dynamics have been applied.
     */
    double   _get_joint_position(const std::string& jointname)
    {
        int qpos_adr = m_->jnt_qposadr[_get_handle_from_map(jointname)];
        return qpos_[qpos_adr];
    }


    /**
     * @brief _get_joint_velocity
     * @param jointname Name of the joint to query
     * @return Current joint velocity in rad/s (revolute) or m/s (prismatic)
     *
     * Returns the actual joint velocity from the simulation state (d_->qvel).
     * This is used by MuJoCo's controllers for damping calculations.
     */
    double _get_joint_velocity(const std::string& jointname)
    {
        int qvel_adr = m_->jnt_dofadr[_get_handle_from_map(jointname)];
        return qvel_[qvel_adr];
    }


    void _set_joint_position(const std::string& jointname, const double& angle_rad)
    {
        int qpos_adr = m_->jnt_qposadr[_get_handle_from_map(jointname)];
        d_->qpos[qpos_adr] = angle_rad;
    }

    /**
     * @brief _set_joint_target_position
     * @param jointname Name of the joint to control
     * @param angle_rad Target position in radians
     *
     * When using a position actuator (type="position" in XML), MuJoCo's built-in
     * PD controller computes the applied torque as:
     *
     *     torque = kp * (target_position - actual_position) - kv * actual_velocity
     *
     * Where:
     *   - target_position = d_->ctrl[actuator_id] (set by this method)
     *   - actual_position = d_->qpos (current joint position)
     *   - actual_velocity = d_->qvel (current joint velocity)
     *   - kp = position gain from XML actuator definition
     *   - kv = velocity gain from XML actuator definition
     *
     * The computed torque is applied to the joint and stored in d_->qfrc_actuator.
     *
     * @note This method enables smooth, physically-realistic motion with proper
     *       dynamics, unlike set_joint_position() which teleports the joint.
     *
     * @see set_joint_position() for direct position control without dynamics
     */
    void _set_joint_target_position(const std::string& jointname, const double& angle_rad)
    {
        int actuator_id = get_actuator_handle_from_joint(jointname);
        d_->ctrl[actuator_id] = angle_rad;
    }


    /**
     * @brief _set_joint_target_velocity
     * @param jointname Name of the joint to control
     * @param velocity_rad_per_sec Target velocity in rad/s
     *
     * For velocity actuators, MuJoCo computes:
     *     torque = kv * (target_velocity - actual_velocity)
     *
     * Where kv is the velocity gain from the XML actuator definition.
     */
    void _set_joint_target_velocity(const std::string& jointname, const double& velocity_rad_per_sec)
    {
        int actuator_id = get_actuator_handle_from_joint(jointname);
        d_->ctrl[actuator_id] = velocity_rad_per_sec;
    }


    /**
     * @brief _set_joint_target_force
     * @param jointname Name of the joint to control
     * @param force_nm Target torque in Nm
     *
     * For torque actuators, MuJoCo applies the torque directly:
     *     torque = target_force
     *
     * No PD controller is used; the commanded torque is applied directly.
     */
    void _set_joint_target_force(const std::string& jointname, const double& force_nm)
    {
        int actuator_id = get_actuator_handle_from_joint(jointname);
        d_->ctrl[actuator_id] = force_nm;
    }


    /**
     * @brief _get_joint_force
     * @param jointname Name of the joint to query
     * @return Applied force/torque in Newtons (for prismatic joints) or Newton-meters (for revolute joints)
     *
     * Returns the actual force/torque currently being applied to the joint by the actuator.
     * This value is computed by MuJoCo's actuator controller during mj_step() and stored
     * in d_->qfrc_actuator.
     *
     * For different actuator types, this represents:
     *   - Position actuator: The torque computed by the PD controller
     *     torque = kp * (target_position - actual_position) - kv * actual_velocity
     *
     *   - Velocity actuator: The torque computed by the velocity controller
     *     torque = kv * (target_velocity - actual_velocity)
     *
     *   - Torque actuator: The directly commanded torque
     *     torque = target_force (clamped to ctrlrange if specified)
     *
     * This value represents the actual effort being applied to the joint and can be used for:
     *   - Monitoring actuator saturation
     *   - Implementing torque limits or safety checks
     *   - Feedforward control compensation
     *   - Analyzing control performance
     *
     *
     *   MuJoCo provides several force arrays in mjData:
     *
     * - d_->qfrc_actuator: Forces/torques generated by actuators (what this method returns)
     * - d_->qfrc_passive: Forces from passive elements (springs, dampers)
     * - d_->qfrc_constraint: Constraint forces (contacts, joints limits)
     * - d_->qfrc_inertia: Inertia forces
     * - d_->qfrc_applied: Externally applied forces
     *
     * The total force on a joint is the sum of all these components.
     * For most control applications, qfrc_actuator is the value of interest
     * as it represents the controlled effort.
     *
     *
     * @note The force/torque value is positive in the positive joint direction
     * @note This reflects the actual applied force after any limits (ctrlrange, actuator limits)
     * @see set_joint_target_forces() for commanding torques directly
     * @see set_joint_target_positions() for position-based control that generates torques
     * @see set_joint_target_velocities() for velocity-based control that generates torques
     */
    double _get_joint_force(const std::string& jointname)
    {
        int joint_id = _get_handle_from_map(jointname);
        // qfrc_actuator contains the forces generated by actuators
        return d_->qfrc_actuator[joint_id];
    }


};
///------------------------------------------------------------------------------------------------

DQ_MujocoInterface::DQ_MujocoInterface()
{
    impl_ = std::make_shared<DQ_MujocoInterface::Impl>();
}

void DQ_MujocoInterface::_check_model() const
{
    if (!impl_->model_loaded_)
        throw std::runtime_error("Undefined xml model. You must load the model first");
}



void DQ_MujocoInterface::load_model(const std::string &xml_model)
{
    impl_->load_model(xml_model);
}



void DQ_MujocoInterface::trigger_next_simulation_step() const
{
    _check_model();
    impl_->step_simulation();
}

double DQ_MujocoInterface::get_simulation_time() const
{
    _check_model();
    return impl_->d_->time;
}

bool DQ_MujocoInterface::should_close_window() const
{
    _check_model();
    return glfwWindowShouldClose(impl_->window_);
}

void DQ_MujocoInterface::render()
{
    _check_model();
    impl_->render();
}

VectorXd DQ_MujocoInterface::get_joint_positions(const std::vector<std::string> &jointnames) const
{
    _check_model();
    int n = jointnames.size();
    VectorXd joint_positions(n);
    for(auto i=0;i<n;i++)
        joint_positions(i)=impl_->_get_joint_position(jointnames.at(i));
    return joint_positions;
}

VectorXd DQ_MujocoInterface::get_joint_velocities(const std::vector<std::string>& jointnames) const
{
    _check_model();
    int n = jointnames.size();
    VectorXd joint_velocities(n);
    for(auto i=0;i<n;i++)
        joint_velocities(i)=impl_->_get_joint_velocity(jointnames.at(i));
    return joint_velocities;
}


/**
 * @brief Directly set joint positions (teleportation, no dynamics)
 * @param jointnames Vector of joint names
 * @param angles_rad Vector of desired joint positions
 *
 * @warning This bypasses physics and actuators completely. Use for initialization
 *          or debugging only, not for control during simulation.
 * @see set_joint_target_positions() for proper dynamic control
 */
void DQ_MujocoInterface::set_joint_positions(const std::vector<std::string>& jointnames,
                                                    const VectorXd& angles_rad)
{
    _check_model();
    for(std::size_t i=0;i<jointnames.size();i++)
        impl_->_set_joint_position(jointnames.at(i), angles_rad(i));
}


/**
 * @brief Set target positions for position actuators (with dynamics)
 * @param jointnames Vector of joint names
 * @param angles_rad Vector of desired joint positions
 *
 * Uses MuJoCo's built-in PD controller to generate torques that drive the
 * joints toward the target positions. This produces smooth, realistic motion.
 *
 * @see set_joint_positions() for direct teleportation
 * @see set_joint_target_velocities() for velocity control
 * @see set_joint_target_forces() for torque control
 */
void DQ_MujocoInterface::set_joint_target_positions(const std::vector<std::string>& jointnames,
                                                    const VectorXd& angles_rad)
{
    _check_model();
    for(std::size_t i=0;i<jointnames.size();i++)
        impl_->_set_joint_target_position(jointnames.at(i), angles_rad(i));
}


/**
 * @brief Set target velocities for velocity actuators (with dynamics)
 * @param jointnames Vector of joint names
 * @param velocities_rad_per_sec Vector of desired joint velocities in rad/s
 *
 * Uses MuJoCo's velocity controller to generate torques that drive the
 * joints toward the target velocities.
 *
 * @see set_joint_target_positions() for position control
 * @see set_joint_target_forces() for torque control
 */
void DQ_MujocoInterface::set_joint_target_velocities(const std::vector<std::string>& jointnames,
                                                     const VectorXd& velocities_rad_per_sec)
{
    _check_model();
    for(std::size_t i=0;i<jointnames.size();i++)
        impl_->_set_joint_target_velocity(jointnames.at(i), velocities_rad_per_sec(i));
}

/**
 * @brief Set target forces/torques for torque actuators (direct control)
 * @param jointnames Vector of joint names
 * @param forces Vector of desired forces (N) or torques (Nm)
 *
 * Applies the commanded forces/torques directly to the joints.
 * No PD controller is used - the commanded values are applied as-is
 * (subject to ctrlrange limits if specified in XML).
 *
 * @warning Use with care as there is no built-in damping or position limiting
 * @see set_joint_target_positions() for position control
 * @see set_joint_target_velocities() for velocity control
 */
void DQ_MujocoInterface::set_joint_target_forces(const std::vector<std::string>& jointnames,
                                                 const VectorXd& forces)
{
    _check_model();
    for(std::size_t i=0;i<jointnames.size();i++)
        impl_->_set_joint_target_force(jointnames.at(i), forces(i));
}


/**
 * @brief get_joint_forces
 * @param jointnames Vector of joint names to query
 * @return VectorXd containing the applied forces/torques for all specified joints
 *
 * Returns the actual forces/torques currently being applied to the specified joints
 * by their respective actuators. The order of returned values matches the order
 * of joint names in the input vector.
 *
 * Example usage:
 * @code
 * std::vector<std::string> joints = {"joint1", "joint2", "joint3"};
 * VectorXd torques = robot.get_joint_forces(joints);
 * std::cout << "Joint torques: " << torques.transpose() << std::endl;
 * @endcode
 *
 * @note Forces are in Newtons (prismatic joints), torques in Newton-meters (revolute joints)
 * @see set_joint_target_forces()
 * @see get_joint_positions()
 * @see get_joint_velocities()
 */
VectorXd DQ_MujocoInterface::get_joint_forces(const std::vector<std::string>& jointnames) const
{
    _check_model();
    int n = jointnames.size();
    VectorXd joint_forces(n);
    for(auto i=0;i<n;i++)
        joint_forces(i)=impl_->_get_joint_force(jointnames.at(i));
    return joint_forces;
}


