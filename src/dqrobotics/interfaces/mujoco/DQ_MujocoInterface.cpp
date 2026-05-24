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

        opt_.frame = mjFRAME_SITE;          // Show site frames

        //// bodyframe axis width and length
        m_->vis.scale.framewidth = 0.05;
        m_->vis.scale.framelength = 1;

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
     * @brief Get names of all joints in the model
     * @return Vector of joint names
     *
     * Returns the names of all joints defined in the MuJoCo model.
     * The order of joint names corresponds to the joint indices in MuJoCo.
     */
    std::vector<std::string> _get_joint_names() const
    {
        std::vector<std::string> names;
        names.reserve(m_->njnt);  // Pre-allocate memory for efficiency

        for (int i = 0; i < m_->njnt; i++)
        {
            // MuJoCo stores joint names in the 'names' array with offsets
            const char* joint_name = m_->names + m_->name_jntadr[i];
            names.push_back(std::string(joint_name));
        }
        return names;
    }

    /**
     * @brief Set the position and orientation of a site in the simulation
     *
     * This function modifies the runtime pose of a site. Sites are kinematic markers
     * that do not participate in physics or collision detection, making them ideal
     * for tracking end-effector positions, defining reference frames, or visual debugging.
     *
     * The pose is set in the data structure (d_->site_xpos, d_->site_xmat) rather than
     * the model structure because:
     * - m_->site_pos is static and loaded from XML (read-only after initialization)
     * - d_->site_xpos stores the current runtime state that updates each simulation step
     *
     * @param site_name The name of the site as defined in the MuJoCo XML file
     * @param pose The target pose as a Dual Quaternion (DQ) containing:
     *             - Translation: x, y, z position in world coordinates
     *             - Rotation: quaternion (w, x, y, z) orientation
     *
     * @throws std::runtime_error If site_name is not found in the model
     *
     * @note After setting the pose, the change is immediate but may be overwritten
     *       in the next simulation step if the site is attached to a moving body.
     *       For static sites (like test_frame), the pose will persist until changed.
     *
     * @example
     *   DQ target_pose;  // Create a dual quaternion
     *   target_pose.set_translation(0.5, 0.2, 0.3);
     *   target_pose.set_rotation(1, 0, 0, 0);  // Identity quaternion
     *   _set_site_pose("test_frame", target_pose);
     */
    void _set_site_pose(const std::string& site_name, const DQ& pose)
    {
        int site_id = mj_name2id(m_, mjOBJ_SITE, site_name.c_str());
        if (site_id == -1)
            throw std::runtime_error("Site '" + site_name + "' not found");

        VectorXd p = pose.translation().vec3();
        VectorXd r = pose.P().vec4();

        // Set position
        d_->site_xpos[3 * site_id]     = p(0);
        d_->site_xpos[3 * site_id + 1] = p(1);
        d_->site_xpos[3 * site_id + 2] = p(2);

        // Set orientation from quaternion
        mju_quat2Mat(d_->site_xmat + 9 * site_id, r.data());

    }



    /**
     * @brief Get the current position and orientation of a site from the simulation
     *
     * This function retrieves the runtime pose of a site directly from MuJoCo's
     * data structure. The returned pose reflects any modifications made by:
     * - Previous calls to _set_site_pose()
     * - Physics simulation (if site is attached to a moving body)
     * - Control system updates
     *
     * @param site_name The name of the site as defined in the MuJoCo XML file
     * @return DQ Dual quaternion containing the site's current pose:
     *         - Translation: world position (x, y, z)
     *         - Rotation: quaternion (w, x, y, z) orientation
     *
     * @throws std::runtime_error If site_name is not found in the model
     *
     * @note This function reads from the data structure, which always contains
     *       the most up-to-date simulation state.
     *
     *       <site name="test_frame" type="sphere" size="0.05" rgba="1 0 0 0.5" pos="0 0 0"/>
     *
     * @example
     *   // Get current end-effector pose for control
     *   DQ current_pose = _get_site_pose("test_frame");
     *   VectorXd pos = current_pose.translation().vec3();
     *   std::cout << "Site position: (" << pos(0) << ", " << pos(1) << ", " << pos(2) << ")" << std::endl;
     */
    DQ _get_site_pose(const std::string& site_name)
    {
        // Get the site ID
        int site_id = mj_name2id(m_, mjOBJ_SITE, site_name.c_str());
        if (site_id == -1)
            throw std::runtime_error("Site '" + site_name + "' not found");

        // Get position from d_->site_xpos (3 values per site)
        // Note: Use site_xpos for sites, xpos is for bodies
        int pos_idx = 3 * site_id;
        DQ p = d_->xpos[pos_idx]    * i_ +
               d_->xpos[pos_idx + 1] * j_ +
               d_->xpos[pos_idx + 2] * k_;

        // Get orientation from d_->site_xquat (4 values per site: w, x, y, z)
        int quat_idx = 4 * site_id;
        DQ r = DQ(d_->xquat[quat_idx],      // w
                  d_->xquat[quat_idx + 1],  // x
                  d_->xquat[quat_idx + 2],  // y
                  d_->xquat[quat_idx + 3]); // z
        r = r.normalize();  // Ensure it's a unit quaternion

        // Return the dual quaternion pose: r + 0.5 * ε * p * r
        return r + 0.5 * E_ * p * r;
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



    /**
     * @brief _set_joint_position
     * @param jointname Name of the joint to set
     * @param angle_rad Desired joint position in radians (revolute) or meters (prismatic)
     *
     * Directly sets the joint position in the simulation state by writing to d_->qpos.
     * This completely bypasses MuJoCo's physics engine, actuators, and controllers.
     *
     * @warning This method teleports the joint to the desired position instantly,
     *          ignoring any physical laws, constraints, or actuator limits.
     *          Use this ONLY for:
     *          - Initializing the robot to a starting configuration
     *          - Resetting the simulation to a known state
     *          - Debugging or testing purposes
     *          - Teleporting objects without dynamics
     *
     * @warning Using this during active control will override any actuator
     *          commands and may cause unexpected behavior or simulation instability.
     *
     * @note This method does NOT generate torques in d_->qfrc_actuator.
     * @note The change takes effect immediately and will affect subsequent physics steps.
     *
     * @see _set_joint_target_position() for proper dynamic position control
     * @see _set_joint_target_velocity() for velocity control
     * @see _set_joint_target_force() for torque control
     *
     * Example of appropriate use:
     * @code
     * // Initialize robot to home position before simulation starts
     * _set_joint_position("joint1", 0.0);
     * _set_joint_position("joint2", 1.57);
     * @endcode
     *
     * Example of inappropriate use:
     * @code
     * // DO NOT use for control during simulation - this bypasses physics!
     * while(simulating) {
     *     _set_joint_position("joint1", target_position);  // WRONG!
     * }
     * @endcode
     */
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


/**
 * @brief Enable or disable wireframe rendering mode
 * @param flag true to enable wireframe view, false to disable
 *
 * When enabled, all geometries are rendered as wireframe outlines instead
 * of solid surfaces. This is useful for:
 * - Debugging collision geometries and joint alignments
 * - Understanding the structure of complex assemblies
 * - Visualizing internal components that would otherwise be hidden
 *
 * Example usage:
 * @code
 * set_wireframe_view(true);   // Enable wireframe
 * set_wireframe_view(false);  // Disable wireframe (back to solid)
 * @endcode
 *
 * @note Wireframe view can also be toggled during simulation for dynamic debugging
 * @see toggle_wireframe() for easy key-binding
 */
void DQ_MujocoInterface::set_wireframe_view(const bool &flag)
{
    _check_model();
    impl_->scn_.flags[mjRND_WIREFRAME] = flag ? 1 : 0;
}


/**
 * @brief Enable or disable joint frame/axis visualization
 * @param flag true to show joint axes, false to hide them
 *
 * When enabled, this visualizes the position and orientation of all joint axes
 * in the model. Each joint is displayed as a line or arrow indicating:
 * - The joint's position in space
 * - The axis direction (revolute joints: rotation axis, prismatic joints: translation axis)
 *
 * This is particularly useful for:
 * - Debugging kinematic chains and joint placements
 * - Verifying joint axis orientations are correct
 * - Understanding robot structure without loading a full GUI
 * - Teaching robot kinematics concepts
 *
 * Example usage:
 * @code
 * robot.set_joint_view(true);   // Show all joint axes
 * robot.set_joint_view(false);  // Hide joint axes
 * @endcode
 *
 * @note Joint axes are typically color-coded:
 *       - Red: X-axis
 *       - Green: Y-axis
 *       - Blue: Z-axis
 *
 * @note This visualizes the FRAMES of joints, not to be confused with:
 *       - set_wireframe_view() for mesh wireframes
 *       - opt_.frame = mjFRAME_SITE for site frames
 *
 * @see set_wireframe_view() for wireframe rendering
 * @see mjVIS_JOINT flag in MuJoCo documentation
 */
void DQ_MujocoInterface::set_joint_view(const bool &flag)
{
    _check_model();
    impl_->opt_.flags[mjVIS_JOINT] = flag ? 1 : 0; // Enable visualization of joint axes
}

void DQ_MujocoInterface::set_contact_view(const bool &flag)
{
    _check_model();
    impl_->opt_.flags[mjVIS_CONTACTFORCE] = flag ? 1 : 0;
}

void DQ_MujocoInterface::set_center_of_mass_view(const bool &flag)
{
    _check_model();
    impl_->opt_.flags[mjVIS_COM] = flag ? 1 : 0;
}


/**
 * @brief Get current joint positions
 * @param jointnames Vector of joint names to query
 * @return VectorXd containing the current positions of the specified joints
 *
 * Returns the actual joint positions from the simulation state (d_->qpos).
 * These values reflect the current physical positions after all dynamics,
 * constraints, and control actions have been applied.
 *
 * For different joint types:
 * - Revolute joints: Position in radians (range typically -π to π or continuous)
 * - Prismatic joints: Position in meters
 *
 * The order of returned values matches the order of joint names in the input vector.
 *
 * Example usage:
 * @code
 * std::vector<std::string> joints = {"joint1", "joint2", "joint3"};
 * VectorXd positions = robot.get_joint_positions(joints);
 * std::cout << "Current positions: " << positions.transpose() << std::endl;
 *
 * // Get individual joint position
 * double joint1_pos = robot.get_joint_positions({"joint1"})(0);
 * @endcode
 *
 * @note This method reads the actual state after physics simulation
 * @note Use set_joint_target_positions() to command new positions
 * @note Use set_joint_positions() to teleport positions (bypasses physics)
 *
 * @see get_joint_velocities() for reading velocities
 * @see get_joint_forces() for reading applied forces/torques
 * @see set_joint_target_positions() for commanding positions with dynamics
 */
VectorXd DQ_MujocoInterface::get_joint_positions(const std::vector<std::string> &jointnames) const
{
    _check_model();
    int n = jointnames.size();
    VectorXd joint_positions(n);
    for(auto i=0;i<n;i++)
        joint_positions(i)=impl_->_get_joint_position(jointnames.at(i));
    return joint_positions;
}


/**
 * @brief Get current joint velocities
 * @param jointnames Vector of joint names to query
 * @return VectorXd containing the current velocities of the specified joints
 *
 * Returns the actual joint velocities from the simulation state (d_->qvel).
 * These values reflect the current physical velocities after all dynamics,
 * constraints, and control actions have been applied.
 *
 * For different joint types:
 * - Revolute joints: Angular velocity in radians per second (rad/s)
 * - Prismatic joints: Linear velocity in meters per second (m/s)
 *
 * The order of returned values matches the order of joint names in the input vector.
 *
 * Example usage:
 * @code
 * std::vector<std::string> joints = {"joint1", "joint2", "joint3"};
 * VectorXd velocities = robot.get_joint_velocities(joints);
 * std::cout << "Current velocities: " << velocities.transpose() << " rad/s" << std::endl;
 *
 * // Check if robot is moving
 * if (robot.get_joint_velocities(joints).norm() < 0.01) {
 *     std::cout << "Robot is stationary" << std::endl;
 * }
 * @endcode
 *
 * @note Velocities are used by MuJoCo's controllers for damping calculations
 * @note In position control mode, the damping term is: -kv * actual_velocity
 * @note Positive velocity indicates motion in the positive joint direction
 *
 * @see get_joint_positions() for reading positions
 * @see get_joint_forces() for reading applied forces/torques
 * @see set_joint_target_velocities() for commanding velocities with dynamics
 */
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
 * @brief Directly set joint positions (teleportation, bypasses physics)
 * @param jointnames Vector of joint names
 * @param angles_rad Vector of desired joint positions (radians or meters)
 *
 * This method teleports the specified joints to the desired positions
 * instantly, completely bypassing MuJoCo's physics engine and actuators.
 * No dynamics, constraints, or actuator limits are applied.
 *
 * @warning This method is NOT suitable for real-time control during simulation.
 *          Use set_joint_target_positions() instead for proper dynamic control.
 *
 * Appropriate use cases:
 * - Initializing robot pose before simulation starts
 * - Resetting simulation to a known configuration
 * - Debugging or testing specific joint configurations
 * - Positioning objects without simulating motion
 *
 * Inappropriate use cases:
 * - Real-time feedback control
 * - Testing controller performance
 * - Any scenario where realistic physics is required
 *
 * Example:
 * @code
 * std::vector<std::string> joints = {"shoulder", "elbow", "wrist"};
 * VectorXd home_pos(3);
 * home_pos << 0.0, 1.57, 0.0;
 *
 * // Initialize robot to home position at start
 * robot.set_joint_positions(joints, home_pos);
 *
 * // Then use dynamic control during simulation
 * while(simulating) {
 *     robot.set_joint_target_positions(joints, desired_positions);
 *     robot.trigger_next_simulation_step();
 * }
 * @endcode
 *
 * @note The simulation will continue from this teleported state in the next step
 * @see set_joint_target_positions() for dynamic position control with physics
 * @see set_joint_target_velocities() for dynamic velocity control
 * @see set_joint_target_forces() for direct torque control
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

DQ DQ_MujocoInterface::get_site_pose(const std::string &objectname) const
{
    _check_model();
    return impl_->_get_site_pose(objectname);
}

void DQ_MujocoInterface::set_site_pose(const std::string &objectname, const DQ &pose)
{
    // For C++20
    // std::string function_name = static_cast<std::string>(std::source_location::current().function_name());
    std::string function_name = {"DQ_MujocoInterface::set_site_pose"};
    if (!is_unit(pose))
        throw std::runtime_error(function_name + ". The pose must be a unit dual quaternion!");
    _check_model();
    impl_->_set_site_pose(objectname, pose);
}


/**
 * @brief Get names of all joints in the loaded model
 * @return Vector containing the names of all joints
 *
 * Returns the names of all joints defined in the MuJoCo XML model.
 * The order of joint names corresponds to the joint indices in MuJoCo.
 *
 * Example usage:
 * @code
 * DQ_MujocoInterface robot;
 * robot.load_model("robot.xml");
 * std::vector<std::string> all_joints = robot.get_joint_names();
 * std::cout << "Robot has " << all_joints.size() << " joints:" << std::endl;
 * for (const auto& joint : all_joints) {
 *     std::cout << "  - " << joint << std::endl;
 * }
 * @endcode
 *
 * @note This method is useful for discovering available joints without
 *       manually specifying them from the XML file.
 * @see get_number_of_joints() for getting just the count
 */
std::vector<std::string> DQ_MujocoInterface::get_joint_names() const
{
    _check_model();
    return impl_->_get_joint_names();
}

