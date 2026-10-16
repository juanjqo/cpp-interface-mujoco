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
#include <dqrobotics_extensions/interfaces/mujoco/DQ_MujocoInterface.h>
#include <unordered_map>
#include <mutex>
#include <algorithm>
#include <cstring>
#include <map>
//#include <thread>
#include "mujoco/mujoco.h"
#include "GLFW/glfw3.h"


[[maybe_unused]] static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}


namespace DQ_robotics_extensions
{

using namespace DQ_robotics;
using namespace Eigen;

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

    // Window settings
    RENDERING rendering_{RENDERING::WINDOW};
    std::string window_title_;
    int window_width_{1280};
    int window_height_{920};

    // GLFW is process-wide: it is initialized by the first interface that opens a window
    // and terminated only when the last one is destroyed.
    inline static std::mutex glfw_mutex_;
    inline static int glfw_users_{0};
    bool glfw_acquired_{false};

    // Site poses defined in the XML. set_site_pose() modifies the model, so these are
    // used to restore the original poses in reset().
    std::vector<mjtNum> site_pos0_;
    std::vector<mjtNum> site_quat0_;
    std::vector<mjtByte> site_sameframe0_;

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

    // Reference frames added at runtime. They are drawn as decorations in render() and do not
    // exist in the MuJoCo model. std::map keeps a deterministic drawing order.
    struct ReferenceFrame
    {
        DQ pose = DQ(1);
        mjtNum pos[3] = {0, 0, 0};
        mjtNum mat[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};  // rotation matrix, row-major
        double axis_length;
        double axis_width;
        float opacity;
        bool show_label;
    };
    std::map<std::string, ReferenceFrame> reference_frames_;
    bool scene_full_warned_{false};

    /**
     * @brief Impl
     */
    Impl(const RENDERING& rendering, const std::string& window_title,
         const int& window_width, const int& window_height)
        : rendering_(rendering), window_title_(window_title),
          window_width_(window_width), window_height_(window_height)
    {
        if (window_width <= 0 || window_height <= 0)
            throw std::invalid_argument("The window width and height must be positive");

        // Give the visualization structures a valid empty state, so that they can be
        // freed safely even if no model is ever loaded.
        mjv_defaultCamera(&cam_);
        mjv_defaultOption(&opt_);
        mjv_defaultScene(&scn_);
        mjr_defaultContext(&con_);
    };

    /**
     * @brief ~Impl() Destructor of the class
     */
    ~Impl(){
        // free visualization storage while the window (and its OpenGL context) still exists
        _free_visualization();

        // close the window, and GLFW if this is the last interface using it
        if (window_) glfwDestroyWindow(window_);
        _release_glfw();

        //free MuJoCo model and data
        if (d_) mj_deleteData(d_);
        if (m_) mj_deleteModel(m_);
    }

    /**
     * @brief _acquire_glfw initializes GLFW if no other interface has done it yet.
     */
    void _acquire_glfw()
    {
        if (glfw_acquired_)
            return;
        std::lock_guard<std::mutex> lock(glfw_mutex_);
        if (glfw_users_ == 0 && !glfwInit())
            throw std::runtime_error("Error in glfwInit()");
        glfw_users_++;
        glfw_acquired_ = true;
    }

    /**
     * @brief _release_glfw terminates GLFW if this is the last interface using it.
     */
    void _release_glfw()
    {
        if (!glfw_acquired_)
            return;
        std::lock_guard<std::mutex> lock(glfw_mutex_);
        if (--glfw_users_ == 0)
            glfwTerminate();
        glfw_acquired_ = false;
    }

    /**
     * @brief _free_visualization frees the scene and the GPU context.
     */
    void _free_visualization()
    {
        if (window_)
        {
            // mjr_freeContext releases OpenGL resources, so it needs this window's context
            glfwMakeContextCurrent(window_);
            mjr_freeContext(&con_);
        }
        mjv_freeScene(&scn_);
    }

    /**
     * @brief load_model
     * @param xml_model
     */
    void load_model(const std::string& xml_model)
    {
        if (xml_model.empty())
            throw std::invalid_argument("xml model path cannot be empty");
        // load model from file and check for errors
        char error[1000] = "";
        mjModel* m = mj_loadXML(xml_model.c_str(), NULL, error, sizeof(error));
        if (!m)
            throw std::runtime_error("Failed to load the xml model \"" + xml_model + "\": " + error);

         // make data corresponding to model
        mjData* d = mj_makeData(m);
        if (!d)
        {
            mj_deleteModel(m);
            throw std::runtime_error("Failed to allocate the data for the xml model \"" + xml_model + "\"");
        }

        // The new model is valid: release the previous one, if any
        model_loaded_ = false;
        _free_visualization();
        if (d_) mj_deleteData(d_);
        if (m_) mj_deleteModel(m_);
        handles_map_.clear();
        actuator_map_.clear();

        m_ = m;
        d_ = d;
        xml_model_ = xml_model;
        site_pos0_.assign(m_->site_pos, m_->site_pos + 3 * m_->nsite);
        site_quat0_.assign(m_->site_quat, m_->site_quat + 4 * m_->nsite);
        site_sameframe0_.assign(m_->site_sameframe, m_->site_sameframe + m_->nsite);

        // compute derived quantities (body/site poses, etc.) for the initial state
        mj_forward(m_, d_);
        if (rendering_ == RENDERING::WINDOW)
        {
            // Use the model name (<mujoco model="...">) when no title was given
            std::string title = window_title_;
            if (title.empty())
                title = m_->names;  // the model name is the first entry of the names buffer
            if (title.empty())
                title = "DQ_MujocoInterface";
            initialize(window_width_, window_height_, title);
        }
        model_loaded_ = true;
    }

    /**
     * @brief reset resets the simulation state and the site poses to those defined in the XML.
     * @param keyframe_id Keyframe to reset the state to, or -1 for the initial configuration
     *        of the model (qpos0).
     */
    void reset(const int& keyframe_id = -1)
    {
        if (keyframe_id < 0)
            mj_resetData(m_, d_);
        else
            mj_resetDataKeyframe(m_, d_, keyframe_id);
        std::copy(site_pos0_.begin(), site_pos0_.end(), m_->site_pos);
        std::copy(site_quat0_.begin(), site_quat0_.end(), m_->site_quat);
        std::copy(site_sameframe0_.begin(), site_sameframe0_.end(), m_->site_sameframe);
        // update body/site poses, etc. to match the reset state
        mj_forward(m_, d_);
    }

    /**
     * @brief initialize opens the window (only the first time) and creates the scene and
     *        GPU context for the current model.
     * @param width
     * @param height
     * @param title
     */
    void initialize(const int &width, const int &height, const std::string& title)
    {
        if (!window_)
        {
            _acquire_glfw();
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
        }
        glfwMakeContextCurrent(window_);

        // Reduce geometry quality
        //opt_.quality = mjQUALITY_LOW;       // Lowest quality settings


        // initialize visualization data structures. The free camera starts as set in the model
        // (<statistic center/extent> and <visual><global azimuth/elevation>).
        mjv_defaultFreeCamera(m_, &cam_);
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
     * @brief _acquire_decoration_geom returns the next free geom of the scene, initialized as a
     *        decoration (it mirrors MuJoCo's internal acquireGeom). The caller must increment
     *        scn_.ngeom once the geom is filled.
     * @return nullptr if the scene is full.
     */
    mjvGeom* _acquire_decoration_geom()
    {
        if (scn_.ngeom >= scn_.maxgeom)
        {
            if (!scene_full_warned_)
            {
                std::cerr << "DQ_MujocoInterface: the scene is full (" << scn_.maxgeom
                          << " geoms), so some reference frames are not drawn." << std::endl;
                scene_full_warned_ = true;
            }
            return nullptr;
        }
        mjvGeom* geom = scn_.geoms + scn_.ngeom;
        std::memset(geom, 0, sizeof(mjvGeom));
        mjv_initGeom(geom, mjGEOM_NONE, nullptr, nullptr, nullptr, nullptr);
        geom->objtype = mjOBJ_UNKNOWN;
        geom->objid = -1;
        geom->category = mjCAT_DECOR;
        geom->segid = scn_.ngeom;
        return geom;
    }

    /**
     * @brief _draw_reference_frame adds the geoms of a reference frame to the scene: one arrow per
     *        axis (x: red, y: green, z: blue) and, if enabled, a small sphere at the origin with the
     *        name of the frame as label.
     */
    void _draw_reference_frame(const std::string& name, const ReferenceFrame& frame)
    {
        static const float axis_rgb[3][3] = {{0.9f, 0.1f, 0.1f},
                                             {0.1f, 0.8f, 0.1f},
                                             {0.1f, 0.3f, 0.9f}};
        for (int axis = 0; axis < 3; axis++)
        {
            mjvGeom* geom = _acquire_decoration_geom();
            if (!geom)
                return;
            // The axis direction is the corresponding column of the rotation matrix
            mjtNum to[3];
            for (int k = 0; k < 3; k++)
                to[k] = frame.pos[k] + frame.axis_length * frame.mat[3 * k + axis];
            mjv_connector(geom, mjGEOM_ARROW, frame.axis_width, frame.pos, to);
            for (int k = 0; k < 3; k++)
                geom->rgba[k] = axis_rgb[axis][k];
            geom->rgba[3] = frame.opacity;
            geom->transparent = frame.opacity < 1.0f;
            scn_.ngeom++;
        }

        if (frame.show_label)
        {
            mjvGeom* geom = _acquire_decoration_geom();
            if (!geom)
                return;
            const mjtNum size[3] = {1.5 * frame.axis_width, 0, 0};
            const float rgba[4] = {0.9f, 0.9f, 0.9f, frame.opacity};
            mjv_initGeom(geom, mjGEOM_SPHERE, size, frame.pos, frame.mat, rgba);
            geom->transparent = frame.opacity < 1.0f;
            std::strncpy(geom->label, name.c_str(), sizeof(geom->label) - 1);
            geom->label[sizeof(geom->label) - 1] = '\0';
            scn_.ngeom++;
        }
    }

    /**
     * @brief render
     */
    void render()
    {
        // render into this interface's window (several interfaces may be open)
        glfwMakeContextCurrent(window_);

        // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

        // update scene and render
        mjv_updateScene(m_, d_, &opt_, NULL, &cam_, mjCAT_ALL, &scn_);
        for (const auto& [name, frame] : reference_frames_)
            _draw_reference_frame(name, frame);
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
     * @brief _get_single_dof_joint_handle returns the handle of a hinge or slide joint.
     *
     * The joint methods read and write one value per joint, which is only meaningful for joints
     * with a single DoF. A free joint has 7 positions (x, y, z, qw, qx, qy, qz) and 6 velocities,
     * and a ball joint has 4 and 3, so reading one value of them would silently return a single
     * coordinate (e.g., the x position of a floating base).
     *
     * @throws std::invalid_argument if the joint is a free or ball joint.
     */
    int _get_single_dof_joint_handle(const std::string& jointname)
    {
        const int joint_id = _get_handle_from_map(jointname);
        const int joint_type = m_->jnt_type[joint_id];
        if (joint_type == mjJNT_FREE)
        {
            const char* bodyname = mj_id2name(m_, mjOBJ_BODY, m_->jnt_bodyid[joint_id]);
            throw std::invalid_argument("The joint \"" + jointname + "\" is a free joint (a floating base). "
                                        "Use get_base_pose(), set_base_pose(), get_base_twist() or "
                                        "set_base_twist() with the name of its body (\""
                                        + std::string(bodyname ? bodyname : "") + "\") instead.");
        }
        if (joint_type == mjJNT_BALL)
            throw std::invalid_argument("The joint \"" + jointname + "\" is a ball joint. The joint "
                                        "methods only support hinge and slide joints.");
        return joint_id;
    }

    /**
     * @brief _get_body_handle
     * @throws std::runtime_error if the body does not exist.
     */
    int _get_body_handle(const std::string& bodyname) const
    {
        const int body_id = mj_name2id(m_, mjOBJ_BODY, bodyname.c_str());
        if (body_id == -1)
            throw std::runtime_error("Body '" + bodyname + "' not found");
        return body_id;
    }

    /**
     * @brief _get_free_joint_handle returns the free joint of a floating base.
     * @param bodyname Name of the body that owns the free joint (e.g., "pelvis" or "trunk").
     *
     * Free joints are addressed by their body because they are often unnamed (<freejoint/>).
     * MuJoCo only accepts a free joint as the single joint of a child of the world body, so its
     * qpos is the pose of the body in the world.
     *
     * @throws std::runtime_error if the body does not exist or has no free joint.
     */
    int _get_free_joint_handle(const std::string& bodyname) const
    {
        const int body_id = _get_body_handle(bodyname);
        const int joint_id = m_->body_jntadr[body_id];
        if (m_->body_jntnum[body_id] != 1 || m_->jnt_type[joint_id] != mjJNT_FREE)
            throw std::runtime_error("Body '" + bodyname + "' is not a floating base (it has no free joint)");
        return joint_id;
    }

    /**
     * @brief _get_base_pose returns the pose of a floating base in the world.
     *
     * Read from qpos (the state that mj_step integrates), so it reflects the latest step.
     */
    DQ _get_base_pose(const std::string& bodyname) const
    {
        const mjtNum* qpos = d_->qpos + m_->jnt_qposadr[_get_free_joint_handle(bodyname)];
        // qpos of a free joint: position (x, y, z) followed by the quaternion (w, x, y, z)
        DQ r = DQ(qpos[3], qpos[4], qpos[5], qpos[6]).normalize();
        DQ p = qpos[0]*i_ + qpos[1]*j_ + qpos[2]*k_;
        return r + 0.5*E_*p*r;
    }

    /**
     * @brief _set_base_pose writes the pose of a floating base to qpos. The caller must run
     *        mj_forward() to update the derived quantities (body and site poses, etc.).
     */
    void _set_base_pose(const std::string& bodyname, const DQ& pose)
    {
        mjtNum* qpos = d_->qpos + m_->jnt_qposadr[_get_free_joint_handle(bodyname)];
        const VectorXd p = pose.translation().vec3();
        const VectorXd r = pose.P().vec4();
        std::copy(p.data(), p.data() + 3, qpos);
        std::copy(r.data(), r.data() + 4, qpos + 3);
    }

    /**
     * @brief _get_base_twist returns the twist of a floating base.
     * @param reference_frame WORLD_FRAME: xi = w + E_*(p_dot + cross(p, w)), with w and p_dot in
     *        the world frame (x_dot = 0.5*xi*x). BODY_FRAME: xi_body = w_body + E_*p_dot_body, the
     *        angular velocity and the velocity of the body origin in the body frame
     *        (x_dot = 0.5*x*xi_body), which equals Ad(conj(x), xi).
     *
     * The qvel of a free joint mixes frames: the first three values are the linear velocity of
     * the body origin in the WORLD frame, and the last three are the angular velocity in the BODY
     * frame.
     */
    DQ _get_base_twist(const std::string& bodyname, const REFERENCE_FRAME& reference_frame) const
    {
        const mjtNum* qvel = d_->qvel + m_->jnt_dofadr[_get_free_joint_handle(bodyname)];
        const DQ x = _get_base_pose(bodyname);
        const DQ r = x.P();
        const DQ p_dot = qvel[0]*i_ + qvel[1]*j_ + qvel[2]*k_;   // world frame
        const DQ w_body = qvel[3]*i_ + qvel[4]*j_ + qvel[5]*k_;  // body frame
        if (reference_frame == REFERENCE_FRAME::BODY_FRAME)
            return w_body + E_*(conj(r)*p_dot*r);
        const DQ w = r*w_body*conj(r);
        return w + E_*(p_dot + cross(x.translation(), w));
    }

    /**
     * @brief _set_base_twist writes the twist of a floating base (see _get_base_twist()) to qvel.
     *        The caller must run mj_forward().
     */
    void _set_base_twist(const std::string& bodyname, const DQ& twist, const REFERENCE_FRAME& reference_frame)
    {
        mjtNum* qvel = d_->qvel + m_->jnt_dofadr[_get_free_joint_handle(bodyname)];
        const DQ x = _get_base_pose(bodyname);
        const DQ r = x.P();
        VectorXd p_dot, w_body;
        if (reference_frame == REFERENCE_FRAME::BODY_FRAME)
        {
            p_dot = (r*twist.D()*conj(r)).vec3();  // body -> world
            w_body = twist.P().vec3();
        }
        else
        {
            const DQ w = twist.P();
            p_dot = (twist.D() - cross(x.translation(), w)).vec3();
            w_body = (conj(r)*w*r).vec3();        // world -> body
        }
        std::copy(p_dot.data(), p_dot.data() + 3, qvel);
        std::copy(w_body.data(), w_body.data() + 3, qvel + 3);
    }

    /**
     * @brief _get_body_pose returns the pose of a body frame in the world.
     *
     * Read from d_->xpos and d_->xquat, which, like the site poses, are computed at the beginning
     * of mj_step(). After a step they therefore correspond to the state before that step.
     */
    DQ _get_body_pose(const std::string& bodyname) const
    {
        const int body_id = _get_body_handle(bodyname);
        const mjtNum* xpos = d_->xpos + 3 * body_id;
        const mjtNum* xquat = d_->xquat + 4 * body_id;
        DQ r = DQ(xquat[0], xquat[1], xquat[2], xquat[3]).normalize();
        DQ p = xpos[0]*i_ + xpos[1]*j_ + xpos[2]*k_;
        return r + 0.5*E_*p*r;
    }

    /**
     * @brief _get_floating_base_names returns the names of the bodies that have a free joint.
     *        Unnamed bodies are skipped, since they cannot be addressed by name.
     */
    std::vector<std::string> _get_floating_base_names() const
    {
        std::vector<std::string> names;
        for (int j = 0; j < m_->njnt; j++)
        {
            if (m_->jnt_type[j] != mjJNT_FREE)
                continue;
            const char* bodyname = mj_id2name(m_, mjOBJ_BODY, m_->jnt_bodyid[j]);
            if (bodyname)
                names.push_back(bodyname);
        }
        return names;
    }



    /**
     * @brief Get actuator handle from joint name
     * @param jointname Name of the joint
     * @return Actuator index in the range [0, m_->nu)
     * @throws std::runtime_error if joint has no actuator
     *
     * @note MuJoCo's actuator_trnid array stores the transmission target ID for each actuator
     *       (a joint, tendon, site or body ID, depending on actuator_trntype). For joint
     *       actuators, the mapping is: actuator_trnid[2*i] = joint_id for actuator i
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
            const int trntype = m_->actuator_trntype[i];
            const bool acts_on_joint = (trntype == mjTRN_JOINT || trntype == mjTRN_JOINTINPARENT);
            if (acts_on_joint && m_->actuator_trnid[2 * i] == joint_id) {
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
     * @param only_single_dof If true, skip free and ball joints and unnamed joints
     * @return Vector of joint names
     *
     * Returns the names of all joints defined in the MuJoCo model.
     * The order of joint names corresponds to the joint indices in MuJoCo.
     */
    std::vector<std::string> _get_joint_names(const bool& only_single_dof) const
    {
        std::vector<std::string> names;
        names.reserve(m_->njnt);  // Pre-allocate memory for efficiency

        for (int i = 0; i < m_->njnt; i++)
        {
            // MuJoCo stores joint names in the 'names' array with offsets
            const char* joint_name = m_->names + m_->name_jntadr[i];
            if (only_single_dof)
            {
                const int joint_type = m_->jnt_type[i];
                if (joint_type == mjJNT_FREE || joint_type == mjJNT_BALL || joint_name[0] == '\0')
                    continue;
            }
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
     * MuJoCo recomputes d_->site_xpos and d_->site_xmat from the model (m_->site_pos,
     * m_->site_quat) on every mj_step(), so writing only to mjData would be overwritten
     * one step later. This method therefore:
     * - converts the desired world pose into the frame of the site's parent body and
     *   writes it to m_->site_pos / m_->site_quat, so it persists across steps
     * - also writes d_->site_xpos / d_->site_xmat so the change is visible immediately
     *
     * @param site_name The name of the site as defined in the MuJoCo XML file
     * @param pose The target pose as a Dual Quaternion (DQ) containing:
     *             - Translation: x, y, z position in world coordinates
     *             - Rotation: quaternion (w, x, y, z) orientation
     *
     * @throws std::runtime_error If site_name is not found in the model
     *
     * @note The pose is stored relative to the parent body. For sites in the worldbody
     *       (like test_frame) it stays fixed in the world until changed. For sites attached
     *       to a moving body, it follows that body after the next simulation step.
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

        // Express the desired world pose in the frame of the parent body:
        // local = inverse(body_pose) * world_pose
        int body_id = m_->site_bodyid[site_id];
        mjtNum body_inv_pos[3], body_inv_quat[4];
        mju_negPose(body_inv_pos, body_inv_quat, d_->xpos + 3 * body_id, d_->xquat + 4 * body_id);
        mju_mulPose(m_->site_pos + 3 * site_id, m_->site_quat + 4 * site_id,
                    body_inv_pos, body_inv_quat, p.data(), r.data());

        // Sites declared at the body origin are flagged by the compiler to copy the body frame
        // and ignore site_pos/site_quat. Force MuJoCo to use the values written above.
        m_->site_sameframe[site_id] = mjSAMEFRAME_NONE;

        // Update the runtime state so the new pose is visible before the next step
        d_->site_xpos[3 * site_id]     = p(0);
        d_->site_xpos[3 * site_id + 1] = p(1);
        d_->site_xpos[3 * site_id + 2] = p(2);
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
        const mjtNum* xpos = d_->site_xpos + 3 * site_id;
        DQ p = xpos[0] * i_ + xpos[1] * j_ + xpos[2] * k_;

        // mjData stores site orientations only as rotation matrices (site_xmat, 9 values per site)
        mjtNum quat[4]; // w, x, y, z
        mju_mat2Quat(quat, d_->site_xmat + 9 * site_id);
        DQ r = DQ(quat[0], quat[1], quat[2], quat[3]);
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
        int qpos_adr = m_->jnt_qposadr[_get_single_dof_joint_handle(jointname)];
        return d_->qpos[qpos_adr];
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
        int qvel_adr = m_->jnt_dofadr[_get_single_dof_joint_handle(jointname)];
        return d_->qvel[qvel_adr];
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
        int qpos_adr = m_->jnt_qposadr[_get_single_dof_joint_handle(jointname)];
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
        int dof_adr = m_->jnt_dofadr[_get_single_dof_joint_handle(jointname)];
        // qfrc_actuator (size nv) contains the forces generated by actuators, indexed by DoF
        return d_->qfrc_actuator[dof_adr];
    }


};
///------------------------------------------------------------------------------------------------

namespace
{
/**
 * @brief check_vector_size throws if the vector does not have one value per joint name.
 */
void check_vector_size(const std::vector<std::string>& jointnames, const VectorXd& values,
                       const std::string& function_name)
{
    if (static_cast<std::size_t>(values.size()) != jointnames.size())
        throw std::invalid_argument("Bad call in DQ_MujocoInterface::" + function_name
                                    + ". Expected a vector of size " + std::to_string(jointnames.size())
                                    + " (one value per joint name), but got a vector of size "
                                    + std::to_string(values.size()) + ".");
}
}


DQ_MujocoInterface::DQ_MujocoInterface(const RENDERING& rendering,
                                       const std::string& window_title,
                                       const int& window_width,
                                       const int& window_height)
{
    impl_ = std::make_unique<DQ_MujocoInterface::Impl>(rendering, window_title, window_width, window_height);
}

// Defined here, where Impl is a complete type (required by std::unique_ptr<Impl>)
DQ_MujocoInterface::~DQ_MujocoInterface() = default;

void DQ_MujocoInterface::_check_model() const
{
    if (!impl_->model_loaded_)
        throw std::runtime_error("Undefined xml model. You must load the model first");
}



void DQ_MujocoInterface::load_model(const std::string &xml_model)
{
    impl_->load_model(xml_model);
}



void DQ_MujocoInterface::trigger_next_simulation_step()
{
    _check_model();
    impl_->step_simulation();
}

double DQ_MujocoInterface::get_simulation_time() const
{
    _check_model();
    return impl_->d_->time;
}

/**
 * @brief Check whether the user asked to close the window
 * @return true if the window was closed. Always false in HEADLESS mode.
 */
bool DQ_MujocoInterface::should_close_window() const
{
    _check_model();
    if (!impl_->window_)
        return false;
    return glfwWindowShouldClose(impl_->window_);
}

/**
 * @brief Draw the current state of the simulation in the window
 *
 * In HEADLESS mode this does nothing, so the same simulation loop works in both modes.
 */
void DQ_MujocoInterface::render()
{
    _check_model();
    if (!impl_->window_)
        return;
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
 * @brief Show or hide the frames of the sites defined in the XML model
 * @param flag true to show the site frames (default), false to hide them
 *
 * Hiding them is useful when drawing reference frames (add_reference_frame()) on top of sites.
 * The site geometries themselves (e.g., spheres) are still drawn.
 */
void DQ_MujocoInterface::set_site_frame_view(const bool &flag)
{
    _check_model();
    impl_->opt_.frame = flag ? mjFRAME_SITE : mjFRAME_NONE;
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
    check_vector_size(jointnames, angles_rad, "set_joint_positions");
    for(std::size_t i=0;i<jointnames.size();i++)
        impl_->_set_joint_position(jointnames.at(i), angles_rad(i));
    // update body/site poses, etc. to match the new joint positions
    mj_forward(impl_->m_, impl_->d_);
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
    check_vector_size(jointnames, angles_rad, "set_joint_target_positions");
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
    check_vector_size(jointnames, velocities_rad_per_sec, "set_joint_target_velocities");
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
    check_vector_size(jointnames, forces, "set_joint_target_forces");
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
 * @brief Add a reference frame that is drawn in the viewer
 * @param name Unique name of the frame. It is also shown as a label at its origin.
 * @param axis_length Length of each axis arrow in meters.
 * @param axis_width Width (radius) of each axis arrow in meters.
 * @param opacity Between 0 (invisible) and 1 (opaque). Use different values to tell frames apart,
 *        e.g., 1.0 for a measured pose and 0.4 for a desired pose.
 * @param show_label Show the name of the frame at its origin.
 *
 * Reference frames do not need to be declared in the XML model, and they are only drawn:
 * they are not part of the simulation (no physics, no collisions) and do not affect it.
 * The frame starts at the world origin; move it with set_reference_frame_pose().
 * The axes are drawn as arrows: x in red, y in green, and z in blue.
 *
 * Example usage:
 * @code
 * mi.add_reference_frame("x_desired", 0.15, 0.005, 0.4);
 * while (simulating) {
 *     mi.set_reference_frame_pose("x_desired", x_desired);
 *     ...
 *     mi.render();
 * }
 * @endcode
 *
 * @note A model does not need to be loaded. In HEADLESS mode the frames are stored but not drawn.
 * @throws std::invalid_argument if the name is empty or already used, or if a size or the
 *         opacity is out of range.
 */
void DQ_MujocoInterface::add_reference_frame(const std::string& name,
                                             const double& axis_length,
                                             const double& axis_width,
                                             const double& opacity,
                                             const bool& show_label)
{
    const std::string function_name = error_msg_layout_ + "add_reference_frame";
    if (name.empty())
        throw std::invalid_argument(function_name + ". The name cannot be empty.");
    if (impl_->reference_frames_.count(name))
        throw std::invalid_argument(function_name + ". The reference frame \"" + name + "\" already exists.");
    if (axis_length <= 0 || axis_width <= 0)
        throw std::invalid_argument(function_name + ". The axis length and width must be positive.");
    if (opacity < 0 || opacity > 1)
        throw std::invalid_argument(function_name + ". The opacity must be between 0 and 1.");

    Impl::ReferenceFrame frame;
    frame.axis_length = axis_length;
    frame.axis_width = axis_width;
    frame.opacity = static_cast<float>(opacity);
    frame.show_label = show_label;
    impl_->reference_frames_.emplace(name, frame);
}

/**
 * @brief Set the pose of a reference frame added with add_reference_frame()
 * @param name Name of the frame.
 * @param pose Unit dual quaternion with the pose of the frame in the world.
 * @throws std::runtime_error if the frame does not exist or the pose is not a unit dual quaternion.
 */
void DQ_MujocoInterface::set_reference_frame_pose(const std::string& name, const DQ& pose)
{
    const std::string function_name = error_msg_layout_ + "set_reference_frame_pose";
    auto it = impl_->reference_frames_.find(name);
    if (it == impl_->reference_frames_.end())
        throw std::runtime_error(function_name + ". The reference frame \"" + name
                                 + "\" does not exist. Add it with add_reference_frame() first.");
    if (!is_unit(pose))
        throw std::runtime_error(function_name + ". The pose must be a unit dual quaternion!");

    Impl::ReferenceFrame& frame = it->second;
    frame.pose = pose;
    VectorXd p = pose.translation().vec3();
    VectorXd r = pose.P().vec4();
    for (int k = 0; k < 3; k++)
        frame.pos[k] = p(k);
    mju_quat2Mat(frame.mat, r.data());
}

/**
 * @brief Get the pose of a reference frame, as last set with set_reference_frame_pose()
 * @param name Name of the frame.
 * @return Unit dual quaternion with the pose of the frame in the world (identity if never set).
 * @throws std::runtime_error if the frame does not exist.
 */
DQ DQ_MujocoInterface::get_reference_frame_pose(const std::string& name) const
{
    auto it = impl_->reference_frames_.find(name);
    if (it == impl_->reference_frames_.end())
        throw std::runtime_error(error_msg_layout_ + "get_reference_frame_pose. The reference frame \""
                                 + name + "\" does not exist.");
    return it->second.pose;
}

/**
 * @brief Remove a reference frame added with add_reference_frame()
 * @param name Name of the frame.
 * @throws std::runtime_error if the frame does not exist.
 */
void DQ_MujocoInterface::remove_reference_frame(const std::string& name)
{
    if (impl_->reference_frames_.erase(name) == 0)
        throw std::runtime_error(error_msg_layout_ + "remove_reference_frame. The reference frame \""
                                 + name + "\" does not exist.");
}


/**
 * @brief Get names of all joints in the loaded model
 * @param only_single_dof If true, return only the named hinge and slide joints, which are the
 *        ones the joint methods (get_joint_positions(), etc.) accept. Use it with floating-base
 *        robots, whose free joint (often unnamed) is accessed with get_base_pose() instead.
 * @return Vector containing the names of the joints
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
std::vector<std::string> DQ_MujocoInterface::get_joint_names(const bool& only_single_dof) const
{
    _check_model();
    return impl_->_get_joint_names(only_single_dof);
}


/**
 * @brief Reset simulation to initial state
 *
 * Resets all simulation state variables (joint positions, velocities, actuator states,
 * contact forces, and simulation time) to the initial conditions defined in the XML model.
 * Site poses modified with set_site_pose() are also restored.
 *
 * @throws std::runtime_error If no model is loaded
 *
 * @warning This instantly teleports the robot without dynamics. Use only for resetting
 *          experiments or recovering from invalid states, not during active control.
 *
 * @see load_model() for loading initial configuration
 * @see set_joint_positions() for teleporting to specific configurations
 */
void DQ_MujocoInterface::reset_simulation() {
    _check_model();
    impl_->reset();
}

/**
 * @brief Reset simulation to a keyframe defined in the XML model
 * @param keyframe Name of the keyframe (<keyframe><key name="..."/></keyframe>)
 *
 * Like reset_simulation(), but the state (qpos, qvel, ctrl, time, etc.) is taken from the
 * keyframe. Floating-base robots usually need it: their initial configuration (qpos0) often
 * places the base inside the floor or the legs straight, and models such as the ones in MuJoCo
 * Menagerie provide a standing keyframe (e.g., "home" or "stand"), whose ctrl also makes the
 * position actuators hold that pose.
 *
 * @throws std::runtime_error If no model is loaded or the keyframe does not exist
 */
void DQ_MujocoInterface::reset_simulation(const std::string& keyframe)
{
    _check_model();
    const int keyframe_id = mj_name2id(impl_->m_, mjOBJ_KEY, keyframe.c_str());
    if (keyframe_id == -1)
        throw std::runtime_error(error_msg_layout_ + "reset_simulation. The keyframe \"" + keyframe
                                 + "\" does not exist in the XML model.");
    impl_->reset(keyframe_id);
}


/**
 * @brief Get the names of the floating bases in the model
 * @return Names of the bodies that have a free joint (e.g., {"pelvis"} for a humanoid). Unnamed
 *         bodies are skipped.
 *
 * Use these names with get_base_pose(), set_base_pose(), get_base_twist() and set_base_twist().
 */
std::vector<std::string> DQ_MujocoInterface::get_floating_base_names() const
{
    _check_model();
    return impl_->_get_floating_base_names();
}

/**
 * @brief Get the pose of a floating base
 * @param bodyname Name of the body that owns the free joint (see get_floating_base_names()).
 * @return Unit dual quaternion with the pose of the body frame in the world.
 *
 * Together with get_joint_positions() for the limbs, this is the full configuration of a
 * floating-base robot. For instance, use it as the reference frame of the DQ_SerialManipulator
 * of each leg.
 *
 * @note It is read from the state (qpos), so it reflects the latest simulation step.
 * @throws std::runtime_error If the body does not exist or has no free joint
 */
DQ DQ_MujocoInterface::get_base_pose(const std::string& bodyname) const
{
    _check_model();
    return impl_->_get_base_pose(bodyname);
}

/**
 * @brief Teleport a floating base (no dynamics)
 * @param bodyname Name of the body that owns the free joint.
 * @param pose Unit dual quaternion with the desired pose of the body frame in the world.
 *
 * Like set_joint_positions(), this writes the state directly. The velocity of the base is kept;
 * set it with set_base_twist() (e.g., to zero) if needed.
 *
 * @throws std::runtime_error If the body does not exist or has no free joint, or the pose is
 *         not a unit dual quaternion
 */
void DQ_MujocoInterface::set_base_pose(const std::string& bodyname, const DQ& pose)
{
    if (!is_unit(pose))
        throw std::runtime_error(error_msg_layout_ + "set_base_pose. The pose must be a unit dual quaternion!");
    _check_model();
    impl_->_set_base_pose(bodyname, pose);
    // update body/site poses, etc. to match the new base pose
    mj_forward(impl_->m_, impl_->d_);
}

/**
 * @brief Get the twist of a floating base
 * @param bodyname Name of the body that owns the free joint.
 * @param reference_frame Frame in which the twist is expressed: WORLD_FRAME (default) or BODY_FRAME.
 * @return Pure dual quaternion with the twist of the body.
 *
 * Let x = r + 0.5*E_*p*r be the pose returned by get_base_pose(), w the angular velocity of the
 * body and p_dot the linear velocity of the body origin p.
 *
 * WORLD_FRAME: the twist is expressed in the world (inertial) frame, and its reference point is
 * the world origin:
 *
 *     xi = w + E_*(p_dot + cross(p, w)),    x_dot = 0.5*xi*x,
 *
 *   - primary part, w: angular velocity of the body, in the world frame.
 *   - dual part, p_dot + cross(p, w): linear velocity of the point of the body (extended as a
 *     rigid body) that is at the world origin, in the world frame. It is NOT the velocity of the
 *     body origin, p_dot, unless w = 0 or p = 0. Recover p_dot with
 *         p_dot = xi.D() - cross(x.translation(), xi.P())
 *
 * BODY_FRAME: the twist is expressed in the body frame, and its reference point is the body
 * origin:
 *
 *     xi_body = w_body + E_*p_dot_body = Ad(conj(x), xi),    x_dot = 0.5*x*xi_body,
 *
 *   - primary part, w_body = conj(r)*w*r: angular velocity of the body, in the body frame.
 *   - dual part, p_dot_body = conj(r)*p_dot*r: linear velocity of the body origin, in the body frame.
 *
 * @note MuJoCo stores the velocity of a free joint (qvel) in a third, mixed form: the linear
 *       velocity of the body origin in the world frame, followed by the angular velocity in the
 *       body frame. This method converts it, so qvel never needs to be handled directly.
 *
 * @see set_base_twist()
 * @throws std::runtime_error If the body does not exist or has no free joint
 */
DQ DQ_MujocoInterface::get_base_twist(const std::string& bodyname, const REFERENCE_FRAME& reference_frame) const
{
    _check_model();
    return impl_->_get_base_twist(bodyname, reference_frame);
}

/**
 * @brief Set the twist of a floating base (no dynamics)
 * @param bodyname Name of the body that owns the free joint.
 * @param twist Pure dual quaternion with the desired twist, expressed in reference_frame with the
 *        same convention as get_base_twist().
 * @param reference_frame Frame in which the twist is expressed: WORLD_FRAME (default) or BODY_FRAME.
 *
 * Build the twist from the velocities you want with
 *
 *     // WORLD_FRAME: w and p_dot (velocity of the body origin p) in the world frame
 *     const DQ p = get_base_pose(bodyname).translation();
 *     set_base_twist(bodyname, w + E_*(p_dot + cross(p, w)));
 *
 *     // BODY_FRAME: w_body and p_dot_body (velocity of the body origin) in the body frame
 *     set_base_twist(bodyname, w_body + E_*p_dot_body, REFERENCE_FRAME::BODY_FRAME);
 *
 * In the world frame, the cross(p, w) term matters: xi = w alone (no dual part) rotates the body
 * about the world origin, so its origin moves on a circle. To spin the body in place, use
 * xi = w + E_*cross(p, w), or w_body alone in the body frame.
 *
 * Like set_base_pose(), it writes the state directly and bypasses the dynamics. The twist then
 * evolves with the physics (gravity, contacts, ...) during the next steps.
 *
 * @see get_base_twist() and the twist_open_loop example
 * @throws std::runtime_error If the body does not exist or has no free joint, or the twist is
 *         not a pure dual quaternion
 */
void DQ_MujocoInterface::set_base_twist(const std::string& bodyname, const DQ& twist,
                                        const REFERENCE_FRAME& reference_frame)
{
    if (!is_pure(twist))
        throw std::runtime_error(error_msg_layout_ + "set_base_twist. The twist must be a pure dual quaternion!");
    _check_model();
    impl_->_set_base_twist(bodyname, twist, reference_frame);
    // update velocity-dependent quantities
    mj_forward(impl_->m_, impl_->d_);
}

/**
 * @brief Get the pose of a body frame
 * @param bodyname Name of the body (e.g., a foot, the trunk or a hand).
 * @return Unit dual quaternion with the pose of the body frame in the world.
 *
 * @note Like get_site_pose(), it is computed at the beginning of each simulation step, so after
 *       trigger_next_simulation_step() it corresponds to the state before that step.
 * @throws std::runtime_error If the body does not exist
 */
DQ DQ_MujocoInterface::get_body_pose(const std::string& bodyname) const
{
    _check_model();
    return impl_->_get_body_pose(bodyname);
}

/**
 * @brief Make the viewer camera follow a body
 * @param bodyname Name of the body to follow (e.g., the floating base). An empty name releases
 *        the camera, which then stays where it is.
 *
 * The camera looks at the center of mass of the body and its children, so a walking robot stays
 * in view. The mouse still rotates and zooms the camera. It does nothing in HEADLESS mode, and it
 * is cleared by load_model().
 *
 * @throws std::runtime_error If the body does not exist
 */
void DQ_MujocoInterface::set_camera_tracking(const std::string& bodyname)
{
    _check_model();
    mjvCamera& cam = impl_->cam_;
    if (bodyname.empty())
    {
        cam.type = mjCAMERA_FREE;
        cam.trackbodyid = -1;
        return;
    }
    cam.trackbodyid = impl_->_get_body_handle(bodyname);
    cam.type = mjCAMERA_TRACKING;
    cam.fixedcamid = -1;
}

} // namespace DQ_robotics_extensions
