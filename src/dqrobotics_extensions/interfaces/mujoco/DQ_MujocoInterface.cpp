/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2025-2026 Juan Jose Quiroz Omana

This file is part of cpp-interface-mujoco, an unofficial interface for C++17 between DQ Robotics
and MuJoCo: https://github.com/juanjqo/cpp-interface-mujoco
It uses DQ Robotics (https://dqrobotics.github.io) but is not part of DQ Robotics.

The mouse and scroll callbacks are adapted from MuJoCo's sample/basic.cc,
Copyright 2021 DeepMind Technologies Limited, licensed under the Apache License, Version 2.0.
*/

#include <chrono>
#include <dqrobotics_extensions/interfaces/mujoco/DQ_MujocoInterface.h>
#include "DQ_MujocoSceneBuilderImpl.h"
#include "DQ_MujocoCameras.h"
#ifdef DQ_MUJOCO_HAS_GUI
#include "DQ_MujocoGui.h"
#endif
#include <set>
#include <unordered_map>
#include <mutex>
#include <algorithm>
#include <atomic>
#include <cstring>
#include <limits>
#include <iostream>
#include <map>
#include <thread>
#include "mujoco/mujoco.h"
#include "GLFW/glfw3.h"
#include <marinholab/sas/core/sas_thread_manager.hpp>


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
    // The mouse and scroll handling below is adapted from MuJoCo's sample/basic.cc (Apache License 2.0,
    // see the notice at the top of this file).

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

#ifdef DQ_MUJOCO_HAS_GUI
    static void key_callback(GLFWwindow* window, int key, [[maybe_unused]] int scancode, int action, int mods) {
        Impl* self = static_cast<Impl*>(glfwGetWindowUserPointer(window));
        if (self && self->_gui_active())
            self->gui_->on_key(key, mods, action != GLFW_RELEASE);
    }

    static void char_callback(GLFWwindow* window, unsigned int codepoint) {
        Impl* self = static_cast<Impl*>(glfwGetWindowUserPointer(window));
        if (self && self->_gui_active())
            self->gui_->on_char(codepoint);
    }

    static void focus_callback(GLFWwindow* window, int focused) {
        Impl* self = static_cast<Impl*>(glfwGetWindowUserPointer(window));
        if (self && self->_gui_active())
            self->gui_->on_focus(focused == GLFW_TRUE);
    }
#endif

    // Mouse button handler
    void mouse_button(GLFWwindow* window, [[maybe_unused]] int button, [[maybe_unused]] int action, [[maybe_unused]] int mods) {
#ifdef DQ_MUJOCO_HAS_GUI
        if (_gui_active())
        {
            gui_->on_mouse_button(button, action == GLFW_PRESS, mods);
            // A click on a panel is for the GUI, and Ctrl + drag drags the selected body (see
            // _perturb()): not for the camera
            if (action == GLFW_PRESS && (gui_->wants_mouse() || ((mods & GLFW_MOD_CONTROL) && gui_->has_selection())))
                return;
        }
#endif
        button_left_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
        button_middle_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
        button_right_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
        glfwGetCursorPos(window, &lastx_, &lasty_);
    }

    // Mouse move handler
    void mouse_move(GLFWwindow* window, double xpos, double ypos) {
#ifdef DQ_MUJOCO_HAS_GUI
        if (_gui_active())
            gui_->on_mouse_move(xpos, ypos);
#endif
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
        std::lock_guard<std::mutex> lock(viewer_mutex_);
        mjv_moveCamera(m_, action, dx / height, dy / height, &cam_);
    }

    // Scroll handler
    void scroll([[maybe_unused]] GLFWwindow* window, double yoffset) {
#ifdef DQ_MUJOCO_HAS_GUI
        if (_gui_active())
        {
            gui_->on_scroll(0, yoffset);
            if (gui_->wants_mouse())
                return;  // scrolls a panel
        }
#endif
        std::lock_guard<std::mutex> lock(viewer_mutex_);
        mjv_moveCamera(m_, mjMOUSE_ZOOM, 0, 0.05 * yoffset, &cam_);
    }

public:
    std::unordered_map<std::string, int> handles_map_; //
    std::unordered_map<std::string, int> actuator_map_; // actuator handles
    std::mutex lookup_mutex_;  // protects the handle caches above

    //----------------------------------------------------------------------------------------------
    // Threads
    //
    // - Viewer thread (WINDOW mode, from load_model() on): draws the scene at 60 Hz. The window and
    //   its events stay on the main thread (GLFW requirement); only the OpenGL context is moved to
    //   the viewer thread, which is allowed. The main thread processes the events in
    //   should_close_window().
    // - Physics thread (between start_simulation() and stop_simulation()): steps the simulation in
    //   real time. Getters read a snapshot of the state published after each step, and the actuator
    //   commands go to a buffer applied before each step, so the user never waits for mj_step().
    //
    // Locks, always taken in this order: viewer_mutex_ -> physics_mutex_ -> {state_mutex_,
    // command_mutex_}. reference_frames_mutex_ and lookup_mutex_ are leaves: no other lock is taken
    // while holding them.
    //----------------------------------------------------------------------------------------------
    using ThreadManager = marinholab::sas::core::ThreadManager;
    using steady_clock = std::chrono::steady_clock;

    std::mutex physics_mutex_;  // protects d_ and the mutable parts of m_ (site poses)
    std::mutex viewer_mutex_;   // protects cam_, opt_, scn_

    // Physics thread
    std::unique_ptr<ThreadManager> physics_thread_;
    std::atomic<bool> simulation_running_{false};
    // The program steps the simulation with trigger_next_simulation_step() (until it calls
    // start_simulation()): the GUI must not start the real-time simulation, which would make the
    // program's next step throw
    std::atomic<bool> stepped_by_program_{false};
    double anchor_simulation_time_{0};          // simulation time and wall-clock time from which
    steady_clock::time_point anchor_wall_time_;  // the physics thread keeps real time
    std::atomic<int> max_steps_per_tick_{1};
    std::atomic<double> real_time_factor_{0};
    double rtf_window_simulated_{0};             // simulated time and start of the current
    steady_clock::time_point rtf_window_start_;  // real-time factor measurement window

    // State published by the physics thread after each step, read by the getters while running
    mjData* snapshot_ = nullptr;
    std::mutex state_mutex_;

    // Actuator commands written while running, applied by the physics thread before each step
    std::vector<mjtNum> command_ctrl_;
    std::vector<char> command_pending_;
    std::mutex command_mutex_;

    // Viewer thread
    std::unique_ptr<ThreadManager> viewer_thread_;
    std::atomic<bool> viewer_release_requested_{false};
    std::atomic<bool> viewer_released_{false};
    bool viewer_has_context_{false};  // only used by the viewer thread
    std::atomic<int> framebuffer_width_{0};
    std::atomic<int> framebuffer_height_{0};
    std::atomic<int> screen_width_{0};   // size of the window in screen coordinates (for the GUI)
    std::atomic<int> screen_height_{0};
    steady_clock::time_point last_event_poll_;

    // GUI of the viewer (set_gui_enabled()). gui_ is created by the main thread, under viewer_mutex_,
    // and drawn by the viewer thread. Its requests (play, pause, step, reset) are run by the main
    // thread in poll_events(), like calls of the user.
    std::atomic<bool> gui_enabled_{false};
#ifdef DQ_MUJOCO_HAS_GUI
    std::unique_ptr<MujocoGui> gui_;
    MujocoGui::State gui_state_;  // only used by the viewer thread
#endif
    // Perturbation of a body with the mouse (Ctrl + drag in the GUI), protected by physics_mutex_:
    // set by the viewer thread, applied before each step
    mjvPerturb pert_;
    bool perturb_force_applied_{false};
#ifdef DQ_MUJOCO_HAS_GUI
    // The body dragged with the gizmo (GUI), held at its pose before each step. Protected by
    // physics_mutex_, like the fixed bodies moved with it (by name), which the main thread writes
    // in the specification so that edit_scene() keeps them.
    struct GizmoHold
    {
        bool active = false;
        int body = -1;
        MujocoGui::Movable movable = MujocoGui::Movable::NONE;
        mjtNum pos[3] = {0, 0, 0};
        mjtNum quat[4] = {1, 0, 0, 0};
    };
    GizmoHold gizmo_hold_;
    std::vector<std::string> gizmo_moved_fixed_bodies_;
#endif

    // Camera thread (from the first start_camera() on): renders the cameras offscreen, with its own
    // OpenGL context. Its lock order: it takes physics_mutex_ to copy the state.
    std::unique_ptr<CameraRenderer> cameras_;

public:
    std::string xml_model_;
    bool model_loaded_{false};
    mjModel* m_ = nullptr;  // MuJoCo model
    mjData* d_ = nullptr;   // MuJoCo data

    // The specification compiled into m_, kept for edit_scene() (mj_recompile() needs the same
    // specification to keep the state), and the virtual file system of models loaded from memory
    struct VFSDeleter
    {
        void operator()(mjVFS* vfs) const { mj_deleteVFS(vfs); delete vfs; }
    };
    using VFSPtr = std::unique_ptr<mjVFS, VFSDeleter>;
    mjSpec* spec_ = nullptr;
    VFSPtr vfs_;
    std::string camera_tracking_body_;  // re-resolved after edits, since body ids can change

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
    std::atomic<double> avg_computation_time_microseconds_{50.0};
    double computation_time_samples_ = 1000;

    // Reference frames added at runtime. They are drawn as decorations by the viewer and do not
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
    std::mutex reference_frames_mutex_;  // the user changes the frames while the viewer draws them
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
        mjv_defaultPerturb(&pert_);
    };

    /**
     * @brief ~Impl() Destructor of the class
     */
    ~Impl(){
        // stop the threads before anything they use is freed
        _stop_simulation();
        physics_thread_.reset();
        cameras_.reset();

        // free visualization storage while the window (and its OpenGL context) still exists
        _free_visualization();
#ifdef DQ_MUJOCO_HAS_GUI
        gui_.reset();  // the OpenGL context is current in this thread (_free_visualization())
#endif

        // close the window, and GLFW if this is the last interface using it
        if (window_) glfwDestroyWindow(window_);
        _release_glfw();

        //free MuJoCo model and data
        if (snapshot_) mj_deleteData(snapshot_);
        if (d_) mj_deleteData(d_);
        if (m_) mj_deleteModel(m_);
        if (spec_) mj_deleteSpec(spec_);
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
        _stop_viewer();
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
        // parse and compile the model, and check for errors
        char error[1000] = "";
        mjSpec* spec = mj_parseXML(xml_model.c_str(), nullptr, error, sizeof(error));
        if (!spec)
            throw std::runtime_error("Failed to load the xml model \"" + xml_model + "\": " + error);
        mjModel* m = mj_compile(spec, nullptr);
        if (!m)
        {
            const std::string message = mjs_getError(spec);
            mj_deleteSpec(spec);
            throw std::runtime_error("Failed to load the xml model \"" + xml_model + "\": " + message);
        }
        _set_model(m, spec, nullptr, xml_model);
    }

    /**
     * @brief load_model loads a model held in memory. The XML and its assets are added to a
     *        MuJoCo virtual file system, so no file is read from disk.
     */
    void load_model(const DQ_MujocoModel& model)
    {
        if (model.name.empty())
            throw std::invalid_argument("The name of the model cannot be empty");
        if (model.xml.empty())
            throw std::invalid_argument("The xml of the model \"" + model.name + "\" cannot be empty");

        // The virtual file system is kept with the model: edit_scene() compiles it again
        VFSPtr vfs_holder(new mjVFS);
        mjVFS& vfs = *vfs_holder;
        mj_defaultVFS(&vfs);

        auto add_buffer = [&](const std::string& name, const void* buffer, std::size_t size)
        {
            if (size > static_cast<std::size_t>(std::numeric_limits<int>::max()))
                throw std::invalid_argument("The file \"" + name + "\" of the model \"" + model.name
                                            + "\" is too large");
            const int result = mj_addBufferVFS(&vfs, name.c_str(), buffer, static_cast<int>(size));
            if (result == 2)
                throw std::invalid_argument("The file \"" + name + "\" of the model \"" + model.name
                                            + "\" is repeated (an asset has the name of the model?)");
            if (result != 0)
                throw std::runtime_error("Failed to add the file \"" + name + "\" of the model \""
                                         + model.name + "\" to the virtual file system");
        };
        add_buffer(model.name, model.xml.data(), model.xml.size());
        for (const auto& [name, content] : model.assets)
            add_buffer(name, content.data(), content.size());

        char error[1000] = "";
        mjSpec* spec = mj_parseXML(model.name.c_str(), &vfs, error, sizeof(error));
        if (!spec)
            throw std::runtime_error("Failed to load the model \"" + model.name + "\": " + error);
        mjModel* m = mj_compile(spec, &vfs);
        if (!m)
        {
            const std::string message = mjs_getError(spec);
            mj_deleteSpec(spec);
            throw std::runtime_error("Failed to load the model \"" + model.name + "\": " + message);
        }
        _set_model(m, spec, std::move(vfs_holder), model.name);
    }

    /**
     * @brief _set_model replaces the current model by m, compiled from spec (with vfs, for models
     *        loaded from memory), creates its data and, in WINDOW mode, prepares the window to
     *        render it. This object then owns m, spec and vfs.
     * @param xml_model Path or name of the model, used in error messages.
     */
    void _set_model(mjModel* m, mjSpec* spec, VFSPtr vfs, const std::string& xml_model)
    {
         // make data corresponding to model
        mjData* d = mj_makeData(m);
        mjData* snapshot = d ? mj_makeData(m) : nullptr;
        if (!d || !snapshot)
        {
            if (d) mj_deleteData(d);
            mj_deleteModel(m);
            mj_deleteSpec(spec);
            throw std::runtime_error("Failed to allocate the data for the xml model \"" + xml_model + "\"");
        }

        // The new model is valid: stop the threads and release the previous one, if any (the
        // cameras of the previous model are stopped)
        _stop_simulation();
        physics_thread_.reset();  // its period depends on the timestep of the model
        cameras_.reset();
        model_loaded_ = false;
        _free_visualization();
        if (snapshot_) mj_deleteData(snapshot_);
        if (d_) mj_deleteData(d_);
        if (m_) mj_deleteModel(m_);
        if (spec_) mj_deleteSpec(spec_);
        {
            std::lock_guard<std::mutex> lock(lookup_mutex_);
            handles_map_.clear();
            actuator_map_.clear();
        }

        m_ = m;
        d_ = d;
        snapshot_ = snapshot;
        mjv_defaultPerturb(&pert_);  // the bodies of the previous model
        perturb_force_applied_ = false;
#ifdef DQ_MUJOCO_HAS_GUI
        gizmo_hold_ = GizmoHold();
        gizmo_moved_fixed_bodies_.clear();
#endif
        spec_ = spec;
        vfs_ = std::move(vfs);
        camera_tracking_body_.clear();
        command_ctrl_.assign(m_->nu, 0);
        command_pending_.assign(m_->nu, 0);
        // Catch up at most 50 ms of simulation per wake-up of the physics thread
        max_steps_per_tick_ = std::max(1, static_cast<int>(0.05 / m_->opt.timestep));
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
        modify_state([&]() {
            if (keyframe_id < 0)
                mj_resetData(m_, d_);
            else
                mj_resetDataKeyframe(m_, d_, keyframe_id);
            std::copy(site_pos0_.begin(), site_pos0_.end(), m_->site_pos);
            std::copy(site_quat0_.begin(), site_quat0_.end(), m_->site_quat);
            std::copy(site_sameframe0_.begin(), site_sameframe0_.end(), m_->site_sameframe);
            // update body/site poses, etc. to match the reset state
            mj_forward(m_, d_);
            // The reset sets the time and the controls: keep real time from the new time, and drop
            // the commands written before the reset
            _anchor_real_time();
            std::lock_guard<std::mutex> lock(command_mutex_);
            std::fill(command_pending_.begin(), command_pending_.end(), 0);
        });
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

            // Set callbacks using static wrapper functions
            glfwSetMouseButtonCallback(window_, &Impl::mouse_button_callback);
            glfwSetCursorPosCallback(window_, &Impl::mouse_move_callback);
            glfwSetScrollCallback(window_, &Impl::scroll_callback);
#ifdef DQ_MUJOCO_HAS_GUI
            glfwSetKeyCallback(window_, &Impl::key_callback);
            glfwSetCharCallback(window_, &Impl::char_callback);
            glfwSetWindowFocusCallback(window_, &Impl::focus_callback);
#endif
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
        _apply_visual_settings();

        int framebuffer_width, framebuffer_height;
        glfwGetFramebufferSize(window_, &framebuffer_width, &framebuffer_height);
        framebuffer_width_ = framebuffer_width;
        framebuffer_height_ = framebuffer_height;
        int screen_width, screen_height;
        glfwGetWindowSize(window_, &screen_width, &screen_height);
        screen_width_ = screen_width;
        screen_height_ = screen_height;
#ifdef DQ_MUJOCO_HAS_GUI
        if (gui_enabled_)
            _create_gui();
#endif

        last_time_ = std::chrono::high_resolution_clock::now();
        _start_viewer();
    }

    /**
     * @brief _apply_visual_settings sets the visual settings of the interface in the model
     *        (needed again after mj_recompile(), which takes them from the specification).
     */
    void _apply_visual_settings()
    {
        //// bodyframe axis width and length
        m_->vis.scale.framewidth = 0.05;
        m_->vis.scale.framelength = 1;
    }

    /**
     * @brief recompile applies the edits of the specification to the model, keeping the state
     *        (mj_recompile()). Elements that did not exist before start at their "home"
     *        configuration, and the site poses set with set_site_pose() are kept.
     *
     * The threads can keep running: the physics thread and the viewer wait for the locks (a few
     * milliseconds), and the getters read the snapshot until the new one is published.
     * @param rebuild_gpu_context True if the edit added or removed GPU resources (textures,
     *        meshes, ...): the viewer must be stopped, since the context is rebuilt in this thread.
     */
    void recompile(const DQ_MujocoSceneBuilder::Impl& scene, const bool& rebuild_gpu_context)
    {
        auto names_of = [this](const mjtObj& type, const int& count) {
            std::set<std::string> names;
            for (int i = 0; i < count; i++)
                if (const char* name = mj_id2name(m_, type, i))
                    names.insert(name);
            return names;
        };
        const std::set<std::string> old_joints = names_of(mjOBJ_JOINT, m_->njnt);
        const std::set<std::string> old_bodies = names_of(mjOBJ_BODY, m_->nbody);
        const std::set<std::string> old_actuators = names_of(mjOBJ_ACTUATOR, m_->nu);
        struct SitePose { std::vector<mjtNum> pos, quat; mjtByte sameframe; };
        std::map<std::string, SitePose> site_poses;
        for (int i = 0; i < m_->nsite; i++)
            if (const char* name = mj_id2name(m_, mjOBJ_SITE, i))
                site_poses[name] = {{m_->site_pos + 3 * i, m_->site_pos + 3 * i + 3},
                                    {m_->site_quat + 4 * i, m_->site_quat + 4 * i + 4},
                                    m_->site_sameframe[i]};
        const double old_timestep = m_->opt.timestep;

        // Lock order: viewer_mutex_ before physics_mutex_
        std::unique_lock<std::mutex> viewer_lock(viewer_mutex_, std::defer_lock);
        if (window_)
            viewer_lock.lock();
        std::lock_guard<std::mutex> lock(physics_mutex_);
        // First the structure, which also creates the pending keyframes, then the keyframes (same
        // layout). The changes were checked on a copy, so these do not fail.
        if (mj_recompile(spec_, vfs_.get(), m_, d_) != 0)
            throw std::runtime_error(std::string("Failed to apply the changes to the scene: ")
                                     + mjs_getError(spec_));
        scene.fix_keyframes(spec_, m_);
        if (mj_recompile(spec_, vfs_.get(), m_, d_) != 0)
            throw std::runtime_error(std::string("Failed to apply the changes to the scene: ")
                                     + mjs_getError(spec_));
        _apply_visual_settings();
        // The body ids can change: stop dragging any body
        mjv_defaultPerturb(&pert_);
        mju_zero(d_->xfrc_applied, 6 * m_->nbody);
        perturb_force_applied_ = false;
#ifdef DQ_MUJOCO_HAS_GUI
        gizmo_hold_ = GizmoHold();
#endif

        // The sizes of the model changed
        {
            std::lock_guard<std::mutex> state_lock(state_mutex_);
            mj_deleteData(snapshot_);
            snapshot_ = mj_makeData(m_);
        }
        {
            std::lock_guard<std::mutex> command_lock(command_mutex_);
            command_ctrl_.assign(m_->nu, 0);
            command_pending_.assign(m_->nu, 0);
        }
        max_steps_per_tick_ = std::max(1, static_cast<int>(0.05 / m_->opt.timestep));
        if (m_->opt.timestep != old_timestep)
            physics_thread_.reset();  // its period is the timestep (edit_scene() stopped it)
        {
            std::lock_guard<std::mutex> lookup_lock(lookup_mutex_);
            handles_map_.clear();
            actuator_map_.clear();
        }

        // Reset values of the sites, and the poses set with set_site_pose()
        site_pos0_.assign(m_->site_pos, m_->site_pos + 3 * m_->nsite);
        site_quat0_.assign(m_->site_quat, m_->site_quat + 4 * m_->nsite);
        site_sameframe0_.assign(m_->site_sameframe, m_->site_sameframe + m_->nsite);
        for (int i = 0; i < m_->nsite; i++)
        {
            const char* name = mj_id2name(m_, mjOBJ_SITE, i);
            auto it = name ? site_poses.find(name) : site_poses.end();
            if (it == site_poses.end())
                continue;
            std::copy(it->second.pos.begin(), it->second.pos.end(), m_->site_pos + 3 * i);
            std::copy(it->second.quat.begin(), it->second.quat.end(), m_->site_quat + 4 * i);
            m_->site_sameframe[i] = it->second.sameframe;
        }

        // New elements start at their home configuration (new objects are already at their pose)
        const int home = mj_name2id(m_, mjOBJ_KEY, DQ_MujocoSceneBuilder::Impl::home_keyframe_name_);
        if (home >= 0)
        {
            const mjtNum* key_qpos = m_->key_qpos + home * m_->nq;
            for (int j = 0; j < m_->njnt; j++)
            {
                const char* joint_name = mj_id2name(m_, mjOBJ_JOINT, j);
                const char* body_name = mj_id2name(m_, mjOBJ_BODY, m_->jnt_bodyid[j]);
                const bool is_new = m_->jnt_type[j] == mjJNT_FREE
                                    ? (body_name && !old_bodies.count(body_name))
                                    : (joint_name && !old_joints.count(joint_name));
                if (!is_new)
                    continue;
                const int adr = m_->jnt_qposadr[j];
                const int size = m_->jnt_type[j] == mjJNT_FREE ? 7 : (m_->jnt_type[j] == mjJNT_BALL ? 4 : 1);
                std::copy(key_qpos + adr, key_qpos + adr + size, d_->qpos + adr);
            }
            for (int a = 0; a < m_->nu; a++)
            {
                const char* actuator_name = mj_id2name(m_, mjOBJ_ACTUATOR, a);
                if (actuator_name && !old_actuators.count(actuator_name))
                    d_->ctrl[a] = m_->key_ctrl[home * m_->nu + a];
            }
        }
        mj_forward(m_, d_);
        if (simulation_running_)
            _publish_state();

        if (window_)
        {
            // The GPU context holds the textures, meshes, etc. of the model
            if (rebuild_gpu_context)
            {
                mjr_freeContext(&con_);
                mjv_freeScene(&scn_);
                mjv_defaultScene(&scn_);
                mjr_defaultContext(&con_);
                mjv_makeScene(m_, &scn_, 2000);
                mjr_makeContext(m_, &con_, mjFONTSCALE_150);
            }
            const int body_id = camera_tracking_body_.empty()
                                ? -1 : mj_name2id(m_, mjOBJ_BODY, camera_tracking_body_.c_str());
            if (cam_.type == mjCAMERA_TRACKING)
            {
                if (body_id >= 0)
                    cam_.trackbodyid = body_id;
                else
                {
                    cam_.type = mjCAMERA_FREE;
                    cam_.trackbodyid = -1;
                    camera_tracking_body_.clear();
                }
            }
        }
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

    //----------------------------------------------------------------------------------------------
    // Viewer thread
    //----------------------------------------------------------------------------------------------

    /**
     * @brief _viewer_tick draws one frame. It runs in the viewer thread, at 60 Hz.
     *
     * Only the copy of the simulation state into the scene (mjv_updateScene, a few microseconds)
     * holds the physics lock; the drawing does not, so it never delays the physics or the user.
     */
    void _viewer_tick()
    {
        // _stop_viewer() asks the thread to release the OpenGL context before stopping it
        if (viewer_release_requested_)
        {
            if (viewer_has_context_)
            {
                glfwMakeContextCurrent(nullptr);
                viewer_has_context_ = false;
            }
            viewer_released_ = true;
            return;
        }
        if (!viewer_has_context_)
        {
            glfwMakeContextCurrent(window_);  // allowed from any thread
            glfwSwapInterval(0);              // the thread is paced by its clock, not by vsync
            viewer_has_context_ = true;
        }

        // Copy the reference frames, so that the user is not blocked while they are drawn
        std::vector<std::pair<std::string, ReferenceFrame>> frames;
        {
            std::lock_guard<std::mutex> lock(reference_frames_mutex_);
            frames.assign(reference_frames_.begin(), reference_frames_.end());
        }

        {
            std::lock_guard<std::mutex> viewer_lock(viewer_mutex_);
#ifdef DQ_MUJOCO_HAS_GUI
            MujocoGui* gui = gui_enabled_ ? gui_.get() : nullptr;
#endif
            double simulation_time;
            {
                std::lock_guard<std::mutex> physics_lock(physics_mutex_);
                mjv_updateScene(m_, d_, &opt_, &pert_, &cam_, mjCAT_ALL, &scn_);
                simulation_time = d_->time;
#ifdef DQ_MUJOCO_HAS_GUI
                if (gui)
                    gui_state_.copy(m_, d_);  // the panels are drawn without the physics lock
#endif
            }
            for (const auto& [name, frame] : frames)
                _draw_reference_frame(name, frame);
#ifdef DQ_MUJOCO_HAS_GUI
            if (gui)
                gui->highlight_selection(m_, scn_);
#endif

            mjrRect viewport = {0, 0, framebuffer_width_.load(), framebuffer_height_.load()};
            bool gui_drawn = false;
#ifdef DQ_MUJOCO_HAS_GUI
            if (gui)
            {
                // The GUI fills the window and leaves the 3D view in the middle. The statistics
                // replace the text overlay.
                const MujocoGui::Statistics statistics{simulation_running_.load(), stepped_by_program_.load(),
                                                       real_time_factor_.load(), fps_,
                                                       avg_computation_time_microseconds_.load()};
                viewport = gui->begin_frame({m_, gui_state_, opt_, scn_, cam_, camera_tracking_body_, statistics,
                                             screen_width_.load(), screen_height_.load(),
                                             viewport.width, viewport.height});
                // A click in the 3D view selects the body under the cursor, in the scene as drawn
                MujocoGui::Pick pick;
                if (gui->take_pick(pick))
                {
                    // MuJoCo documents geomid, flexid and skinid as nullable, but mjv_select()
                    // writes them unconditionally
                    mjtNum point[3] = {0, 0, 0};
                    int geom_id = -1, flex_id = -1, skin_id = -1;
                    std::lock_guard<std::mutex> physics_lock(physics_mutex_);
                    const int body = mjv_select(m_, d_, &opt_, pick.aspect_ratio, pick.relx, pick.rely,
                                                &scn_, point, &geom_id, &flex_id, &skin_id);
                    // The clicked point in the body frame: where a drag holds the body
                    mjtNum local_point[3] = {0, 0, 0};
                    if (body > 0)
                    {
                        mjtNum offset[3];
                        mju_sub3(offset, point, d_->xpos + 3 * body);
                        mju_mulMatTVec3(local_point, d_->xmat + 9 * body, offset);
                    }
                    gui->select_body(body, local_point);
                }
                _perturb(gui->perturb_input());
                _gizmo(gui->gizmo_input());
                mjr_render(viewport, &scn_, &con_);
                gui->render();
                gui_drawn = true;
            }
            else
#endif
            mjr_render(viewport, &scn_, &con_);

            if (show_profiler_ && !gui_drawn)
            {
                std::string text = "t: " + std::to_string(simulation_time) + "s";
                mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, {viewport.width - 310, viewport.height - 210, 300, 200},
                            text.c_str(), nullptr, &con_);
                text = "FPS: " + std::to_string(fps_);
                mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, {viewport.width - 310, viewport.height - 250, 300, 200},
                            text.c_str(), nullptr, &con_);
                text = "ct: " + std::to_string(avg_computation_time_microseconds_.load()) + " us";
                mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, {viewport.width - 310, viewport.height - 290, 300, 200},
                            text.c_str(), nullptr, &con_);
                if (simulation_running_)
                {
                    text = "RTF: " + std::to_string(real_time_factor_.load());
                    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPRIGHT, {viewport.width - 310, viewport.height - 330, 300, 200},
                                text.c_str(), nullptr, &con_);
                }
            }
        }

        glfwSwapBuffers(window_);  // allowed from any thread
        update_fps();
    }

    /**
     * @brief _start_viewer moves the OpenGL context to a new viewer thread. Main thread only.
     */
    void _start_viewer()
    {
        if (!window_ || viewer_thread_)
            return;
        glfwMakeContextCurrent(nullptr);  // a context can only be current in one thread
        viewer_release_requested_ = false;
        viewer_released_ = false;
        viewer_thread_ = std::make_unique<ThreadManager>("mujoco_viewer", 1.0 / 60.0,
                                                         [this]() { _viewer_tick(); },
                                                         ThreadManager::PRIORITY::BACKGROUND);
        viewer_thread_->start();
    }

    /**
     * @brief _stop_viewer stops the viewer thread and makes the OpenGL context current in the
     *        calling (main) thread again, e.g., to free it or to make the scene of a new model.
     */
    void _stop_viewer()
    {
        if (!viewer_thread_)
            return;
        // The context must be released by the thread that holds it
        viewer_release_requested_ = true;
        const auto deadline = steady_clock::now() + std::chrono::seconds(2);
        while (!viewer_released_ && steady_clock::now() < deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (!viewer_released_)
            std::cerr << "DQ_MujocoInterface: the viewer thread did not release the OpenGL context." << std::endl;
        viewer_thread_->stop();
        viewer_thread_.reset();
        if (window_)
            glfwMakeContextCurrent(window_);
    }

    /**
     * @brief poll_events processes the window events (mouse, resize, close). Main thread only
     *        (GLFW requirement). Polled at most every 4 ms, since a poll takes tens of microseconds.
     */
    void poll_events()
    {
        if (!window_)
            return;
        const auto now = steady_clock::now();
        if (now - last_event_poll_ < std::chrono::milliseconds(4))
            return;
        last_event_poll_ = now;
        glfwPollEvents();
        int width, height;
        glfwGetFramebufferSize(window_, &width, &height);
        framebuffer_width_ = width;
        framebuffer_height_ = height;
        glfwGetWindowSize(window_, &width, &height);
        screen_width_ = width;
        screen_height_ = height;
#ifdef DQ_MUJOCO_HAS_GUI
        if (gui_)
        {
            _run_gui_requests();
            _keep_moved_fixed_bodies();
        }
#endif
    }

#ifdef DQ_MUJOCO_HAS_GUI
    /**
     * @brief _perturb drives MuJoCo's perturbation of the selected body with the mouse (Ctrl + drag
     *        in the GUI). While the simulation steps, _step() pulls the body towards the mouse with
     *        a spring force, so the dynamics respond (contacts, joints, controllers). While it does
     *        not run in real time, free and kinematic bodies are also moved to the mouse at once,
     *        as in a paused simulation. Viewer thread, with viewer_mutex_ held.
     */
    void _perturb(const MujocoGui::PerturbInput& input)
    {
        const bool moved = input.dx != 0 || input.dy != 0;
        if (!input.start && !input.end && !moved && !pert_.active)
            return;
        std::lock_guard<std::mutex> lock(physics_mutex_);
        if (input.start && input.body > 0 && input.body < m_->nbody)
        {
            pert_.select = input.body;
            mju_copy3(pert_.localpos, input.localpos);
            mjv_initPerturb(m_, d_, &scn_, &pert_);  // the reference is the current pose
            pert_.active = input.active;
        }
        if (input.end)
        {
            pert_.active = 0;
            pert_.select = 0;
            return;
        }
        if (!pert_.active)
            return;
        if (moved)
            mjv_movePerturb(m_, d_, input.action, input.dx, input.dy, &scn_, &pert_);
        if (!simulation_running_)
        {
            mjv_applyPerturbPose(m_, d_, &pert_, 1);
            mj_forward(m_, d_);
        }
    }

    /**
     * @brief _gizmo moves the body dragged with the gizmo (GUI) to the pose of the gizmo, and holds
     *        it there while it is dragged: _step() applies the pose before each step, so the
     *        dynamics do not move the body away. Free bodies are released with zero velocity.
     *        Viewer thread, with viewer_mutex_ held.
     */
    void _gizmo(const MujocoGui::GizmoInput& input)
    {
        if (!input.moved && !input.end)
            return;
        std::lock_guard<std::mutex> lock(physics_mutex_);
        if (input.moved && input.body > 0 && input.body < m_->nbody)
        {
            const MujocoGui::Movable movable = MujocoGui::movable(m_, input.body);
            if (movable != MujocoGui::Movable::NONE)
            {
                gizmo_hold_.active = true;
                gizmo_hold_.body = input.body;
                gizmo_hold_.movable = movable;
                mju_copy3(gizmo_hold_.pos, input.pos);
                mju_copy4(gizmo_hold_.quat, input.quat);
                _apply_gizmo_hold();
                if (simulation_running_)
                    _publish_state();
                else
                    mj_forward(m_, d_);
            }
        }
        if (input.end && gizmo_hold_.active)
        {
            if (gizmo_hold_.movable == MujocoGui::Movable::FIXED)
                if (const char* name = mj_id2name(m_, mjOBJ_BODY, gizmo_hold_.body))
                    gizmo_moved_fixed_bodies_.push_back(name);
            gizmo_hold_.active = false;
        }
    }

    /**
     * @brief _apply_gizmo_hold puts the body dragged with the gizmo at its pose: in the state (free
     *        bodies, with zero velocity, and kinematic bodies) or in the model (fixed bodies
     *        attached to the world). The caller holds physics_mutex_.
     */
    void _apply_gizmo_hold()
    {
        const int body = gizmo_hold_.body;
        switch (gizmo_hold_.movable)
        {
        case MujocoGui::Movable::FREE:
        {
            const int joint = m_->body_jntadr[body];
            mjtNum* qpos = d_->qpos + m_->jnt_qposadr[joint];
            mju_copy3(qpos, gizmo_hold_.pos);
            mju_copy4(qpos + 3, gizmo_hold_.quat);
            mju_zero(d_->qvel + m_->jnt_dofadr[joint], 6);
            break;
        }
        case MujocoGui::Movable::KINEMATIC:
        {
            const int mocap = m_->body_mocapid[body];
            mju_copy3(d_->mocap_pos + 3 * mocap, gizmo_hold_.pos);
            mju_copy4(d_->mocap_quat + 4 * mocap, gizmo_hold_.quat);
            break;
        }
        case MujocoGui::Movable::FIXED:  // its parent is the world: the pose in the world
            mju_copy3(m_->body_pos + 3 * body, gizmo_hold_.pos);
            mju_copy4(m_->body_quat + 4 * body, gizmo_hold_.quat);
            break;
        case MujocoGui::Movable::NONE:
            break;
        }
    }

    /**
     * @brief _keep_moved_fixed_bodies writes the poses of the fixed bodies moved with the gizmo in
     *        the specification, so that edit_scene() (which compiles it) keeps them. A body of a
     *        robot placed with DQ_MujocoSceneBuilder::add_robot() is in a frame: its pose in the
     *        specification is relative to it. Main thread only (it owns the specification).
     */
    void _keep_moved_fixed_bodies()
    {
        std::vector<std::string> names;
        {
            std::lock_guard<std::mutex> lock(physics_mutex_);
            names.swap(gizmo_moved_fixed_bodies_);
        }
        for (const std::string& name : names)
        {
            mjtNum pos[3], quat[4];
            {
                std::lock_guard<std::mutex> lock(physics_mutex_);
                const int body = mj_name2id(m_, mjOBJ_BODY, name.c_str());
                if (body < 0)
                    continue;
                mju_copy3(pos, m_->body_pos + 3 * body);
                mju_copy4(quat, m_->body_quat + 4 * body);
            }
            mjsBody* spec_body = mjs_findBody(spec_, name.c_str());
            if (!spec_body)
                continue;
            if (mjsFrame* frame = mjs_getFrame(spec_body->element))
            {
                if (mjs_getFrame(frame->element) || frame->alt.type != mjORIENTATION_QUAT)
                {
                    std::cerr << "DQ_MujocoInterface: the new pose of \"" << name << "\" is not kept by "
                              << "edit_scene() (it is in a nested or rotated frame)." << std::endl;
                    continue;
                }
                // The pose relative to the frame
                mjtNum frame_quat[4], inverse[4], offset[3], q[4];
                mju_copy4(frame_quat, frame->quat);
                mju_normalize4(frame_quat);
                mju_negQuat(inverse, frame_quat);
                mju_sub3(offset, pos, frame->pos);
                mju_rotVecQuat(pos, offset, inverse);
                mju_mulQuat(q, inverse, quat);
                mju_copy4(quat, q);
            }
            mju_copy3(spec_body->pos, pos);
            mju_copy4(spec_body->quat, quat);
            spec_body->alt.type = mjORIENTATION_QUAT;
        }
    }
#endif

    /**
     * @brief _gui_active whether the GUI is drawn (it exists and is enabled).
     */
    bool _gui_active() const
    {
#ifdef DQ_MUJOCO_HAS_GUI
        return gui_ && gui_enabled_;
#else
        return false;
#endif
    }

#ifdef DQ_MUJOCO_HAS_GUI
    /**
     * @brief _create_gui creates the GUI, if it does not exist yet and there is a window. Main
     *        thread only (it asks GLFW for the content scale of the window).
     */
    void _create_gui()
    {
        if (gui_ || !window_)
            return;
        // Scale for high-DPI displays. Where the framebuffer is already larger than the window
        // (macOS, Wayland), ImGui scales by that ratio itself.
        float scale_x = 1, scale_y = 1;
        glfwGetWindowContentScale(window_, &scale_x, &scale_y);
        const float framebuffer_scale = screen_width_ > 0
                                        ? static_cast<float>(framebuffer_width_) / screen_width_ : 1.0f;
        auto gui = std::make_unique<MujocoGui>(scale_x / std::max(1.0f, framebuffer_scale));
        std::lock_guard<std::mutex> lock(viewer_mutex_);
        gui_ = std::move(gui);
    }

    /**
     * @brief _run_gui_requests runs the actions asked in the GUI (play, pause, step, reset), as if
     *        the user called the corresponding methods. Main thread only.
     */
    void _run_gui_requests()
    {
        const MujocoGui::Requests requests = gui_->take_requests();
        try
        {
            // A program that steps the simulation is its clock: the GUI does not run nor step it
            // (a request sent before the program's first step is ignored here)
            if (requests.toggle_simulation)
            {
                if (simulation_running_)
                    _stop_simulation();
                else if (!stepped_by_program_)
                    _start_simulation();
            }
            if (requests.step && !simulation_running_ && !stepped_by_program_)
                step_simulation();
            if (requests.reset)
                reset(requests.reset_keyframe);
        }
        catch (const std::exception& e)
        {
            std::cerr << "DQ_MujocoInterface: the action of the GUI failed: " << e.what() << std::endl;
        }
    }
#endif

    //----------------------------------------------------------------------------------------------
    // Physics thread
    //----------------------------------------------------------------------------------------------

    /**
     * @brief _anchor_real_time makes the physics thread keep real time from the current simulation
     *        time on. The caller must hold physics_mutex_.
     */
    void _anchor_real_time()
    {
        anchor_simulation_time_ = d_->time;
        anchor_wall_time_ = steady_clock::now();
    }

    /**
     * @brief _publish_state copies the state read by the getters into the snapshot. The caller
     *        must hold physics_mutex_.
     */
    void _publish_state()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        snapshot_->time = d_->time;
        mju_copy(snapshot_->qpos, d_->qpos, m_->nq);
        mju_copy(snapshot_->qvel, d_->qvel, m_->nv);
        mju_copy(snapshot_->ctrl, d_->ctrl, m_->nu);
        mju_copy(snapshot_->qfrc_actuator, d_->qfrc_actuator, m_->nv);
        mju_copy(snapshot_->xpos, d_->xpos, 3 * m_->nbody);
        mju_copy(snapshot_->xquat, d_->xquat, 4 * m_->nbody);
        mju_copy(snapshot_->site_xpos, d_->site_xpos, 3 * m_->nsite);
        mju_copy(snapshot_->site_xmat, d_->site_xmat, 9 * m_->nsite);
        mju_copy(snapshot_->sensordata, d_->sensordata, m_->nsensordata);
    }

    /**
     * @brief _apply_commands writes the pending actuator commands to d_->ctrl. The caller must
     *        hold physics_mutex_.
     */
    void _apply_commands()
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        for (int i = 0; i < m_->nu; i++)
        {
            if (command_pending_[i])
            {
                d_->ctrl[i] = command_ctrl_[i];
                command_pending_[i] = 0;
            }
        }
    }

    /**
     * @brief _step steps the simulation and measures the computation time. The caller must hold
     *        physics_mutex_.
     */
    void _step()
    {
        // The body dragged with the mouse (GUI) is pulled towards the mouse by a spring force,
        // and kinematic bodies follow it. The force is cleared when the drag ends.
        if (pert_.active)
        {
            mju_zero(d_->xfrc_applied, 6 * m_->nbody);
            mjv_applyPerturbPose(m_, d_, &pert_, 0);
            mjv_applyPerturbForce(m_, d_, &pert_);
            perturb_force_applied_ = true;
        }
        else if (perturb_force_applied_)
        {
            mju_zero(d_->xfrc_applied, 6 * m_->nbody);
            perturb_force_applied_ = false;
        }
#ifdef DQ_MUJOCO_HAS_GUI
        // The body dragged with the gizmo stays where the gizmo is
        if (gizmo_hold_.active)
            _apply_gizmo_hold();
#endif
        const auto start = std::chrono::high_resolution_clock::now();
        mj_step(m_, d_);
        const auto end = std::chrono::high_resolution_clock::now();
        computation_time_ = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        const double& N = computation_time_samples_;
        avg_computation_time_microseconds_ = (1.0/N)*computation_time_ + ((N-1.0)/N)*avg_computation_time_microseconds_;
    }

    /**
     * @brief _physics_tick runs in the physics thread, every timestep. It steps until the
     *        simulation time reaches the wall-clock time ("catch-up"), so that the simulation
     *        keeps real time even when a wake-up is late. The lock is taken per step, so the
     *        viewer and the teleport methods never wait more than one step.
     */
    void _physics_tick()
    {
        const auto now = steady_clock::now();
        int steps = 0;
        while (steps < max_steps_per_tick_)
        {
            std::lock_guard<std::mutex> lock(physics_mutex_);
            const double target = anchor_simulation_time_
                                  + std::chrono::duration<double>(now - anchor_wall_time_).count();
            if (d_->time >= target)
                break;
            _apply_commands();
            _step();
            _publish_state();
            rtf_window_simulated_ += m_->opt.timestep;
            steps++;
        }
        if (steps == max_steps_per_tick_)
        {
            // The computer cannot keep up: run as fast as possible from here on, instead of trying
            // to recover the lost time later
            std::lock_guard<std::mutex> lock(physics_mutex_);
            _anchor_real_time();
        }

        // Real-time factor, measured over windows of 0.5 s
        const double window = std::chrono::duration<double>(now - rtf_window_start_).count();
        if (window >= 0.5)
        {
            real_time_factor_ = rtf_window_simulated_ / window;
            rtf_window_simulated_ = 0;
            rtf_window_start_ = now;
        }
    }

    /**
     * @brief _start_simulation starts the physics thread (if it is not running).
     * @param resume True to continue after a pause (edit_scene()): the last real-time factor is
     *        kept until the next measurement.
     */
    void _start_simulation(const bool& resume = false)
    {
        if (simulation_running_)
            return;
        {
            std::lock_guard<std::mutex> lock(physics_mutex_);
            _anchor_real_time();
            _publish_state();
            std::lock_guard<std::mutex> command_lock(command_mutex_);
            std::fill(command_pending_.begin(), command_pending_.end(), 0);
        }
        if (!resume)
            real_time_factor_ = 0;
        rtf_window_simulated_ = 0;
        rtf_window_start_ = steady_clock::now();
        if (!physics_thread_)
            physics_thread_ = std::make_unique<ThreadManager>("mujoco_physics", m_->opt.timestep,
                                                              [this]() { _physics_tick(); });
        // From here on, the getters read the snapshot and the commands go to the buffer
        simulation_running_ = true;
        physics_thread_->start();
    }

    /**
     * @brief _stop_simulation stops the physics thread (if it is running). The state is kept.
     * @param pause True for a pause (edit_scene()), which keeps the last real-time factor.
     */
    void _stop_simulation(const bool& pause = false)
    {
        if (!simulation_running_)
            return;
        physics_thread_->stop();
        // Commands written after the last step are not lost
        {
            std::lock_guard<std::mutex> lock(physics_mutex_);
            _apply_commands();
        }
        simulation_running_ = false;
        if (!pause)
            real_time_factor_ = 0;
    }

    //----------------------------------------------------------------------------------------------
    // Access to the state, safe while the threads run
    //----------------------------------------------------------------------------------------------

    /**
     * @brief read_state calls f with the data that the getters read: the snapshot while the
     *        physics thread runs (under its lock), and d_ otherwise (only this thread writes d_
     *        then, so it can be read without the lock).
     */
    template<typename F>
    auto read_state(F&& f)
    {
        if (simulation_running_)
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            return f(static_cast<const mjData*>(snapshot_));
        }
        return f(static_cast<const mjData*>(d_));
    }

    /**
     * @brief modify_state runs f, which writes d_ directly (teleports, resets), under the physics
     *        lock. While the physics thread runs, the snapshot is updated too, so that the getters
     *        see the change at once. It waits at most for the step in progress.
     */
    template<typename F>
    void modify_state(F&& f)
    {
        std::lock_guard<std::mutex> lock(physics_mutex_);
        f();
        if (simulation_running_)
            _publish_state();
    }

    /**
     * @brief command sets the control of an actuator: in d_->ctrl, or in the command buffer
     *        while the physics thread runs, which applies it before its next step.
     */
    void command(const int& actuator_id, const double& value)
    {
        if (simulation_running_)
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            command_ctrl_[actuator_id] = value;
            command_pending_[actuator_id] = 1;
        }
        else
        {
            std::lock_guard<std::mutex> lock(physics_mutex_);  // the viewer may be reading d_
            d_->ctrl[actuator_id] = value;
        }
    }

    /**
     * @brief step_simulation steps once (synchronous mode).
     */
    void step_simulation()
    {
        if (!m_ || !d_)
            throw std::runtime_error("Model or data not initialized");
        if (simulation_running_)
            throw std::runtime_error("The simulation is running in real time (start_simulation()). "
                                     "Call stop_simulation() before stepping it with "
                                     "trigger_next_simulation_step().");
        std::lock_guard<std::mutex> lock(physics_mutex_);
        _step();
    }

    int _get_handle_from_map(const std::string& objectname)
    {
        std::lock_guard<std::mutex> lock(lookup_mutex_);
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

    // The caller must hold lookup_mutex_
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
    DQ _get_base_pose(const mjData* d, const std::string& bodyname) const
    {
        const mjtNum* qpos = d->qpos + m_->jnt_qposadr[_get_free_joint_handle(bodyname)];
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
    DQ _get_base_twist(const mjData* d, const std::string& bodyname, const REFERENCE_FRAME& reference_frame) const
    {
        const mjtNum* qvel = d->qvel + m_->jnt_dofadr[_get_free_joint_handle(bodyname)];
        const DQ x = _get_base_pose(d, bodyname);
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
        const DQ x = _get_base_pose(d_, bodyname);
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
    DQ _get_body_pose(const mjData* d, const std::string& bodyname) const
    {
        const int body_id = _get_body_handle(bodyname);
        const mjtNum* xpos = d->xpos + 3 * body_id;
        const mjtNum* xquat = d->xquat + 4 * body_id;
        DQ r = DQ(xquat[0], xquat[1], xquat[2], xquat[3]).normalize();
        DQ p = xpos[0]*i_ + xpos[1]*j_ + xpos[2]*k_;
        return r + 0.5*E_*p*r;
    }

    /**
     * @brief _set_body_pose moves a kinematic body (a mocap body): its pose is part of the state
     *        (mocap_pos and mocap_quat). The kinematics is updated, so that the body and site poses
     *        reflect the change at once. The caller must hold physics_mutex_.
     * @throws std::invalid_argument If the body is not a mocap body
     */
    void _set_body_pose(const std::string& bodyname, const DQ& pose)
    {
        const int body_id = _get_body_handle(bodyname);
        const int mocap_id = m_->body_mocapid[body_id];
        if (mocap_id < 0)
        {
            const bool is_free = m_->body_jntnum[body_id] == 1
                                 && m_->jnt_type[m_->body_jntadr[body_id]] == mjJNT_FREE;
            if (is_free)
                throw std::invalid_argument("The body \"" + bodyname + "\" is a FREE body (a floating base): "
                                            "move it with set_base_pose().");
            throw std::invalid_argument("The body \"" + bodyname + "\" is not KINEMATIC, so it cannot be moved. "
                                        "Add it with DQ_MujocoSceneBuilder::BODY::KINEMATIC (or mocap=\"true\" "
                                        "in the XML).");
        }
        const VectorXd p = pose.translation().vec3();
        const VectorXd r = pose.P().vec4();
        std::copy(p.data(), p.data() + 3, d_->mocap_pos + 3 * mocap_id);
        std::copy(r.data(), r.data() + 4, d_->mocap_quat + 4 * mocap_id);
        mj_kinematics(m_, d_);
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
        {
            std::lock_guard<std::mutex> lock(lookup_mutex_);
            auto search = actuator_map_.find(jointname);
            if (search != actuator_map_.end()) {
                return search->second;
            }
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

        std::lock_guard<std::mutex> lock(lookup_mutex_);
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
    DQ _get_site_pose(const mjData* d, const std::string& site_name)
    {
        // Get the site ID
        int site_id = mj_name2id(m_, mjOBJ_SITE, site_name.c_str());
        if (site_id == -1)
            throw std::runtime_error("Site '" + site_name + "' not found");

        // Get position from d->site_xpos (3 values per site)
        // Note: Use site_xpos for sites, xpos is for bodies
        const mjtNum* xpos = d->site_xpos + 3 * site_id;
        DQ p = xpos[0] * i_ + xpos[1] * j_ + xpos[2] * k_;

        // mjData stores site orientations only as rotation matrices (site_xmat, 9 values per site)
        mjtNum quat[4]; // w, x, y, z
        mju_mat2Quat(quat, d->site_xmat + 9 * site_id);
        DQ r = DQ(quat[0], quat[1], quat[2], quat[3]);
        r = r.normalize();  // Ensure it's a unit quaternion

        // Return the dual quaternion pose: r + 0.5 * ε * p * r
        return r + 0.5 * E_ * p * r;
    }


    /**
     * @brief _get_sensor_id returns the id of a sensor, by its name.
     */
    int _get_sensor_id(const std::string& sensorname, const std::string& function_name) const
    {
        const int id = mj_name2id(m_, mjOBJ_SENSOR, sensorname.c_str());
        if (id < 0)
            throw std::runtime_error(function_name + ". There is no sensor \"" + sensorname + "\" in the model.");
        return id;
    }

    /**
     * @brief _get_site_sensor_id returns the id of the sensor of a given type attached to a site
     *        (the typed sensor methods address sensors by their site, whatever their names).
     * @param what The kind of sensor, for the error message (e.g., "IMU (accelerometer)").
     */
    int _get_site_sensor_id(const std::string& sitename, const mjtSensor& type, const std::string& what,
                            const std::string& function_name) const
    {
        const int site_id = mj_name2id(m_, mjOBJ_SITE, sitename.c_str());
        if (site_id < 0)
            throw std::runtime_error(function_name + ". There is no site \"" + sitename + "\" in the model.");
        for (int s = 0; s < m_->nsensor; s++)
            if (m_->sensor_type[s] == type && m_->sensor_objtype[s] == mjOBJ_SITE && m_->sensor_objid[s] == site_id
                && m_->sensor_refid[s] < 0)  // in the world frame (a framequat can be relative to another frame)
                return s;
        throw std::runtime_error(function_name + ". The site \"" + sitename + "\" has no " + what + " sensor.");
    }

    const mjtNum* _sensor_data(const mjData* d, const int& sensor_id) const
    {
        return d->sensordata + m_->sensor_adr[sensor_id];
    }

    /**
     * @brief _get_joint_position
     * @param jointname Name of the joint to query
     * @return Current joint position in radians (revolute) or meters (prismatic)
     *
     * Returns the actual joint position from the simulation state (d_->qpos).
     * This reflects the current physical position after dynamics have been applied.
     */
    double _get_joint_position(const mjData* d, const std::string& jointname)
    {
        int qpos_adr = m_->jnt_qposadr[_get_single_dof_joint_handle(jointname)];
        return d->qpos[qpos_adr];
    }


    /**
     * @brief _get_joint_velocity
     * @param jointname Name of the joint to query
     * @return Current joint velocity in rad/s (revolute) or m/s (prismatic)
     *
     * Returns the actual joint velocity from the simulation state (d_->qvel).
     * This is used by MuJoCo's controllers for damping calculations.
     */
    double _get_joint_velocity(const mjData* d, const std::string& jointname)
    {
        int qvel_adr = m_->jnt_dofadr[_get_single_dof_joint_handle(jointname)];
        return d->qvel[qvel_adr];
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
        command(get_actuator_handle_from_joint(jointname), angle_rad);
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
        command(get_actuator_handle_from_joint(jointname), velocity_rad_per_sec);
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
        command(get_actuator_handle_from_joint(jointname), force_nm);
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
    double _get_joint_force(const mjData* d, const std::string& jointname)
    {
        int dof_adr = m_->jnt_dofadr[_get_single_dof_joint_handle(jointname)];
        // qfrc_actuator (size nv) contains the forces generated by actuators, indexed by DoF
        return d->qfrc_actuator[dof_adr];
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

/**
 * @brief Load a model held in memory
 * @param model XML text and the files it refers to (meshes, textures, <include> files).
 *
 * Works like load_model(const std::string&), but no file is read from disk: the XML and the
 * assets are given to MuJoCo through a virtual file system. Use it with the models of the
 * robots library (e.g., GenericQuadrupedRobot::description().model) or with models built at runtime.
 *
 * @throws std::invalid_argument If the name or the XML is empty, or an asset has the name of the model
 * @throws std::runtime_error If MuJoCo fails to load the model (the message includes MuJoCo's
 *         error, e.g., a missing asset)
 */
void DQ_MujocoInterface::load_model(const DQ_MujocoModel& model)
{
    impl_->load_model(model);
}



/**
 * @brief Advance the simulation by one timestep (stepped simulation)
 *
 * Calls MuJoCo's mj_step() once: the simulation time advances by the timestep of the model
 * (<option timestep="..."/>, 0.002 s by default), and the getters then read the state after the
 * step. The commands (set_joint_target_*) written before the call are applied in it. It does not
 * wait: to run in real time, pace the loop at the timestep (e.g., with a sas Clock); otherwise the
 * simulation runs as fast as the computer allows. The viewer thread draws the window meanwhile.
 * @code
 * while (!mi.should_close_window())
 * {
 *     mi.set_joint_target_positions(joints, controller(mi.get_joint_positions(joints)));
 *     mi.trigger_next_simulation_step();
 *     clock.update_and_sleep();
 * }
 * @endcode
 *
 * Use it for deterministic experiments and per-step control; start_simulation() runs the
 * simulation in real time instead. The program is then the clock of the simulation: the GUI (see
 * set_gui_enabled()) does not play or step it, until the program calls start_simulation().
 *
 * @throws std::runtime_error If no model is loaded, or the simulation runs in real time
 *         (start_simulation(); call stop_simulation() first)
 */
void DQ_MujocoInterface::trigger_next_simulation_step()
{
    _check_model();
    impl_->step_simulation();
    impl_->stepped_by_program_ = true;
}

double DQ_MujocoInterface::get_simulation_time() const
{
    _check_model();
    return impl_->read_state([](const mjData* d) { return d->time; });
}

/**
 * @brief Start the simulation in real time
 *
 * A physics thread steps the simulation so that the simulation time follows the wall-clock time,
 * like a real robot (and like CoppeliaSim in its default mode). The program does not step it:
 * it reads the state and sends commands at its own rate, e.g.,
 * @code
 * mi.start_simulation();
 * while (!mi.should_close_window())
 * {
 *     VectorXd q = mi.get_joint_positions(joints);
 *     mi.set_joint_target_positions(joints, controller(q));
 *     clock.update_and_sleep();   // the control rate, e.g., with a sas Clock
 * }
 * mi.stop_simulation();
 * @endcode
 *
 * While it runs:
 * - The getters read the state published after the last step, so they never wait for a step.
 * - The set_joint_target_* commands are applied before the next step.
 * - Teleports (set_joint_positions(), set_base_pose(), reset_simulation(), ...) wait for the step
 *   in progress, if any, and take effect at once.
 * - trigger_next_simulation_step() throws: stop the simulation to step it by hand.
 *
 * If the computer cannot keep up, the simulation runs as fast as it can (see
 * get_real_time_factor()). Calling it while the simulation runs does nothing.
 *
 * @throws std::runtime_error If no model is loaded
 */
void DQ_MujocoInterface::start_simulation()
{
    _check_model();
    impl_->_start_simulation();
    impl_->stepped_by_program_ = false;  // the GUI can pause and play it again
}

/**
 * @brief Stop the real-time simulation started by start_simulation()
 *
 * The state is kept: the simulation can be started again, stepped with
 * trigger_next_simulation_step(), or reset. Calling it when the simulation is not running does
 * nothing. load_model() and the destructor also stop it.
 */
void DQ_MujocoInterface::stop_simulation()
{
    impl_->_stop_simulation();
}

/**
 * @brief Whether the simulation runs in real time (between start_simulation() and stop_simulation())
 */
bool DQ_MujocoInterface::is_simulation_running() const
{
    return impl_->simulation_running_;
}

/**
 * @brief Simulated time per wall-clock time, measured over the last 0.5 s
 * @return About 1 while the simulation keeps real time, less if the computer cannot keep up, and
 *         0 when the simulation is not running (or during its first 0.5 s).
 */
double DQ_MujocoInterface::get_real_time_factor() const
{
    return impl_->real_time_factor_;
}

/**
 * @brief Check whether the user asked to close the window
 * @return true if the window was closed. Always false in HEADLESS mode.
 *
 * It also processes the window events (mouse, resize, close), which GLFW only allows in the main
 * thread, so call it regularly from the main thread, e.g., as the condition of the main loop.
 * Otherwise the window keeps being drawn but does not respond.
 */
bool DQ_MujocoInterface::should_close_window() const
{
    _check_model();
    if (!impl_->window_)
        return false;
    impl_->poll_events();
    return glfwWindowShouldClose(impl_->window_);
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
    std::lock_guard<std::mutex> lock(impl_->viewer_mutex_);
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
    std::lock_guard<std::mutex> lock(impl_->viewer_mutex_);
    impl_->opt_.flags[mjVIS_JOINT] = flag ? 1 : 0; // Enable visualization of joint axes
}

void DQ_MujocoInterface::set_contact_view(const bool &flag)
{
    _check_model();
    std::lock_guard<std::mutex> lock(impl_->viewer_mutex_);
    impl_->opt_.flags[mjVIS_CONTACTFORCE] = flag ? 1 : 0;
}

void DQ_MujocoInterface::set_center_of_mass_view(const bool &flag)
{
    _check_model();
    std::lock_guard<std::mutex> lock(impl_->viewer_mutex_);
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
    std::lock_guard<std::mutex> lock(impl_->viewer_mutex_);
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
    // All the values come from the same simulation step
    return impl_->read_state([&](const mjData* d) {
        const int n = jointnames.size();
        VectorXd joint_positions(n);
        for (int i = 0; i < n; i++)
            joint_positions(i) = impl_->_get_joint_position(d, jointnames.at(i));
        return joint_positions;
    });
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
    // All the values come from the same simulation step
    return impl_->read_state([&](const mjData* d) {
        const int n = jointnames.size();
        VectorXd joint_velocities(n);
        for (int i = 0; i < n; i++)
            joint_velocities(i) = impl_->_get_joint_velocity(d, jointnames.at(i));
        return joint_velocities;
    });
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
    impl_->modify_state([&]() {
        for (std::size_t i = 0; i < jointnames.size(); i++)
            impl_->_set_joint_position(jointnames.at(i), angles_rad(i));
        // update body/site poses, etc. to match the new joint positions
        mj_forward(impl_->m_, impl_->d_);
    });
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
    // All the values come from the same simulation step
    return impl_->read_state([&](const mjData* d) {
        const int n = jointnames.size();
        VectorXd joint_forces(n);
        for (int i = 0; i < n; i++)
            joint_forces(i) = impl_->_get_joint_force(d, jointnames.at(i));
        return joint_forces;
    });
}

DQ DQ_MujocoInterface::get_site_pose(const std::string &objectname) const
{
    _check_model();
    return impl_->read_state([&](const mjData* d) { return impl_->_get_site_pose(d, objectname); });
}

void DQ_MujocoInterface::set_site_pose(const std::string &objectname, const DQ &pose)
{
    // For C++20
    // std::string function_name = static_cast<std::string>(std::source_location::current().function_name());
    std::string function_name = {"DQ_MujocoInterface::set_site_pose"};
    if (!is_unit(pose))
        throw std::runtime_error(function_name + ". The pose must be a unit dual quaternion!");
    _check_model();
    impl_->modify_state([&]() { impl_->_set_site_pose(objectname, pose); });
}

/**
 * @brief The names of the sensors of the model (MuJoCo <sensor> elements, and the sensors added with
 *        DQ_MujocoSceneBuilder::add_imu(), ...), in the order of the model.
 */
std::vector<std::string> DQ_MujocoInterface::get_sensor_names() const
{
    _check_model();
    std::vector<std::string> names;
    for (int s = 0; s < impl_->m_->nsensor; s++)
        if (const char* name = mj_id2name(impl_->m_, mjOBJ_SENSOR, s))
            names.push_back(name);
    return names;
}

/**
 * @brief The values of a sensor, by its name: any MuJoCo sensor (e.g., jointpos, framelinvel, touch),
 *        as MuJoCo computes it, also while the simulation runs
 * @param sensorname Name of the sensor (the name of the <sensor> element).
 * @return Its values (e.g., 3 for an accelerometer, 4 for a framequat: w, x, y, z).
 *
 * The values are those of the last step of the simulation. For IMUs, force/torque sensors, touch
 * sensors and rangefinders, the typed methods (get_imu_orientation(), get_wrench(), ...) return them
 * as DQ Robotics types, addressed by the site of the sensor.
 * @throws std::runtime_error If there is no sensor with that name
 */
VectorXd DQ_MujocoInterface::get_sensor_data(const std::string& sensorname) const
{
    _check_model();
    const int id = impl_->_get_sensor_id(sensorname, error_msg_layout_ + "get_sensor_data");
    return impl_->read_state([&](const mjData* d) {
        const mjtNum* data = impl_->_sensor_data(d, id);
        return VectorXd(Map<const VectorXd>(data, impl_->m_->sensor_dim[id]));
    });
}

/**
 * @brief The orientation of an IMU in the world (a MuJoCo framequat sensor on its site)
 * @param sitename The site of the IMU (e.g., added with DQ_MujocoSceneBuilder::add_imu()).
 * @return A unit quaternion: the rotation from the world to the frame of the IMU.
 * @throws std::runtime_error If there is no such site, or it has no framequat sensor (in the world)
 */
DQ DQ_MujocoInterface::get_imu_orientation(const std::string& sitename) const
{
    _check_model();
    const int id = impl_->_get_site_sensor_id(sitename, mjSENS_FRAMEQUAT, "IMU (framequat)",
                                              error_msg_layout_ + "get_imu_orientation");
    return impl_->read_state([&](const mjData* d) {
        const mjtNum* q = impl_->_sensor_data(d, id);
        return DQ(q[0], q[1], q[2], q[3]).normalize();
    });
}

/**
 * @brief The angular velocity measured by an IMU (a MuJoCo gyro sensor on its site)
 * @param sitename The site of the IMU.
 * @return A pure quaternion, in rad/s, expressed in the frame of the IMU (as a real gyroscope).
 * @throws std::runtime_error If there is no such site, or it has no gyro sensor
 */
DQ DQ_MujocoInterface::get_imu_angular_velocity(const std::string& sitename) const
{
    _check_model();
    const int id = impl_->_get_site_sensor_id(sitename, mjSENS_GYRO, "IMU (gyro)",
                                              error_msg_layout_ + "get_imu_angular_velocity");
    return impl_->read_state([&](const mjData* d) {
        const mjtNum* w = impl_->_sensor_data(d, id);
        return DQ(0, w[0], w[1], w[2]);
    });
}

/**
 * @brief The linear acceleration measured by an IMU (a MuJoCo accelerometer on its site)
 * @param sitename The site of the IMU.
 * @return A pure quaternion, in m/s^2, expressed in the frame of the IMU. As a real accelerometer,
 *         it includes gravity: at rest it measures 9.81 m/s^2 upwards, and in free fall zero.
 * @note MuJoCo does not compute accelerations for bodies welded to the world, so an IMU on one
 *       measures zero. Mount IMUs on bodies that move (e.g., the floating base of a robot).
 * @throws std::runtime_error If there is no such site, or it has no accelerometer
 */
DQ DQ_MujocoInterface::get_imu_linear_acceleration(const std::string& sitename) const
{
    _check_model();
    const int id = impl_->_get_site_sensor_id(sitename, mjSENS_ACCELEROMETER, "IMU (accelerometer)",
                                              error_msg_layout_ + "get_imu_linear_acceleration");
    return impl_->read_state([&](const mjData* d) {
        const mjtNum* a = impl_->_sensor_data(d, id);
        return DQ(0, a[0], a[1], a[2]);
    });
}

/**
 * @brief The wrench measured by a force/torque sensor (MuJoCo force and torque sensors on its site)
 * @param sitename The site of the sensor (e.g., added with
 *        DQ_MujocoSceneBuilder::add_force_torque_sensor()), on the body whose interaction with its
 *        parent body is measured.
 * @return The wrench f + E_*tau, in N and N·m, expressed in the frame of the site, at its origin: the
 *         force and the torque that the parent body exerts on the body of the site. E.g., a sensor
 *         at a wrist, with nothing in the hand, measures the weight of what is below it, upwards.
 * @throws std::runtime_error If there is no such site, or it has no force or no torque sensor
 */
DQ DQ_MujocoInterface::get_wrench(const std::string& sitename) const
{
    _check_model();
    const std::string function_name = error_msg_layout_ + "get_wrench";
    const int force_id = impl_->_get_site_sensor_id(sitename, mjSENS_FORCE, "force", function_name);
    const int torque_id = impl_->_get_site_sensor_id(sitename, mjSENS_TORQUE, "torque", function_name);
    return impl_->read_state([&](const mjData* d) {
        const mjtNum* f = impl_->_sensor_data(d, force_id);
        const mjtNum* tau = impl_->_sensor_data(d, torque_id);
        return DQ(0, f[0], f[1], f[2]) + E_*DQ(0, tau[0], tau[1], tau[2]);
    });
}

/**
 * @brief The normal force measured by a touch sensor (a MuJoCo touch sensor on its site)
 * @param sitename The site of the sensor (e.g., added with DQ_MujocoSceneBuilder::add_touch_sensor()):
 *        the contacts inside its volume are measured.
 * @return The sum of the normal forces of those contacts, in N (zero without contact).
 * @throws std::runtime_error If there is no such site, or it has no touch sensor
 */
double DQ_MujocoInterface::get_touch(const std::string& sitename) const
{
    _check_model();
    const int id = impl_->_get_site_sensor_id(sitename, mjSENS_TOUCH, "touch", error_msg_layout_ + "get_touch");
    return impl_->read_state([&](const mjData* d) { return impl_->_sensor_data(d, id)[0]; });
}

/**
 * @brief The distance measured by a rangefinder (a MuJoCo rangefinder on its site)
 * @param sitename The site of the rangefinder (e.g., added with
 *        DQ_MujocoSceneBuilder::add_rangefinder()), which measures along its z axis.
 * @return The distance to the first geometry along the z axis of the site, in m, or -1 if there is
 *         none. Geometries of the body of the site are not seen.
 * @throws std::runtime_error If there is no such site, or it has no rangefinder
 */
double DQ_MujocoInterface::get_range(const std::string& sitename) const
{
    _check_model();
    const int id = impl_->_get_site_sensor_id(sitename, mjSENS_RANGEFINDER, "rangefinder",
                                              error_msg_layout_ + "get_range");
    return impl_->read_state([&](const mjData* d) { return impl_->_sensor_data(d, id)[0]; });
}


/**
 * @brief The names of the cameras of the model (MuJoCo <camera> elements, and the cameras added
 *        with DQ_MujocoSceneBuilder::add_camera()), in the order of the model.
 */
std::vector<std::string> DQ_MujocoInterface::get_camera_names() const
{
    _check_model();
    std::vector<std::string> names;
    for (int c = 0; c < impl_->m_->ncam; c++)
        if (const char* name = mj_id2name(impl_->m_, mjOBJ_CAMERA, c))
            names.push_back(name);
    return names;
}

/**
 * @brief Start rendering a camera: a camera thread renders its images offscreen, at its rate, and
 *        get_camera_image() returns the latest one. Starting it again changes its settings.
 * @param cameraname Name of a camera of the model (see get_camera_names()).
 * @param rate_hz Images per second (in wall-clock time), at most 500. If the computer cannot render
 *        them that fast, images are skipped: the count of the images tells how many were rendered.
 * @param with_depth Also read the depth of each pixel, as distances in meters.
 * @param quality FULL (as the viewer, the default) or FAST (without shadows, reflections and skybox).
 *
 * The size of the images is the resolution of the camera in the model (set by add_camera(), or the
 * resolution attribute of a <camera>), or 640x480 if it has none. A camera looks along the -z axis
 * of its frame, with x to the right of the image and y up, and its fovy is the vertical field of view.
 *
 * The camera thread has its own OpenGL context: a hidden window in WINDOW mode, and EGL in HEADLESS
 * mode (no display needed; it needs a library built with EGL). It holds the physics lock only to
 * copy the state (a few microseconds), so the cameras do not delay the simulation. Call it from the
 * main thread (the one that loaded the model).
 *
 * @throws std::invalid_argument If there is no such camera, or the rate is not in (0, 500]
 * @throws std::runtime_error If the OpenGL context of the cameras cannot be created (e.g., HEADLESS
 *         mode without EGL)
 */
void DQ_MujocoInterface::start_camera(const std::string& cameraname, const double& rate_hz, const bool& with_depth,
                                      const CAMERA_QUALITY& quality)
{
    _check_model();
    if (!impl_->cameras_)
        impl_->cameras_ = std::make_unique<CameraRenderer>(impl_->rendering_ == RENDERING::WINDOW, impl_->m_,
                                                           impl_->d_, impl_->physics_mutex_);
    impl_->cameras_->start_camera(cameraname, rate_hz, with_depth, quality);
}

/**
 * @brief Stop rendering a camera (its last image is discarded). Call it from the main thread.
 * @throws std::invalid_argument If the camera is not started
 */
void DQ_MujocoInterface::stop_camera(const std::string& cameraname)
{
    _check_model();
    if (!impl_->cameras_)
        throw std::invalid_argument(error_msg_layout_ + "stop_camera. The camera \"" + cameraname
                                    + "\" is not started.");
    impl_->cameras_->stop_camera(cameraname);
}

/**
 * @brief The latest image of a started camera, also while the simulation runs
 * @return The image: rgb (and depth) with rows from top to bottom, as a ROS sensor_msgs/Image; the
 *         simulation time of the state it shows; and its count (zero before the first image). Read
 *         it again when the count changes, e.g., to publish each image once.
 * @throws std::runtime_error If the camera is not started
 */
DQ_MujocoInterface::CameraImage DQ_MujocoInterface::get_camera_image(const std::string& cameraname) const
{
    _check_model();
    if (!impl_->cameras_)
        throw std::runtime_error(error_msg_layout_ + "get_camera_image. The camera \"" + cameraname
                                 + "\" is not started (start_camera()).");
    return impl_->cameras_->get_image(cameraname);
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
 * while (!mi.should_close_window()) {
 *     mi.set_reference_frame_pose("x_desired", x_desired);
 *     ...
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
    std::lock_guard<std::mutex> lock(impl_->reference_frames_mutex_);
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
    std::lock_guard<std::mutex> lock(impl_->reference_frames_mutex_);
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
    std::lock_guard<std::mutex> lock(impl_->reference_frames_mutex_);
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
    std::lock_guard<std::mutex> lock(impl_->reference_frames_mutex_);
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
    return impl_->read_state([&](const mjData* d) { return impl_->_get_base_pose(d, bodyname); });
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
    impl_->modify_state([&]() {
        impl_->_set_base_pose(bodyname, pose);
        // update body/site poses, etc. to match the new base pose
        mj_forward(impl_->m_, impl_->d_);
    });
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
    return impl_->read_state([&](const mjData* d) { return impl_->_get_base_twist(d, bodyname, reference_frame); });
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
    impl_->modify_state([&]() {
        impl_->_set_base_twist(bodyname, twist, reference_frame);
        // update velocity-dependent quantities
        mj_forward(impl_->m_, impl_->d_);
    });
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
    return impl_->read_state([&](const mjData* d) { return impl_->_get_body_pose(d, bodyname); });
}

/**
 * @brief Move a kinematic body
 * @param bodyname Name of a KINEMATIC body (DQ_MujocoSceneBuilder::BODY::KINEMATIC, or a body with
 *        mocap="true" in the XML).
 * @param pose Unit dual quaternion with the desired pose of the body frame in the world.
 *
 * Kinematic bodies are moved by the program, not by the physics: gravity and contacts do not move
 * them, but they collide with and push FREE objects and robots. Call it as often as needed, e.g.,
 * in every iteration of the control loop to follow a trajectory; it is cheap (it only updates the
 * kinematics). Like the other teleports, it takes effect at once, also while the simulation runs
 * in real time. reset_simulation() puts the body back at its initial pose.
 *
 * @note MuJoCo treats the moved body as having zero velocity in the contacts, so an object resting
 *       on a moving kinematic platform slides instead of being carried by friction.
 * @throws std::runtime_error If the body does not exist, or the pose is not a unit dual quaternion
 * @throws std::invalid_argument If the body is not kinematic (FIXED bodies never move; FREE bodies
 *         are moved with set_base_pose())
 */
void DQ_MujocoInterface::set_body_pose(const std::string& bodyname, const DQ& pose)
{
    if (!is_unit(pose))
        throw std::runtime_error(error_msg_layout_ + "set_body_pose. The pose must be a unit dual quaternion!");
    _check_model();
    impl_->modify_state([&]() { impl_->_set_body_pose(bodyname, pose); });
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
    const int body_id = bodyname.empty() ? -1 : impl_->_get_body_handle(bodyname);
    std::lock_guard<std::mutex> lock(impl_->viewer_mutex_);
    impl_->camera_tracking_body_ = bodyname;
    mjvCamera& cam = impl_->cam_;
    if (body_id == -1)
    {
        cam.type = mjCAMERA_FREE;
        cam.trackbodyid = -1;
        return;
    }
    cam.trackbodyid = body_id;
    cam.type = mjCAMERA_TRACKING;
    cam.fixedcamid = -1;
}

/**
 * @brief Whether this build of the library has the GUI of the viewer (see set_gui_enabled())
 * @return false if the library was built with DQ_MUJOCO_BUILD_GUI=OFF.
 */
bool DQ_MujocoInterface::is_gui_available()
{
#ifdef DQ_MUJOCO_HAS_GUI
    return true;
#else
    return false;
#endif
}

/**
 * @brief Show or hide the GUI of the viewer
 * @param flag true to show the GUI, false to show only the 3D view (default).
 *
 * The GUI surrounds the 3D view with docked panels, in the style of the editors of 3D tools:
 * - Scene: the tree of bodies, with a search box. Select a body to see its properties, here or by
 *   clicking it in the 3D view (Escape: none). The selected body and its children are highlighted.
 *   Ctrl + drag in the 3D view interacts with the selected body, as in MuJoCo's simulate: left
 *   rotates it and right moves it (Shift: in the horizontal plane). While the simulation steps, a
 *   spring force pulls the body towards the mouse, so the robot and its controller respond to it;
 *   otherwise free and kinematic bodies are moved to the mouse at once.
 * - Gizmo, as in Omniverse: the selected body gets handles to move (W) and rotate (E) it, in the
 *   world or body axes (toolbar in the 3D view; Q: no gizmo). It moves free bodies (objects,
 *   floating bases), kinematic bodies, and fixed bodies attached to the world (e.g., the base of a
 *   fixed robot, which moves the whole robot); bodies moved by their joints have none. A dragged
 *   body is held at the gizmo, also while the simulation runs, and free bodies are released with
 *   zero velocity. Fixed bodies keep their new pose, also in edit_scene(); reset_simulation() puts
 *   free and kinematic bodies back.
 * - Navigation gizmo (top-right corner of the 3D view): the axes of the world (X, Y, Z), as in
 *   Blender. Click an axis to look along it, or drag the gizmo to turn the camera.
 * - Properties: the pose of the selected body (position and rotation), its mass, and a button
 *   to follow it with the camera.
 * - Joints: the position (within its limits) and velocity of each hinge and slide joint.
 * - Sensors: the values of the sensors of the model.
 * - Visualization: rendering options, elements to show (joints, contacts, ...), frames, labels
 *   and groups.
 * The menu bar has the simulation controls (play, pause, step, reset, reset to a keyframe) and the
 * camera (free, following a body, or a camera of the model), and the status bar shows the
 * simulation time, the real-time factor and the frame rate. The panels can be moved, docked
 * elsewhere, or closed (View menu).
 *
 * The simulation controls act as the corresponding methods called by the program: play and pause
 * are start_simulation() and stop_simulation(), and step is trigger_next_simulation_step(). They run
 * in the main thread, in should_close_window(). Keys: Space (play/pause), Right (step), Backspace
 * (reset). In a program that steps the simulation itself (trigger_next_simulation_step()), play
 * and step are disabled: the program is the clock (playing would make its next step throw).
 *
 * Call it from the main thread, before or after load_model(). In HEADLESS mode it has no effect.
 *
 * @throws std::runtime_error If flag is true and the library has no GUI (see is_gui_available())
 */
void DQ_MujocoInterface::set_gui_enabled(const bool& flag)
{
#ifdef DQ_MUJOCO_HAS_GUI
    impl_->gui_enabled_ = flag;
    if (flag)
        impl_->_create_gui();
#else
    if (flag)
        throw std::runtime_error(error_msg_layout_ + "set_gui_enabled. This build of the library has no GUI "
                                 "(it was built with DQ_MUJOCO_BUILD_GUI=OFF).");
#endif
}

/**
 * @brief Whether the GUI of the viewer is enabled (see set_gui_enabled())
 */
bool DQ_MujocoInterface::is_gui_enabled() const
{
    return impl_->gui_enabled_;
}

/**
 * @brief Edit the loaded scene, also while the simulation runs: add or remove objects, sites,
 *        lights and robots
 * @param edit A function that edits the scene with the methods of DQ_MujocoSceneBuilder, e.g.,
 * @code
 * mi.edit_scene([&](DQ_MujocoSceneBuilder& scene) {
 *     scene.add_sphere("ball2", x, 0.03, DQ_MujocoSceneBuilder::BODY::FREE);
 *     scene.remove("ball1");
 * });
 * @endcode
 *
 * Robots with files (meshes, textures) can be added too, also to a model loaded from a file.
 *
 * The state of the simulation is kept (time, positions, velocities, commands): the scene changes
 * without a reset, and new objects and robots start at their home configuration. The "home"
 * keyframe of the scene is updated to include them. Other keyframes of the model are not.
 * Site poses set with set_site_pose() and the camera are kept too.
 *
 * The edits are checked before they are applied: if they make the scene invalid (e.g., a name
 * that is already used), or edit throws, the scene is left as it was and the exception is
 * rethrown. A real-time simulation keeps running: it waits while the model is recompiled and then
 * catches up. Call it from the main thread (the one that loaded the model).
 *
 * @note Recommendation: build the scene before starting the control loop, and do not edit it while
 *       the loop runs. Each edit recompiles the whole scene, which stalls the simulation: about 10 ms
 *       with primitive shapes, but hundreds of milliseconds with robots that have large meshes (e.g.,
 *       about 260 ms to add a Unitree Z1 of cpp-interface-mujoco-robots). To move objects during the
 *       loop, use KINEMATIC bodies and set_body_pose().
 *
 * @note Handles and names cached by the interface are refreshed, since the ids of the elements can
 *       change. Names given by the user stay valid.
 * @throws std::runtime_error If no model is loaded, or the edits make the scene invalid
 */
void DQ_MujocoInterface::edit_scene(const std::function<void(DQ_MujocoSceneBuilder&)>& edit)
{
    _check_model();
    Impl& impl = *impl_;
#ifdef DQ_MUJOCO_HAS_GUI
    impl._keep_moved_fixed_bodies();  // the bodies moved with the gizmo stay where they are
#endif

    // Edit the specification, and check the result before changing the model, since a failed
    // mj_recompile() would leave the model and the data invalid. The checks compile a copy: the
    // specification itself must only be compiled by mj_recompile(), which maps the state of the
    // previous compilation to the new model.
    mjSpec* backup = mj_copySpec(impl.spec_);
    if (!backup)
        throw std::runtime_error(error_msg_layout_ + "edit_scene. Failed to copy the scene.");
    struct Compiled
    {
        mjSpec* spec = nullptr;
        mjModel* model = nullptr;
        ~Compiled()
        {
            if (model) mj_deleteModel(model);
            if (spec) mj_deleteSpec(spec);
        }
    };
    std::vector<std::string> added_files;
    try
    {
        DQ_MujocoSceneBuilder scene(std::make_unique<DQ_MujocoSceneBuilder::Impl>(impl.spec_, impl.m_));
        edit(scene);

        // The files of the robots added by the edit (meshes, textures). A model loaded from a file
        // has no virtual file system: the one created here falls back to the file system.
        if (!scene.impl_->assets_.empty() && !impl.vfs_)
        {
            impl.vfs_.reset(new mjVFS);
            mj_defaultVFS(impl.vfs_.get());
        }
        for (const auto& [name, content] : scene.impl_->assets_)
        {
            const int result = content.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())
                ? -1 : mj_addBufferVFS(impl.vfs_.get(), name.c_str(), content.data(),
                                       static_cast<int>(content.size()));
            if (result == 0)
                added_files.push_back(name);
            else if (result != 2)  // 2: already there, the same file (its path has the hash of its content)
                throw std::runtime_error(error_msg_layout_ + "edit_scene. Failed to add the file \""
                                         + name + "\" to the scene.");
        }

        // Check the edited scene
        Compiled check;
        check.spec = mj_copySpec(impl.spec_);
        check.model = check.spec ? mj_compile(check.spec, impl.vfs_.get()) : nullptr;
        if (!check.model)
            throw std::runtime_error(error_msg_layout_ + "edit_scene. Invalid scene: "
                                     + (check.spec ? mjs_getError(check.spec) : "failed to copy it"));

        // Pause only what must be paused (and continue at the end, also if it fails): the viewer
        // if the GPU context must be rebuilt, the physics thread if its period (the timestep)
        // changes, and the cameras (their scenes are remade). Otherwise the threads only wait for
        // the locks while the model is recompiled.
        const mjModel* c = check.model;
        const mjModel* m = impl.m_;
        const bool gpu_resources_change = c->ntex != m->ntex || c->nmesh != m->nmesh || c->nhfield != m->nhfield
                                          || c->nskin != m->nskin || c->nflex != m->nflex;
        const bool rebuild_gpu_context = impl.window_ && gpu_resources_change;
        const bool restart_physics = impl.simulation_running_ && c->opt.timestep != m->opt.timestep;
        if (restart_physics)
            impl._stop_simulation(true);
        if (rebuild_gpu_context)
            impl._stop_viewer();  // the OpenGL context is now current in this thread
        if (impl.cameras_)
            impl.cameras_->pause();
        struct Resume
        {
            Impl& impl;
            bool viewer;
            bool physics;
            bool camera_gpu_resources;
            ~Resume()
            {
                if (impl.cameras_)
                {
                    try
                    {
                        impl.cameras_->model_changed(camera_gpu_resources);
                    }
                    catch (const std::exception& e)
                    {
                        std::cerr << "DQ_MujocoInterface::edit_scene: the cameras could not be updated: "
                                  << e.what() << std::endl;
                    }
                }
                if (viewer)
                    impl._start_viewer();
                if (physics)
                    impl._start_simulation(true);
            }
        } resume{impl, rebuild_gpu_context, restart_physics, gpu_resources_change};

        impl.recompile(*scene.impl_, rebuild_gpu_context);
    }
    catch (...)
    {
        mj_deleteSpec(impl.spec_);
        impl.spec_ = backup;
        for (const auto& name : added_files)
            mj_deleteFileVFS(impl.vfs_.get(), name.c_str());
        throw;
    }
    mj_deleteSpec(backup);
}

} // namespace DQ_robotics_extensions
