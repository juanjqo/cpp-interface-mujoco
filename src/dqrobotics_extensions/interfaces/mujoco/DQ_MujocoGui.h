/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2025-2026 Juan Jose Quiroz Omana

This file is part of cpp-interface-mujoco, an unofficial interface for C++17 between DQ Robotics
and MuJoCo: https://github.com/juanjqo/cpp-interface-mujoco
It uses DQ Robotics (https://dqrobotics.github.io) but is not part of DQ Robotics.
*/

#pragma once
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <vector>
#include "mujoco/mujoco.h"

struct ImGuiContext;

namespace DQ_robotics_extensions
{

/**
 * @brief MujocoGui draws the GUI of the viewer (Dear ImGui): docked panels around the 3D view
 *        (scene tree, properties of the selected body, joints, sensors, visualization options),
 *        a menu bar with the simulation controls, and a status bar.
 *
 * Threads (see DQ_MujocoInterface::Impl):
 * - The main thread records the window events (on_*(), from the GLFW callbacks), since GLFW
 *   only processes events there, and runs the requests of the GUI (take_requests()).
 * - The viewer thread, with the OpenGL context, draws the GUI (begin_frame(), render()). ImGui is
 *   only used by this thread, so the platform side of ImGui is implemented here instead of with
 *   imgui_impl_glfw, which calls GLFW functions that are only allowed in the main thread.
 *
 * ImGui is not exposed: this header does not include it.
 */
class MujocoGui
{
public:
    /**
     * @brief Simulation state shown by the GUI, copied from mjData by the viewer thread under
     *        the physics lock (the panels are drawn without it).
     */
    struct State
    {
        double time = 0;
        std::vector<mjtNum> qpos, qvel, xpos, xquat, sensordata;
        void copy(const mjModel* m, const mjData* d);
    };

    struct Statistics
    {
        bool running = false;
        double real_time_factor = 0;
        double fps = 0;
        double step_time_us = 0;
    };

    /**
     * @brief What the viewer thread gives the GUI to draw a frame. The caller holds the viewer lock,
     *        which protects m (its structure), opt, scn, cam and tracking_body.
     */
    struct Frame
    {
        const mjModel* m;
        const State& state;
        mjvOption& opt;
        mjvScene& scn;
        mjvCamera& cam;
        std::string& tracking_body;  // body followed by the camera (empty: none)
        Statistics statistics;
        int window_width, window_height;            // screen coordinates (mouse, layout)
        int framebuffer_width, framebuffer_height;  // pixels (rendering)
    };

    /**
     * @brief Actions asked in the GUI that change the simulation. The main thread runs them, as if
     *        the user called the methods of DQ_MujocoInterface.
     */
    struct Requests
    {
        bool toggle_simulation = false;  // start_simulation() or stop_simulation()
        bool step = false;               // trigger_next_simulation_step(), if not running
        bool reset = false;              // reset_simulation(), to reset_keyframe if >= 0
        int reset_keyframe = -1;
    };

    /**
     * @brief A click in the 3D view (not a drag of the camera), to select the body under the cursor
     *        with mjv_select(): the point relative to the 3D view, from its bottom-left corner.
     */
    struct Pick
    {
        double relx = 0, rely = 0;
        double aspect_ratio = 1;
    };

    /**
     * @brief The mouse interaction with the selected body in the 3D view, in the last frame, for
     *        MuJoCo's perturbation (mjv_initPerturb(), mjv_movePerturb()): Ctrl + left drag rotates
     *        the body, Ctrl + right drag moves it (with Shift: in the horizontal plane).
     */
    struct PerturbInput
    {
        bool start = false;  // a drag started: perturb body, held at localpos (in the body frame)
        bool end = false;    // the drag ended
        int body = -1;
        int active = 0;      // mjPERT_ROTATE or mjPERT_TRANSLATE
        mjtNum localpos[3] = {0, 0, 0};
        int action = mjMOUSE_NONE;  // the motion of the mouse in this frame, relative to the
        double dx = 0, dy = 0;      // height of the 3D view
    };

    /**
     * @param content_scale Scale of the contents of the window (e.g., 2 on a high-DPI display).
     */
    explicit MujocoGui(const float& content_scale);
    // Needs the OpenGL context current in the calling thread if the GUI was drawn
    ~MujocoGui();
    MujocoGui(const MujocoGui&) = delete;
    MujocoGui& operator=(const MujocoGui&) = delete;

    // Main thread: window events (GLFW key codes and modifiers, screen coordinates)
    void on_mouse_button(const int& button, const bool& down, const int& glfw_mods);
    void on_mouse_move(const double& x, const double& y);
    void on_scroll(const double& dx, const double& dy);
    void on_key(const int& glfw_key, const int& glfw_mods, const bool& down);
    void on_char(const unsigned int& codepoint);
    void on_focus(const bool& focused);
    // Whether the GUI uses the mouse (it is over a panel), as of the last frame: the 3D view then
    // ignores it
    bool wants_mouse() const { return wants_mouse_; }
    // Whether a body is selected: Ctrl + drag in the 3D view is then for the body, not the camera
    bool has_selection() const { return has_selection_; }
    Requests take_requests();

    // Viewer thread, with the OpenGL context current and the viewer lock held
    /**
     * @brief begin_frame clears the window and builds the GUI of a frame.
     * @return The rectangle of the 3D view, in pixels, for mjr_render().
     */
    mjrRect begin_frame(const Frame& frame);
    void render();  // draws the GUI over the 3D view

    // Viewer thread, with the viewer lock held: the selected body (-1: none)
    int selected_body() const { return selected_body_; }
    // From the 3D view (the Scene panel reveals it), with the clicked point in the body frame
    void select_body(const int& body, const mjtNum local_point[3]);
    bool take_pick(Pick& pick);         // the last click in the 3D view, if any
    const PerturbInput& perturb_input() const { return perturb_; }
    // Makes the geoms of the selected body and of its children glow, in the selection color (amber) (call it
    // after mjv_updateScene(), which makes the geoms of the frame)
    void highlight_selection(const mjModel* m, mjvScene& scn) const;

private:
    struct Event
    {
        enum class Type { MOUSE_BUTTON, MOUSE_MOVE, SCROLL, KEY, CHAR, FOCUS } type;
        int a = 0, b = 0;
        double x = 0, y = 0;
    };

    ImGuiContext* context_ = nullptr;
    bool renderer_initialized_ = false;
    float content_scale_ = 1;
    std::chrono::steady_clock::time_point last_frame_;

    // Events recorded by the main thread, given to ImGui by the viewer thread
    std::mutex events_mutex_;
    std::vector<Event> events_;
    std::atomic<bool> wants_mouse_{false};
    std::atomic<bool> has_selection_{false};

    std::mutex requests_mutex_;
    Requests requests_;

    // State of the panels (viewer thread only)
    int selected_body_ = -1;
    bool reveal_selection_ = false;  // open the tree of the Scene panel at the selected body
    bool click_in_view_ = false;     // the left button was pressed in the 3D view
    bool pick_requested_ = false;
    Pick pick_;
    mjtNum picked_point_[3] = {0, 0, 0};  // the clicked point of picked_body_, in its frame
    int picked_body_ = -1;
    PerturbInput perturb_;
    int perturb_button_ = -1;  // the button that drags the body (-1: none)
    bool show_scene_ = true;
    bool show_properties_ = true;
    bool show_joints_ = true;
    bool show_sensors_ = true;
    bool show_visualization_ = true;
    bool reset_layout_ = true;
    char filter_[128] = "";

    void _push(const Event& event);
    void _process_events();
    void _apply_style();
    void _request(const Requests& request);

    void _menu_bar(const Frame& frame);
    void _status_bar(const Frame& frame);
    void _dock_layout(const unsigned int& dockspace_id);
    void _scene_panel(const Frame& frame);
    void _properties_panel(const Frame& frame);
    void _joints_panel(const Frame& frame);
    void _sensors_panel(const Frame& frame);
    void _visualization_panel(const Frame& frame);
    void _shortcuts(const Frame& frame);
};

}
