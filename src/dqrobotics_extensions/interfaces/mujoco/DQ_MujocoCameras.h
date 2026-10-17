/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2025-2026 Juan Jose Quiroz Omana

This file is part of cpp-interface-mujoco, an unofficial interface for C++17 between DQ Robotics
and MuJoCo: https://github.com/juanjqo/cpp-interface-mujoco
It uses DQ Robotics (https://dqrobotics.github.io) but is not part of DQ Robotics.
*/

// Internal header (not installed): the camera renderer of DQ_MujocoInterface.

#pragma once
#include <dqrobotics_extensions/interfaces/mujoco/DQ_MujocoInterface.h>
#include "mujoco/mujoco.h"
#include <marinholab/sas/core/sas_thread_manager.hpp>
#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

struct GLFWwindow;

namespace DQ_robotics_extensions
{

/**
 * @brief Renders the cameras of a model offscreen, each at its own rate, in a camera thread with
 *        its own OpenGL context: a hidden GLFW window in WINDOW mode (next to the context of the
 *        viewer), or EGL in HEADLESS mode (no display needed).
 *
 * The thread holds the physics lock only to copy the state into the scene of a camera (a few
 * microseconds); it renders and reads the pixels back without it, so it never delays the physics.
 *
 * The context and the GPU resources are created in the calling (main) thread, so that errors are
 * reported at once; the thread is stopped while they change (start_camera(), stop_camera(), and
 * around a recompilation of the model), as the viewer is.
 */
class CameraRenderer
{
public:
    using CameraImage = DQ_MujocoInterface::CameraImage;
    using CAMERA_QUALITY = DQ_MujocoInterface::CAMERA_QUALITY;

    /**
     * @param window_mode True in WINDOW mode (GLFW is initialized): the context is a hidden window.
     * @param m, d The model and data of the interface. They must outlive the renderer.
     * @param physics_mutex The lock of d (and of the mutable parts of m).
     */
    CameraRenderer(const bool& window_mode, const mjModel* m, mjData* d, std::mutex& physics_mutex);
    ~CameraRenderer();  // main thread
    CameraRenderer(const CameraRenderer&) = delete;
    CameraRenderer& operator=(const CameraRenderer&) = delete;

    // Main thread
    void start_camera(const std::string& name, const double& rate_hz, const bool& with_depth,
                      const CAMERA_QUALITY& quality);
    void stop_camera(const std::string& name);

    // Any thread
    CameraImage get_image(const std::string& name) const;

    // Around a recompilation of the model (edit_scene()), main thread: pause() stops the thread;
    // model_changed() updates the cameras (by name: a camera that no longer exists is stopped), and
    // the GPU resources if the meshes, textures, ... changed, and restarts the thread
    void pause();
    void model_changed(const bool& gpu_resources_changed);

private:
    struct Camera
    {
        int id = -1;
        int width = 0;
        int height = 0;
        std::chrono::steady_clock::duration period{};
        bool with_depth = false;
        CAMERA_QUALITY quality = CAMERA_QUALITY::FULL;
        mjvScene scene;
        mjvCamera camera;
        std::chrono::steady_clock::time_point next;
        std::vector<unsigned char> rgb;  // as read from OpenGL (rows from bottom to top)
        std::vector<float> depth;
    };

    const bool window_mode_;
    const mjModel* m_;
    mjData* d_;  // not const: mjv_updateScene() takes it so
    std::mutex& physics_mutex_;
    mjvOption option_;  // what the cameras show: the visual geometries, no sites, frames or labels

    // OpenGL context: a hidden GLFW window, or EGL (opaque handles, see DQ_MujocoCameras.cpp)
    GLFWwindow* hidden_window_ = nullptr;
    void* egl_display_ = nullptr;
    void* egl_context_ = nullptr;
    void* egl_surface_ = nullptr;
    bool context_created_ = false;
    mjrContext gpu_;
    bool gpu_created_ = false;

    std::map<std::string, Camera> cameras_;  // changed only while the thread is stopped

    // Latest image of each camera
    std::map<std::string, CameraImage> images_;
    mutable std::mutex images_mutex_;

    // Camera thread
    std::unique_ptr<marinholab::sas::core::ThreadManager> thread_;
    std::atomic<bool> release_requested_{false};
    std::atomic<bool> released_{false};
    bool thread_has_context_ = false;  // only used by the thread

    void _create_context();
    void _destroy_context();
    void _make_current() const;
    void _release_current() const;
    // Runs f with the context current in the calling (main) thread, and restores the context that
    // was current before (e.g., that of the window, while edit_scene() rebuilds the viewer)
    template<typename F>
    void _with_context(F&& f);
    void _make_gpu_resources(const bool& rebuild);
    void _make_scene(Camera& camera);

    void _start_thread();
    void _stop_thread();
    void _tick();
    void _render(const std::string& name, Camera& camera);
};

}
