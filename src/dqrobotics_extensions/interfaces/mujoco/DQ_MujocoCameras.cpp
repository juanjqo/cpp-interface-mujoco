/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2025-2026 Juan Jose Quiroz Omana

This file is part of cpp-interface-mujoco, an unofficial interface for C++17 between DQ Robotics
and MuJoCo: https://github.com/juanjqo/cpp-interface-mujoco
It uses DQ Robotics (https://dqrobotics.github.io) but is not part of DQ Robotics.
*/

#include "DQ_MujocoCameras.h"
#include "GLFW/glfw3.h"
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <thread>
#ifdef DQ_MUJOCO_HAS_EGL
#include <EGL/egl.h>
#include <EGL/eglext.h>
#endif

namespace DQ_robotics_extensions
{

using steady_clock = std::chrono::steady_clock;
using ThreadManager = marinholab::sas::core::ThreadManager;

namespace
{
// The camera thread ticks every 2 ms; each camera renders when it is due, at its own rate
constexpr double tick_period = 0.002;
constexpr double max_rate_hz = 1.0 / tick_period;
constexpr int max_scene_geoms = 10000;
}

CameraRenderer::CameraRenderer(const bool& window_mode, const mjModel* m, mjData* d, std::mutex& physics_mutex)
    : window_mode_(window_mode), m_(m), d_(d), physics_mutex_(physics_mutex)
{
    mjv_defaultOption(&option_);
    std::fill(option_.sitegroup, option_.sitegroup + mjNGROUP, 0);  // no sites (e.g., sensors)
    mjr_defaultContext(&gpu_);
}

CameraRenderer::~CameraRenderer()
{
    _stop_thread();
    for (auto& [name, camera] : cameras_)
        mjv_freeScene(&camera.scene);
    if (gpu_created_)
        _with_context([&]() { mjr_freeContext(&gpu_); });
    _destroy_context();
}

//--------------------------------------------------------------------------------------------------
// OpenGL context
//--------------------------------------------------------------------------------------------------

void CameraRenderer::_create_context()
{
    if (context_created_)
        return;
    if (window_mode_)
    {
        // GLFW creates windows in the main thread; the camera thread makes its context current
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        hidden_window_ = glfwCreateWindow(64, 64, "DQ_MujocoInterface cameras", nullptr, nullptr);
        glfwDefaultWindowHints();
        if (!hidden_window_)
            throw std::runtime_error("Failed to create the OpenGL context of the cameras (a hidden GLFW window).");
        context_created_ = true;
        return;
    }
#ifdef DQ_MUJOCO_HAS_EGL
    // Candidate displays: the GPUs (EGL device platform, independent of any display server), then
    // Mesa's surfaceless platform (e.g., software rendering), then the default display. With
    // LIBGL_ALWAYS_SOFTWARE (Mesa's request for software rendering), only Mesa's software device:
    // Mesa refuses, or fails on, the hardware devices then.
    const char* software = std::getenv("LIBGL_ALWAYS_SOFTWARE");
    const bool software_only = software && std::strcmp(software, "0") != 0 && std::strcmp(software, "false") != 0;
    std::vector<EGLDisplay> displays;
    auto query_devices = reinterpret_cast<PFNEGLQUERYDEVICESEXTPROC>(eglGetProcAddress("eglQueryDevicesEXT"));
    auto device_string = reinterpret_cast<PFNEGLQUERYDEVICESTRINGEXTPROC>(eglGetProcAddress("eglQueryDeviceStringEXT"));
    auto platform_display = reinterpret_cast<PFNEGLGETPLATFORMDISPLAYEXTPROC>(eglGetProcAddress("eglGetPlatformDisplayEXT"));
    EGLDeviceEXT devices[16];
    EGLint ndevices = 0;
    if (query_devices && platform_display && query_devices(16, devices, &ndevices))
        for (int i = 0; i < ndevices; i++)
        {
            const char* extensions = device_string ? device_string(devices[i], EGL_EXTENSIONS) : nullptr;
            const bool is_software = extensions && std::strstr(extensions, "EGL_MESA_device_software");
            if (!software_only || is_software)
                displays.push_back(platform_display(EGL_PLATFORM_DEVICE_EXT, devices[i], nullptr));
        }
#ifdef EGL_PLATFORM_SURFACELESS_MESA
    if (platform_display)
        displays.push_back(platform_display(EGL_PLATFORM_SURFACELESS_MESA, EGL_DEFAULT_DISPLAY, nullptr));
#endif
    displays.push_back(eglGetDisplay(EGL_DEFAULT_DISPLAY));

    const EGLint config_attributes[] = {
        EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8, EGL_ALPHA_SIZE, 8,
        EGL_DEPTH_SIZE, 24, EGL_STENCIL_SIZE, 8, EGL_COLOR_BUFFER_TYPE, EGL_RGB_BUFFER,
        EGL_SURFACE_TYPE, EGL_PBUFFER_BIT, EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT, EGL_NONE};
    for (EGLDisplay display : displays)
    {
        EGLint major, minor, nconfigs = 0;
        EGLConfig config;
        if (display == EGL_NO_DISPLAY || !eglInitialize(display, &major, &minor)
            || !eglChooseConfig(display, config_attributes, &config, 1, &nconfigs) || nconfigs < 1
            || !eglBindAPI(EGL_OPENGL_API))
            continue;
        EGLContext context = eglCreateContext(display, config, EGL_NO_CONTEXT, nullptr);
        if (context == EGL_NO_CONTEXT)
            continue;
        // Without a surface (MuJoCo renders to its offscreen buffer), or else with a 1x1 one
        EGLSurface surface = EGL_NO_SURFACE;
        if (!eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, context))
        {
            const EGLint pbuffer_attributes[] = {EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE};
            surface = eglCreatePbufferSurface(display, config, pbuffer_attributes);
            if (surface == EGL_NO_SURFACE || !eglMakeCurrent(display, surface, surface, context))
            {
                if (surface != EGL_NO_SURFACE)
                    eglDestroySurface(display, surface);
                eglDestroyContext(display, context);
                continue;
            }
        }
        eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        egl_display_ = display;
        egl_context_ = context;
        egl_surface_ = surface;
        context_created_ = true;
        return;
    }
    throw std::runtime_error("Failed to create the OpenGL context of the cameras with EGL: no display (GPU "
                             "device, or Mesa) gave a working OpenGL context.");
#else
    throw std::runtime_error("Cameras in HEADLESS mode need EGL, which was not found when the library was "
                             "built. Use WINDOW mode, or install EGL (e.g., libegl-dev) and build it again.");
#endif
}

void CameraRenderer::_destroy_context()
{
    if (!context_created_)
        return;
    if (hidden_window_)
        glfwDestroyWindow(hidden_window_);
    hidden_window_ = nullptr;
#ifdef DQ_MUJOCO_HAS_EGL
    // The display is not terminated: EGL displays are shared by the whole process
    if (egl_context_)
    {
        if (egl_surface_)
            eglDestroySurface(static_cast<EGLDisplay>(egl_display_), static_cast<EGLSurface>(egl_surface_));
        eglDestroyContext(static_cast<EGLDisplay>(egl_display_), static_cast<EGLContext>(egl_context_));
    }
#endif
    egl_display_ = egl_context_ = egl_surface_ = nullptr;
    context_created_ = false;
}

void CameraRenderer::_make_current() const
{
    if (hidden_window_)
    {
        glfwMakeContextCurrent(hidden_window_);
        return;
    }
#ifdef DQ_MUJOCO_HAS_EGL
    const auto surface = egl_surface_ ? static_cast<EGLSurface>(egl_surface_) : EGL_NO_SURFACE;
    eglMakeCurrent(static_cast<EGLDisplay>(egl_display_), surface, surface, static_cast<EGLContext>(egl_context_));
#endif
}

void CameraRenderer::_release_current() const
{
    if (hidden_window_)
    {
        glfwMakeContextCurrent(nullptr);
        return;
    }
#ifdef DQ_MUJOCO_HAS_EGL
    eglMakeCurrent(static_cast<EGLDisplay>(egl_display_), EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
#endif
}

template<typename F>
void CameraRenderer::_with_context(F&& f)
{
    if (hidden_window_)
    {
        GLFWwindow* previous = glfwGetCurrentContext();
        glfwMakeContextCurrent(hidden_window_);
        f();
        glfwMakeContextCurrent(previous);
        return;
    }
#ifdef DQ_MUJOCO_HAS_EGL
    _make_current();
    f();
    _release_current();
#endif
}

/**
 * @brief _make_gpu_resources makes MuJoCo's GPU resources of the cameras (meshes, textures, ...),
 *        or remakes them (rebuild), with an offscreen buffer as large as the largest camera.
 */
void CameraRenderer::_make_gpu_resources(const bool& rebuild)
{
    int width = 0, height = 0;
    for (const auto& [name, camera] : cameras_)
    {
        width = std::max(width, camera.width);
        height = std::max(height, camera.height);
    }
    _with_context([&]() {
        if (rebuild && gpu_created_)
        {
            mjr_freeContext(&gpu_);
            mjr_defaultContext(&gpu_);
            gpu_created_ = false;
        }
        if (!gpu_created_)
        {
            mjr_makeContext(m_, &gpu_, mjFONTSCALE_50);
            gpu_created_ = true;
        }
        if (gpu_.offWidth < width || gpu_.offHeight < height)
            mjr_resizeOffscreen(std::max(width, gpu_.offWidth), std::max(height, gpu_.offHeight), &gpu_);
    });
}

void CameraRenderer::_make_scene(Camera& camera)
{
    mjv_defaultScene(&camera.scene);
    mjv_makeScene(m_, &camera.scene, max_scene_geoms);
    if (camera.quality == CAMERA_QUALITY::FAST)
    {
        camera.scene.flags[mjRND_SHADOW] = 0;
        camera.scene.flags[mjRND_REFLECTION] = 0;
        camera.scene.flags[mjRND_SKYBOX] = 0;
    }
    mjv_defaultCamera(&camera.camera);
    camera.camera.type = mjCAMERA_FIXED;
    camera.camera.fixedcamid = camera.id;
}

//--------------------------------------------------------------------------------------------------
// Cameras
//--------------------------------------------------------------------------------------------------

void CameraRenderer::start_camera(const std::string& name, const double& rate_hz, const bool& with_depth,
                                  const CAMERA_QUALITY& quality)
{
    const int id = mj_name2id(m_, mjOBJ_CAMERA, name.c_str());
    if (id < 0)
        throw std::invalid_argument("There is no camera \"" + name + "\" in the model.");
    if (!(rate_hz > 0) || rate_hz > max_rate_hz)
        throw std::invalid_argument("The rate of a camera must be positive and at most "
                                    + std::to_string(static_cast<int>(max_rate_hz)) + " Hz.");

    _stop_thread();
    _create_context();
    auto it = cameras_.find(name);
    if (it != cameras_.end())
        mjv_freeScene(&it->second.scene);
    Camera& camera = cameras_[name];
    camera.id = id;
    camera.width = m_->cam_resolution[2*id];
    camera.height = m_->cam_resolution[2*id + 1];
    if (camera.width <= 1 && camera.height <= 1)  // MuJoCo's default resolution: no resolution given
    {
        camera.width = 640;
        camera.height = 480;
    }
    camera.period = std::chrono::duration_cast<steady_clock::duration>(std::chrono::duration<double>(1.0/rate_hz));
    camera.with_depth = with_depth;
    camera.quality = quality;
    camera.rgb.assign(3*camera.width*camera.height, 0);
    camera.depth.assign(with_depth ? camera.width*camera.height : 0, 0.0f);
    camera.next = steady_clock::now();
    _make_scene(camera);
    _make_gpu_resources(false);
    {
        std::lock_guard<std::mutex> lock(images_mutex_);
        images_.try_emplace(name);  // no image yet (count 0)
    }
    _start_thread();
}

void CameraRenderer::stop_camera(const std::string& name)
{
    auto it = cameras_.find(name);
    if (it == cameras_.end())
        throw std::invalid_argument("The camera \"" + name + "\" is not started.");
    _stop_thread();
    mjv_freeScene(&it->second.scene);
    cameras_.erase(it);
    {
        std::lock_guard<std::mutex> lock(images_mutex_);
        images_.erase(name);
    }
    if (!cameras_.empty())
        _start_thread();
}

CameraRenderer::CameraImage CameraRenderer::get_image(const std::string& name) const
{
    std::lock_guard<std::mutex> lock(images_mutex_);
    auto it = images_.find(name);
    if (it == images_.end())
        throw std::runtime_error("The camera \"" + name + "\" is not started (start_camera()).");
    return it->second;
}

void CameraRenderer::pause()
{
    _stop_thread();
}

void CameraRenderer::model_changed(const bool& gpu_resources_changed)
{
    // The cameras, by name (their ids, and the sizes of their scenes, can change)
    for (auto it = cameras_.begin(); it != cameras_.end();)
    {
        mjv_freeScene(&it->second.scene);
        const int id = mj_name2id(m_, mjOBJ_CAMERA, it->first.c_str());
        if (id < 0)  // removed from the scene
        {
            {
                std::lock_guard<std::mutex> lock(images_mutex_);
                images_.erase(it->first);
            }
            it = cameras_.erase(it);
            continue;
        }
        it->second.id = id;
        _make_scene(it->second);
        ++it;
    }
    if (context_created_)
        _make_gpu_resources(gpu_resources_changed);
    if (!cameras_.empty())
        _start_thread();
}

//--------------------------------------------------------------------------------------------------
// Camera thread
//--------------------------------------------------------------------------------------------------

void CameraRenderer::_start_thread()
{
    if (thread_ || cameras_.empty())
        return;
    release_requested_ = false;
    released_ = false;
    thread_ = std::make_unique<ThreadManager>("mujoco_cameras", tick_period, [this]() { _tick(); },
                                              ThreadManager::PRIORITY::BACKGROUND);
    thread_->start();
}

void CameraRenderer::_stop_thread()
{
    if (!thread_)
        return;
    // The context must be released by the thread that holds it
    release_requested_ = true;
    const auto deadline = steady_clock::now() + std::chrono::seconds(2);
    while (!released_ && steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (!released_)
        std::cerr << "DQ_MujocoInterface: the camera thread did not release its OpenGL context." << std::endl;
    thread_->stop();
    thread_.reset();
}

void CameraRenderer::_tick()
{
    if (release_requested_)
    {
        if (thread_has_context_)
        {
            _release_current();
            thread_has_context_ = false;
        }
        released_ = true;
        return;
    }
    if (!thread_has_context_)
    {
        _make_current();
        thread_has_context_ = true;
    }
    const auto now = steady_clock::now();
    for (auto& [name, camera] : cameras_)
    {
        if (now < camera.next)
            continue;
        camera.next += camera.period;
        if (camera.next < now)  // too slow for this rate: skip the images that are late
            camera.next = now + camera.period;
        _render(name, camera);
    }
}

void CameraRenderer::_render(const std::string& name, Camera& camera)
{
    double time;
    {
        std::lock_guard<std::mutex> lock(physics_mutex_);
        mjv_updateScene(m_, d_, &option_, nullptr, &camera.camera, mjCAT_ALL, &camera.scene);
        time = d_->time;
    }
    const mjrRect viewport = {0, 0, camera.width, camera.height};
    mjr_setBuffer(mjFB_OFFSCREEN, &gpu_);
    mjr_render(viewport, &camera.scene, &gpu_);
    mjr_readPixels(camera.rgb.data(), camera.with_depth ? camera.depth.data() : nullptr, viewport, &gpu_);

    // OpenGL rows go from bottom to top; images from top to bottom. The depth buffer holds values in
    // [0, 1]; the distances along the optical axis follow from the near and far clipping planes.
    const int w = camera.width, h = camera.height;
    CameraImage image;
    image.width = w;
    image.height = h;
    image.time = time;
    image.rgb.resize(3*w*h);
    for (int row = 0; row < h; row++)
        std::memcpy(&image.rgb[3*w*row], &camera.rgb[3*w*(h - 1 - row)], 3*w);
    if (camera.with_depth)
    {
        const double extent = m_->stat.extent;
        const double znear = m_->vis.map.znear*extent;
        const double zfar = m_->vis.map.zfar*extent;
        image.depth.resize(w*h);
        for (int row = 0; row < h; row++)
            for (int col = 0; col < w; col++)
            {
                const double z = camera.depth[w*(h - 1 - row) + col];
                image.depth[w*row + col] = static_cast<float>(znear/(1.0 - z*(1.0 - znear/zfar)));
            }
    }
    std::lock_guard<std::mutex> lock(images_mutex_);
    CameraImage& latest = images_[name];
    image.count = latest.count + 1;
    latest = std::move(image);
}

}
