/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2025-2026 Juan Jose Quiroz Omana

This file is part of cpp-interface-mujoco, an unofficial interface for C++17 between DQ Robotics
and MuJoCo: https://github.com/juanjqo/cpp-interface-mujoco
It uses DQ Robotics (https://dqrobotics.github.io) but is not part of DQ Robotics.
*/


#pragma once
#include <dqrobotics/DQ.h>
#include <dqrobotics_extensions/interfaces/mujoco/DQ_MujocoModel.h>
#include <dqrobotics_extensions/interfaces/mujoco/DQ_MujocoSceneBuilder.h>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace DQ_robotics_extensions
{

class DQ_MujocoInterface
{
public:
    /**
     * @brief How the simulation is visualized.
     *  WINDOW:   load_model() opens a window, which a viewer thread draws at 60 Hz.
     *  HEADLESS: no window is opened. Use it to run simulations without a display (e.g.,
     *            automated tests, CI, or batch experiments).
     */
    enum class RENDERING
    {
        WINDOW,
        HEADLESS
    };

    /**
     * @brief Frame in which a twist is expressed (see get_base_twist() and set_base_twist()).
     *  WORLD_FRAME: world frame, reference point at the world origin (x_dot = 0.5*xi*x).
     *  BODY_FRAME:  body frame, reference point at the body origin (x_dot = 0.5*x*xi).
     */
    enum class REFERENCE_FRAME
    {
        BODY_FRAME,
        WORLD_FRAME
    };

    /**
     * @brief How a camera renders its images (see start_camera()).
     *  FULL: as the viewer, with shadows, reflections and the skybox.
     *  FAST: without them: about 2 times faster on a GPU, and 3 times faster with software rendering.
     */
    enum class CAMERA_QUALITY
    {
        FULL,
        FAST
    };

    /**
     * @brief An image of a camera (see get_camera_image()).
     *
     * The layout is that of a ROS sensor_msgs/Image with encoding "rgb8" (and "32FC1" for the depth):
     * rows from top to bottom, pixels from left to right.
     */
    struct CameraImage
    {
        int width = 0;
        int height = 0;
        std::vector<std::uint8_t> rgb;  ///< 3*width*height bytes: R, G, B of each pixel
        std::vector<float> depth;       ///< width*height distances in meters along the optical axis
                                        ///< (empty if the camera was started without depth)
        double time = 0;                ///< simulation time of the state in the image, in seconds
        std::uint64_t count = 0;        ///< images rendered by the camera so far (a new image has a new count)
    };

    /**
     * @brief DQ_MujocoInterface
     * @param rendering WINDOW (default) or HEADLESS.
     * @param window_title Title of the window. If empty, the name of the model is used.
     * @param window_width Width of the window in screen coordinates.
     * @param window_height Height of the window in screen coordinates.
     */
    explicit DQ_MujocoInterface(const RENDERING& rendering = RENDERING::WINDOW,
                                const std::string& window_title = "",
                                const int& window_width = 1280,
                                const int& window_height = 920);
    DQ_MujocoInterface(const DQ_MujocoInterface&) = delete;
    DQ_MujocoInterface& operator= (const DQ_MujocoInterface&) = delete;

    void load_model(const std::string& xml_model);
    void load_model(const DQ_MujocoModel& model);
    // Edit the loaded scene (add/remove objects, robots, ...), also while it runs; the state is kept.
    // Each edit stalls the simulation while it is recompiled (longer with meshes): prefer building
    // the scene before the control loop starts.
    void edit_scene(const std::function<void(DQ_MujocoSceneBuilder&)>& edit);


    // Two ways to run the simulation (the window, if any, is drawn by a viewer thread in both):
    //  - Real time: start_simulation() starts a physics thread that keeps the simulation in real
    //    time, like a real robot; the program reads the state and sends commands at its own rate.
    //  - Stepped: trigger_next_simulation_step() advances one timestep (when not running).
    void start_simulation();
    void stop_simulation();
    bool is_simulation_running() const;
    double get_real_time_factor() const;

    void trigger_next_simulation_step();
    double get_simulation_time() const;

    // Processes the window events: call it regularly from the main thread (e.g., as the loop condition)
    bool should_close_window() const;

    //Visualization tools
    void set_wireframe_view(const bool& flag);
    void set_joint_view(const bool& flag);
    void set_contact_view(const bool &flag);
    void set_center_of_mass_view(const bool& flag);
    void set_site_frame_view(const bool& flag);
    void set_camera_tracking(const std::string& bodyname);

    // GUI of the viewer (WINDOW mode): docked panels around the 3D view (scene tree, properties,
    // joints, sensors, visualization options) and the simulation controls. Off by default.
    static bool is_gui_available();
    void set_gui_enabled(const bool& flag);
    bool is_gui_enabled() const;


    void reset_simulation();
    void reset_simulation(const std::string& keyframe);




    std::vector<std::string> get_joint_names(const bool& only_single_dof = false) const;

    // Joint space methods
    Eigen::VectorXd get_joint_positions(const std::vector<std::string>& jointnames) const;
    void            set_joint_positions(const std::vector<std::string>& jointnames, const Eigen::VectorXd& angles_rad);
    void            set_joint_target_positions(const std::vector<std::string>& jointnames, const Eigen::VectorXd& angles_rad);
    void            set_joint_target_velocities(const std::vector<std::string>& jointnames,const Eigen::VectorXd& velocities_rad_per_sec);
    Eigen::VectorXd get_joint_velocities(const std::vector<std::string>& jointnames) const;
    void            set_joint_target_forces(const std::vector<std::string>& jointnames, const Eigen::VectorXd& forces);
    Eigen::VectorXd get_joint_forces(const std::vector<std::string>& jointnames) const;

    // Floating bases (humanoids, quadrupeds, free objects), addressed by the body that owns the free joint
    std::vector<std::string> get_floating_base_names() const;
    DQ_robotics::DQ get_base_pose(const std::string& bodyname) const;
    void            set_base_pose(const std::string& bodyname, const DQ_robotics::DQ& pose);
    // WORLD_FRAME: xi = w + E_*(p_dot + cross(p, w)), all in the world frame (x_dot = 0.5*xi*x)
    // BODY_FRAME:  xi_body = Ad(conj(x), xi), velocities of the body origin in the body frame (x_dot = 0.5*x*xi_body)
    DQ_robotics::DQ get_base_twist(const std::string& bodyname,
                                   const REFERENCE_FRAME& reference_frame = REFERENCE_FRAME::WORLD_FRAME) const;
    void            set_base_twist(const std::string& bodyname,
                                   const DQ_robotics::DQ& twist,
                                   const REFERENCE_FRAME& reference_frame = REFERENCE_FRAME::WORLD_FRAME);

    DQ_robotics::DQ get_body_pose(const std::string& bodyname) const;
    // Moves a KINEMATIC body (a MuJoCo mocap body), also while the simulation runs
    void            set_body_pose(const std::string& bodyname, const DQ_robotics::DQ& pose);

    DQ_robotics::DQ get_site_pose(const std::string& objectname) const;
    void set_site_pose(const std::string& objectname, const DQ_robotics::DQ& pose);

    // Sensors (MuJoCo <sensor> elements, or added with DQ_MujocoSceneBuilder::add_imu(), ...), also
    // while the simulation runs. Any sensor by its name, or the common ones by their site:
    std::vector<std::string> get_sensor_names() const;
    Eigen::VectorXd get_sensor_data(const std::string& sensorname) const;
    DQ_robotics::DQ get_imu_orientation(const std::string& sitename) const;          // unit quaternion
    DQ_robotics::DQ get_imu_angular_velocity(const std::string& sitename) const;     // in the IMU frame
    DQ_robotics::DQ get_imu_linear_acceleration(const std::string& sitename) const;  // in the IMU frame
    DQ_robotics::DQ get_wrench(const std::string& sitename) const;                   // f + E_*tau
    double          get_touch(const std::string& sitename) const;                    // normal force
    double          get_range(const std::string& sitename) const;                    // distance, -1: none

    // Cameras (MuJoCo <camera> elements, or added with DQ_MujocoSceneBuilder::add_camera()): rendered
    // offscreen by a camera thread, each at its own rate, in WINDOW and HEADLESS mode. Start and stop
    // them from the main thread; read their images from any thread.
    std::vector<std::string> get_camera_names() const;
    void start_camera(const std::string& cameraname,
                      const double& rate_hz = 30.0,
                      const bool& with_depth = false,
                      const CAMERA_QUALITY& quality = CAMERA_QUALITY::FULL);
    void stop_camera(const std::string& cameraname);
    CameraImage get_camera_image(const std::string& cameraname) const;

    // Reference frames: created at runtime and only drawn in the viewer (no XML, no physics)
    void add_reference_frame(const std::string& name,
                             const double& axis_length = 0.1,
                             const double& axis_width = 0.005,
                             const double& opacity = 1.0,
                             const bool& show_label = true);
    void set_reference_frame_pose(const std::string& name, const DQ_robotics::DQ& pose);
    DQ_robotics::DQ get_reference_frame_pose(const std::string& name) const;
    void remove_reference_frame(const std::string& name);


    virtual ~DQ_MujocoInterface();
private:
    class Impl;
    std::unique_ptr<Impl> impl_;
    const std::string error_msg_layout_ = "Bad call in DQ_MujocoInterface::";
    void _check_model() const;


};

}
