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
#include <dqrobotics_extensions/interfaces/mujoco/DQ_MujocoRobotDescription.h>
#include <memory>
#include <string>

namespace DQ_robotics_extensions
{

/**
 * @brief Builds a MuJoCo scene in code: a floor, lights, primitive objects, sites and robots of the
 *        robots library, posed with unit dual quaternions.
 *
 * build() returns a DQ_MujocoModel, which DQ_MujocoInterface::load_model() loads. The same class
 * edits a loaded scene while it runs, with DQ_MujocoInterface::edit_scene().
 *
 * Example:
 * @code
 * DQ_MujocoSceneBuilder scene("pick and place");
 * scene.add_floor();
 * scene.add_box("table", x_table, {0.4, 0.3, 0.02});
 * scene.add_sphere("ball", x_ball, 0.03, DQ_MujocoSceneBuilder::BODY::FREE, {1, 0, 0, 1}, 0.1);
 * const auto arm = scene.add_robot(GenericManipulatorRobot::description(), "arm_", x_arm);
 *
 * DQ_MujocoInterface mi;
 * mi.load_model(scene.build());
 * mi.reset_simulation("home");   // every robot at its home configuration, where it was placed
 * @endcode
 *
 * The scene has a "home" keyframe: each robot added with add_robot() at its own home keyframe
 * (moved to where the robot was placed), and everything else at its initial pose.
 */
class DQ_MujocoSceneBuilder
{
public:
    /**
     * @brief How an object moves.
     *  FIXED:     welded to the world; it never moves.
     *  KINEMATIC: moved by the program, not by the physics: DQ_MujocoInterface::set_body_pose()
     *             sets its pose (also while the simulation runs). Gravity and contacts do not move
     *             it, but it collides with and pushes FREE objects and robots (a MuJoCo mocap body).
     *             E.g., a moving obstacle, a platform or a target that collides.
     *  FREE:      has a free joint, so it falls, collides and can be pushed. It is a floating base:
     *             use get_base_pose(), set_base_twist(), etc. with its name.
     */
    enum class BODY
    {
        FIXED,
        KINEMATIC,
        FREE
    };

    /**
     * @brief The numerical integrator (MuJoCo's <option integrator>). IMPLICITFAST, the default of
     *        the builder, is stable with stiff actuators and damping.
     */
    enum class INTEGRATOR
    {
        EULER,
        RK4,
        IMPLICIT,
        IMPLICITFAST
    };

    explicit DQ_MujocoSceneBuilder(const std::string& name = "scene");
    ~DQ_MujocoSceneBuilder();
    DQ_MujocoSceneBuilder(const DQ_MujocoSceneBuilder&) = delete;
    DQ_MujocoSceneBuilder& operator=(const DQ_MujocoSceneBuilder&) = delete;
    DQ_MujocoSceneBuilder(DQ_MujocoSceneBuilder&&) noexcept;
    DQ_MujocoSceneBuilder& operator=(DQ_MujocoSceneBuilder&&) noexcept;

    // Physics options of the scene (the options of the robots are not used)
    void set_timestep(const double& timestep);
    void set_integrator(const INTEGRATOR& integrator);
    void set_gravity(const Eigen::Vector3d& gravity);

    // Environment
    void add_floor(const std::string& name = "floor");
    void add_light(const std::string& name,
                   const DQ_robotics::DQ& position,
                   const DQ_robotics::DQ& direction = DQ_robotics::DQ(0, 0, 0, -1),
                   const bool& directional = false);

    // Objects: a body with a single geometry, named name (the body and the geometry)
    void add_box(const std::string& name, const DQ_robotics::DQ& pose,
                 const Eigen::Vector3d& half_sizes,
                 const BODY& body = BODY::FIXED,
                 const Eigen::Vector4d& rgba = Eigen::Vector4d(0.7, 0.7, 0.7, 1.0),
                 const double& mass = 1.0);
    void add_sphere(const std::string& name, const DQ_robotics::DQ& pose,
                    const double& radius,
                    const BODY& body = BODY::FIXED,
                    const Eigen::Vector4d& rgba = Eigen::Vector4d(0.7, 0.7, 0.7, 1.0),
                    const double& mass = 1.0);
    void add_cylinder(const std::string& name, const DQ_robotics::DQ& pose,
                      const double& radius, const double& half_length,
                      const BODY& body = BODY::FIXED,
                      const Eigen::Vector4d& rgba = Eigen::Vector4d(0.7, 0.7, 0.7, 1.0),
                      const double& mass = 1.0);
    void add_capsule(const std::string& name, const DQ_robotics::DQ& pose,
                     const double& radius, const double& half_length,
                     const BODY& body = BODY::FIXED,
                     const Eigen::Vector4d& rgba = Eigen::Vector4d(0.7, 0.7, 0.7, 1.0),
                     const double& mass = 1.0);

    // A site in the world: a frame without physics, e.g., a target moved with set_site_pose()
    void add_site(const std::string& name, const DQ_robotics::DQ& pose,
                  const double& size = 0.02,
                  const Eigen::Vector4d& rgba = Eigen::Vector4d(1.0, 0.0, 0.0, 0.5));

    // Robots of the robots library (or any robot described by a DQ_MujocoRobotDescription)
    DQ_MujocoRobotDescription add_robot(const DQ_MujocoRobotDescription& robot,
                                        const std::string& prefix,
                                        const DQ_robotics::DQ& pose = DQ_robotics::DQ(1));

    // Remove an object, a site, a light, the floor, or a robot (by the prefixed name of its root body)
    void remove(const std::string& name);

    DQ_MujocoModel build() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
    inline static const std::string error_msg_layout_ = "Bad call in DQ_MujocoSceneBuilder::";

    // DQ_MujocoInterface::edit_scene() edits the scene of the interface through a builder
    friend class DQ_MujocoInterface;
    explicit DQ_MujocoSceneBuilder(std::unique_ptr<Impl> impl);
};

}
