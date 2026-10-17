/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2025-2026 Juan Jose Quiroz Omana

This file is part of cpp-interface-mujoco, an unofficial interface for C++17 between DQ Robotics
and MuJoCo: https://github.com/juanjqo/cpp-interface-mujoco
It uses DQ Robotics (https://dqrobotics.github.io) but is not part of DQ Robotics.
*/

// Internal header (not installed): the implementation of DQ_MujocoSceneBuilder, also used by
// DQ_MujocoInterface::edit_scene().

#pragma once
#include <dqrobotics_extensions/interfaces/mujoco/DQ_MujocoSceneBuilder.h>
#include "mujoco/mujoco.h"
#include <map>
#include <string>
#include <vector>

namespace DQ_robotics_extensions
{

class DQ_MujocoSceneBuilder::Impl
{
public:
    mjSpec* spec_ = nullptr;
    bool owns_spec_ = true;  // false when it edits the scene of a DQ_MujocoInterface

    // The "home" keyframe of the scene, by name, so that it survives changes of the joint order:
    // the configuration of each named joint, the pose of each free body (7 values, in the world),
    // and the control of each named actuator. Everything else is at its initial value (qpos0).
    std::map<std::string, std::vector<double>> home_qpos_;
    std::map<std::string, std::vector<double>> home_free_pose_;
    std::map<std::string, double> home_ctrl_;

    // Keyframes of the added robots (with their prefixes), replaced by the "home" keyframe. They
    // can only be deleted after a compilation: MuJoCo keeps the keyframes of attached models (and
    // all keyframes, when a body is deleted) pending until the next compilation.
    std::vector<std::string> dropped_keyframes_;

    // The files (meshes, textures, ...) of the robots added by this builder, by their path in the
    // scene: robot_assets/<name>_<hash of the content>.<extension>, so that the same file is
    // stored once, and different files never share a path (or a file name, which MuJoCo's virtual
    // file system also matches). build() returns them; edit_scene() adds them to the interface.
    std::map<std::string, std::vector<unsigned char>> assets_;

    static constexpr const char* home_keyframe_name_ = "home";

    // A new, empty scene
    explicit Impl(const std::string& name);

    // The scene of a loaded model: spec compiled into model, whose "home" keyframe (if any) is kept
    Impl(mjSpec* spec, const mjModel* model);

    ~Impl();
    Impl(const Impl&) = delete;
    Impl& operator=(const Impl&) = delete;

    mjsBody* world() const;
    void check_name_is_free(const mjtObj& type, const std::string& name) const;

    void add_primitive(const std::string& name, const DQ_robotics::DQ& pose, const mjtGeom& type,
                       const double size[3], const BODY& body, const Eigen::Vector4d& rgba,
                       const double& mass);
    DQ_MujocoRobotDescription add_robot(const DQ_MujocoRobotDescription& robot,
                                        const std::string& prefix, const DQ_robotics::DQ& pose);
    void remove(const std::string& name);

    // Reads the "home" values from a keyframe of a model
    void read_home(const mjModel* model, const int& keyframe_id);

    // Called right after spec was compiled into model, when all its keyframes exist: deletes the
    // keyframes of the robots and the old "home" keyframe, and adds a "home" keyframe for the
    // layout of model. The spec must then be compiled again.
    void fix_keyframes(mjSpec* spec, const mjModel* model) const;
    void add_home(mjSpec* spec, const mjModel* model) const;

    // Compiles a copy of the scene with its "home" keyframe and returns it as MJCF
    std::string to_xml() const;
};

}
