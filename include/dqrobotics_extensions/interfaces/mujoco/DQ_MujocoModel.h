/**
SPDX-License-Identifier: LGPL-3.0-or-later
Copyright (C) 2025-2026 Juan Jose Quiroz Omana

This file is part of cpp-interface-mujoco, an unofficial interface for C++17 between DQ Robotics
and MuJoCo: https://github.com/juanjqo/cpp-interface-mujoco
It uses DQ Robotics (https://dqrobotics.github.io) but is not part of DQ Robotics.
*/


#pragma once
#include <map>
#include <string>
#include <vector>

namespace DQ_robotics_extensions
{

/**
 * @brief A MuJoCo model held in memory: the MJCF (XML) text and the files it refers to.
 *
 * Load it with DQ_MujocoInterface::load_model(const DQ_MujocoModel&). No file is read from disk:
 * MuJoCo resolves every file name in the XML (meshes, textures, <include> files) from assets.
 *
 * Example:
 * @code
 * DQ_MujocoModel model;
 * model.name = "robot.xml";
 * model.xml = R"(<mujoco>
 *                  <compiler meshdir="meshes"/>
 *                  <asset><mesh file="link.obj"/></asset>
 *                  ...
 *                </mujoco>)";
 * model.assets["meshes/link.obj"] = obj_bytes;  // the path as MuJoCo resolves it (meshdir + file)
 * mi.load_model(model);
 * @endcode
 */
struct DQ_MujocoModel
{
    /// File name of the XML, e.g., "robot.xml". MuJoCo resolves the asset paths relative to it.
    std::string name;

    /// MJCF (XML) text of the model.
    std::string xml;

    /// Files referenced by the XML, keyed by their path relative to the XML file, as MuJoCo resolves
    /// it (including <compiler meshdir/texturedir> prefixes), e.g., "assets/trunk.stl".
    std::map<std::string, std::vector<unsigned char>> assets;
};

}
