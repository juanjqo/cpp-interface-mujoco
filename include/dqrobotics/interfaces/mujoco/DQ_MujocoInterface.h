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


#pragma once
#include <dqrobotics/DQ.h>
#include <memory>


using namespace DQ_robotics;
using namespace Eigen;

class DQ_MujocoInterface
{
public:
    DQ_MujocoInterface();
    DQ_MujocoInterface(const DQ_MujocoInterface&) = delete;
    DQ_MujocoInterface& operator= (const DQ_MujocoInterface&) = delete;

    void load_model(const std::string& xml_model);


    void trigger_next_simulation_step() const;
    double get_simulation_time() const;

    bool should_close_window() const;
    void render();

    VectorXd get_joint_positions(const std::vector<std::string>& jointnames) const;


    void set_joint_positions(const std::vector<std::string>& jointnames,
                                    const VectorXd& angles_rad);

    void set_joint_target_positions(const std::vector<std::string>& jointnames,
                                    const VectorXd& angles_rad);

    void set_joint_target_velocities(const std::vector<std::string>& jointnames,
                                     const VectorXd& velocities_rad_per_sec);
    VectorXd get_joint_velocities(const std::vector<std::string>& jointnames) const;

    void set_joint_target_forces(const std::vector<std::string>& jointnames,
                                 const VectorXd& forces);

    VectorXd get_joint_forces(const std::vector<std::string>& jointnames) const;


    virtual ~DQ_MujocoInterface() = default;
private:
    class Impl;
    std::shared_ptr<Impl> impl_;
    const std::string error_msg_layout_ = "Bad call in DQ_MujocoInterface::";
    void _check_model() const;


};

