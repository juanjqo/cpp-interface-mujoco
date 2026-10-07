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
#include <string>
#include <vector>

namespace DQ_robotics_extensions
{

class DQ_MujocoInterface
{
public:
    /**
     * @brief How the simulation is visualized.
     *  WINDOW:   load_model() opens a window, and render() draws the scene in it.
     *  HEADLESS: no window is opened and render() does nothing. Use it to run simulations
     *            without a display (e.g., automated tests, CI, or batch experiments).
     */
    enum class RENDERING
    {
        WINDOW,
        HEADLESS
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


    void trigger_next_simulation_step();
    double get_simulation_time() const;

    bool should_close_window() const;
    void render();

    //Visualization tools
    void set_wireframe_view(const bool& flag);
    void set_joint_view(const bool& flag);
    void set_contact_view(const bool &flag);
    void set_center_of_mass_view(const bool& flag);
    void set_site_frame_view(const bool& flag);


    void reset_simulation();




    std::vector<std::string> get_joint_names() const;

    // Joint space methods
    Eigen::VectorXd get_joint_positions(const std::vector<std::string>& jointnames) const;
    void            set_joint_positions(const std::vector<std::string>& jointnames, const Eigen::VectorXd& angles_rad);
    void            set_joint_target_positions(const std::vector<std::string>& jointnames, const Eigen::VectorXd& angles_rad);
    void            set_joint_target_velocities(const std::vector<std::string>& jointnames,const Eigen::VectorXd& velocities_rad_per_sec);
    Eigen::VectorXd get_joint_velocities(const std::vector<std::string>& jointnames) const;
    void            set_joint_target_forces(const std::vector<std::string>& jointnames, const Eigen::VectorXd& forces);
    Eigen::VectorXd get_joint_forces(const std::vector<std::string>& jointnames) const;

    DQ_robotics::DQ get_site_pose(const std::string& objectname) const;
    void set_site_pose(const std::string& objectname, const DQ_robotics::DQ& pose);

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
