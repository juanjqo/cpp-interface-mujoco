/**
Tests for DQ_MujocoInterface.

Each test is a function registered in tests_. Run all of them with
    ./test_interface
or a single one (this is how CTest calls them) with
    ./test_interface <test_name>

The tests must run from the directory that contains the test models (CTest does this).
Most tests run in HEADLESS mode. The ones that check the window (load_model_twice,
two_interfaces_at_once) need a display; on a headless machine, use xvfb-run.
*/

#include "dqrobotics_extensions/interfaces/mujoco/DQ_MujocoInterface.h"
#include <GLFW/glfw3.h>
#include <cmath>
#include <functional>
#include <iostream>
#include <map>

using namespace DQ_robotics;
using namespace DQ_robotics_extensions;
using namespace Eigen;

static constexpr auto HEADLESS = DQ_MujocoInterface::RENDERING::HEADLESS;

/*********************************************
 * MINIMAL TEST HELPERS
 * *******************************************/

#define CHECK(condition)                                                                    \
    if (!(condition))                                                                       \
        throw std::runtime_error(std::string("CHECK(") + #condition + ") failed at line "   \
                                 + std::to_string(__LINE__));

#define CHECK_NEAR(a, b)                                                                    \
    if (std::abs((a) - (b)) > 1e-6)                                                         \
        throw std::runtime_error(std::string("CHECK_NEAR(") + #a + ", " + #b + ") failed at line " \
                                 + std::to_string(__LINE__) + ": " + std::to_string(a)      \
                                 + " != " + std::to_string(b));

#define CHECK_THROWS(exception_type, statement)                                             \
    {                                                                                       \
        bool thrown = false;                                                                \
        try { statement; } catch (const exception_type&) { thrown = true; }                 \
        if (!thrown)                                                                        \
            throw std::runtime_error(std::string("CHECK_THROWS(") + #exception_type         \
                                     + ") failed at line " + std::to_string(__LINE__));     \
    }

static double pose_error(const DQ& a, const DQ& b)
{
    return vec8(a - b).norm();
}

static VectorXd vec(std::initializer_list<double> values)
{
    VectorXd v(values.size());
    int i = 0;
    for (double value : values)
        v(i++) = value;
    return v;
}

static DQ test_pose()
{
    DQ r = cos(M_PI/8) + (0.6*i_ + 0.8*k_)*sin(M_PI/8);
    return r + 0.5*E_*(0.2*i_ - 0.1*j_ + 0.5*k_)*r;
}

/*********************************************
 * TESTS
 * *******************************************/

static void joint_positions_readable_after_set()
{
    DQ_MujocoInterface mi(HEADLESS);
    mi.load_model("test_scene.xml");
    // No simulation step in between: the values must be visible immediately
    mi.set_joint_positions({"hinge", "hinge2"}, vec({0.3, -0.7}));
    VectorXd q = mi.get_joint_positions({"hinge", "hinge2"});
    CHECK_NEAR(q(0), 0.3);
    CHECK_NEAR(q(1), -0.7);
}

static void size_mismatch_throws()
{
    DQ_MujocoInterface mi(HEADLESS);
    mi.load_model("test_scene.xml");
    const std::vector<std::string> joints = {"hinge", "hinge2"};
    CHECK_THROWS(std::invalid_argument, mi.set_joint_positions(joints, vec({0.1})));
    CHECK_THROWS(std::invalid_argument, mi.set_joint_target_positions(joints, vec({0.1})));
    CHECK_THROWS(std::invalid_argument, mi.set_joint_target_velocities(joints, vec({0.1, 0.2, 0.3})));
    CHECK_THROWS(std::invalid_argument, mi.set_joint_target_forces(joints, vec({0.1})));
}

static void site_follows_moving_body()
{
    DQ_MujocoInterface mi(HEADLESS);
    mi.load_model("test_scene.xml");
    // "tip" is 0.3 m along the x axis of the arm, which rotates about z at height 0.5
    mi.set_joint_positions({"hinge"}, vec({M_PI/2}));
    VectorXd p = mi.get_site_pose("tip").translation().vec3();
    CHECK_NEAR(p(0), 0.0);
    CHECK_NEAR(p(1), 0.3);
    CHECK_NEAR(p(2), 0.5);
}

static void set_site_pose_on_worldbody_persists()
{
    DQ_MujocoInterface mi(HEADLESS);
    mi.load_model("test_scene.xml");
    // "world_site" is declared at the body origin, which MuJoCo flags as "same frame as the body"
    const DQ x = test_pose();
    mi.set_site_pose("world_site", x);
    CHECK_NEAR(pose_error(mi.get_site_pose("world_site"), x), 0.0);
    for (int i = 0; i < 50; i++)
        mi.trigger_next_simulation_step();
    CHECK_NEAR(pose_error(mi.get_site_pose("world_site"), x), 0.0);
}

static void set_site_pose_on_moving_body()
{
    DQ_MujocoInterface mi(HEADLESS);
    mi.load_model("test_scene.xml");
    mi.set_joint_positions({"hinge"}, vec({0.4}));
    const DQ x = test_pose();
    mi.set_site_pose("tip", x);
    CHECK_NEAR(pose_error(mi.get_site_pose("tip"), x), 0.0);

    // The pose is stored relative to the arm, so rotating the arm moves the site with it
    mi.set_joint_positions({"hinge"}, vec({0.4 + M_PI}));
    VectorXd p0 = x.translation().vec3();
    VectorXd p1 = mi.get_site_pose("tip").translation().vec3();
    // a rotation of pi about the z axis through (0, 0, 0.5) maps (x, y, z) to (-x, -y, z)
    CHECK_NEAR(p1(0), -p0(0));
    CHECK_NEAR(p1(1), -p0(1));
    CHECK_NEAR(p1(2), p0(2));
}

static void joint_force_with_free_joint()
{
    DQ_MujocoInterface mi(HEADLESS);
    mi.load_model("test_scene.xml");
    // "hinge" has joint id 1 but DoF address 6, because the free joint before it has 6 DoFs
    mi.set_joint_target_forces({"hinge"}, vec({3.0}));
    mi.trigger_next_simulation_step();
    CHECK_NEAR(mi.get_joint_forces({"hinge"})(0), 3.0);
}

static void actuator_lookup_ignores_tendon_actuators()
{
    DQ_MujocoInterface mi(HEADLESS);
    mi.load_model("test_scene.xml");
    // The first actuator drives tendon 1, and "hinge" is joint 1. The command must reach
    // "hinge_motor", not "tendon_motor" (which would push on hinge2 through the tendon).
    mi.set_joint_target_forces({"hinge"}, vec({3.0}));
    mi.trigger_next_simulation_step();
    CHECK_NEAR(mi.get_joint_forces({"hinge"})(0), 3.0);
    CHECK_NEAR(mi.get_joint_forces({"hinge2"})(0), 0.0);
}

static void load_error_includes_mujoco_message()
{
    DQ_MujocoInterface mi(HEADLESS);
    try
    {
        mi.load_model("malformed.xml");
    }
    catch (const std::runtime_error& e)
    {
        // MuJoCo's parser message names the offending element
        std::string message = e.what();
        CHECK(message.find("bogus_element") != std::string::npos);
        return;
    }
    throw std::runtime_error("load_model did not throw for a malformed XML file");
}

static void reset_restores_joints_and_sites()
{
    DQ_MujocoInterface mi(HEADLESS);
    mi.load_model("test_scene.xml");
    const DQ site0 = mi.get_site_pose("world_site");

    mi.set_joint_positions({"hinge", "hinge2"}, vec({0.3, -0.7}));
    mi.set_site_pose("world_site", test_pose());
    for (int i = 0; i < 10; i++)
        mi.trigger_next_simulation_step();

    mi.reset_simulation();
    CHECK_NEAR(mi.get_simulation_time(), 0.0);
    CHECK_NEAR(mi.get_joint_positions({"hinge", "hinge2"}).norm(), 0.0);
    CHECK_NEAR(pose_error(mi.get_site_pose("world_site"), site0), 0.0);
}

static void load_model_twice()
{
    // Windowed, to check that the window and render context are reused for the new model
    DQ_MujocoInterface mi;
    mi.load_model("test_scene.xml");
    mi.get_joint_positions({"hinge"});  // caches the handle of "hinge"

    mi.load_model("pendulum.xml");
    CHECK(mi.get_joint_names() == std::vector<std::string>{"pendulum_joint"});
    // Handles cached for the previous model must not be reused
    CHECK_THROWS(std::runtime_error, mi.get_joint_positions({"hinge"}));
    mi.trigger_next_simulation_step();
    mi.render();
}

static void destroy_without_loading()
{
    // Smoke test: the destructor must not free visualization structures that were never
    // initialized. Reading uninitialized memory is undefined behavior, so a regression may not
    // crash every time; run this test under valgrind to check it thoroughly.
    DQ_MujocoInterface mi;
}

static void headless_mode_needs_no_display()
{
    // CTest runs this test without DISPLAY and WAYLAND_DISPLAY, so opening a window would fail
    DQ_MujocoInterface mi(HEADLESS);
    mi.load_model("test_scene.xml");
    mi.set_joint_target_positions({"hinge2"}, vec({0.5}));
    for (int i = 0; i < 100; i++)
        mi.trigger_next_simulation_step();
    mi.render();  // does nothing
    CHECK(!mi.should_close_window());
    CHECK(mi.get_joint_positions({"hinge2"})(0) > 0.0);  // the servo moved the joint
}

static void two_interfaces_at_once()
{
    DQ_MujocoInterface a;
    a.load_model("test_scene.xml");
    {
        DQ_MujocoInterface b;
        b.load_model("pendulum.xml");
        a.render();
        b.render();
    }
    glfwGetError(nullptr);  // clear any previous error

    // Destroying b must not close a's window or shut down GLFW
    a.trigger_next_simulation_step();
    a.render();
    const char* description = nullptr;
    const int error = glfwGetError(&description);
    if (error != GLFW_NO_ERROR)
        throw std::runtime_error(std::string("GLFW error after destroying the other interface: ")
                                 + (description ? description : std::to_string(error)));
    CHECK(!a.should_close_window());
}

/*********************************************
 * RUNNER
 * *******************************************/

int main(int argc, char** argv)
{
    const std::map<std::string, std::function<void()>> tests_ = {
        {"joint_positions_readable_after_set", joint_positions_readable_after_set},
        {"size_mismatch_throws", size_mismatch_throws},
        {"site_follows_moving_body", site_follows_moving_body},
        {"set_site_pose_on_worldbody_persists", set_site_pose_on_worldbody_persists},
        {"set_site_pose_on_moving_body", set_site_pose_on_moving_body},
        {"joint_force_with_free_joint", joint_force_with_free_joint},
        {"actuator_lookup_ignores_tendon_actuators", actuator_lookup_ignores_tendon_actuators},
        {"load_error_includes_mujoco_message", load_error_includes_mujoco_message},
        {"reset_restores_joints_and_sites", reset_restores_joints_and_sites},
        {"load_model_twice", load_model_twice},
        {"destroy_without_loading", destroy_without_loading},
        {"two_interfaces_at_once", two_interfaces_at_once},
        {"headless_mode_needs_no_display", headless_mode_needs_no_display},
    };

    std::vector<std::string> selected;
    for (int i = 1; i < argc; i++)
        selected.push_back(argv[i]);
    if (selected.empty())
        for (const auto& [name, test] : tests_)
            selected.push_back(name);

    int failures = 0;
    for (const auto& name : selected)
    {
        auto it = tests_.find(name);
        if (it == tests_.end())
        {
            std::cout << "UNKNOWN " << name << std::endl;
            failures++;
            continue;
        }
        try
        {
            it->second();
            std::cout << "PASS " << name << std::endl;
        }
        catch (const std::exception& e)
        {
            std::cout << "FAIL " << name << ": " << e.what() << std::endl;
            failures++;
        }
    }
    return failures == 0 ? 0 : 1;
}
