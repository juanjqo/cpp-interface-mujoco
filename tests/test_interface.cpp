/**
Tests for DQ_MujocoInterface.

Each test is a function registered in tests_. Run all of them with
    ./test_interface
or a single one (this is how CTest calls them) with
    ./test_interface <test_name>

The tests must run from the directory that contains the test models (CTest does this).
Most tests run in HEADLESS mode. The ones that check the window (load_model_twice,
two_interfaces_at_once, reference_frames_more_than_scene_capacity, camera_tracking) need a
display; on a headless machine, use xvfb-run.
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

static void reference_frames_api()
{
    // Works without a model, and in HEADLESS mode (frames are stored, not drawn)
    DQ_MujocoInterface mi(HEADLESS);
    mi.add_reference_frame("x_desired", 0.2, 0.01, 0.4, false);
    CHECK_NEAR(pose_error(mi.get_reference_frame_pose("x_desired"), DQ(1)), 0.0);

    const DQ x = test_pose();
    mi.set_reference_frame_pose("x_desired", x);
    CHECK_NEAR(pose_error(mi.get_reference_frame_pose("x_desired"), x), 0.0);

    // Reference frames are independent of the model
    mi.load_model("test_scene.xml");
    for (int i = 0; i < 10; i++)
        mi.trigger_next_simulation_step();
    mi.reset_simulation();
    CHECK_NEAR(pose_error(mi.get_reference_frame_pose("x_desired"), x), 0.0);

    // Invalid calls
    CHECK_THROWS(std::invalid_argument, mi.add_reference_frame("x_desired"));  // already exists
    CHECK_THROWS(std::invalid_argument, mi.add_reference_frame(""));
    CHECK_THROWS(std::invalid_argument, mi.add_reference_frame("a", 0.0));
    CHECK_THROWS(std::invalid_argument, mi.add_reference_frame("b", 0.1, -0.01));
    CHECK_THROWS(std::invalid_argument, mi.add_reference_frame("c", 0.1, 0.01, 1.5));
    CHECK_THROWS(std::runtime_error, mi.set_reference_frame_pose("unknown", x));
    CHECK_THROWS(std::runtime_error, mi.set_reference_frame_pose("x_desired", 2.0*x));  // not unit

    mi.remove_reference_frame("x_desired");
    CHECK_THROWS(std::runtime_error, mi.get_reference_frame_pose("x_desired"));
    CHECK_THROWS(std::runtime_error, mi.remove_reference_frame("x_desired"));
    mi.add_reference_frame("x_desired");  // the name can be reused after removing it
}

static void reference_frames_more_than_scene_capacity()
{
    // 1000 labeled frames need 4000 geoms, more than the scene holds: the extra frames
    // must be skipped without crashing or corrupting the rendering
    DQ_MujocoInterface mi;
    mi.load_model("test_scene.xml");
    for (int i = 0; i < 1000; i++)
    {
        const std::string name = "frame_" + std::to_string(i);
        mi.add_reference_frame(name);
        mi.set_reference_frame_pose(name, 1 + 0.5*E_*(0.001*i*i_));
    }
    glfwGetError(nullptr);  // clear any previous error
    for (int i = 0; i < 3; i++)
    {
        mi.trigger_next_simulation_step();
        mi.render();
    }
    CHECK(glfwGetError(nullptr) == GLFW_NO_ERROR);
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

static void base_pose_set_and_get()
{
    DQ_MujocoInterface mi(HEADLESS);
    mi.load_model("floating_base.xml");
    const DQ x = test_pose();
    mi.set_base_pose("base", x);
    CHECK_NEAR(pose_error(mi.get_base_pose("base"), x), 0.0);
    // The body frame is updated immediately
    CHECK_NEAR(pose_error(mi.get_body_pose("base"), x), 0.0);
    // ...and so are the frames attached to the base: the foot is 0.4 m below the base origin
    const DQ x_foot = x*(1 + 0.5*E_*(-0.4*k_));
    CHECK_NEAR(pose_error(mi.get_site_pose("foot").translation(), x_foot.translation()), 0.0);

    // Invalid calls
    CHECK_THROWS(std::runtime_error, mi.set_base_pose("base", 2.0*x));  // not unit
    CHECK_THROWS(std::runtime_error, mi.get_base_pose("leg"));          // no free joint
    CHECK_THROWS(std::runtime_error, mi.get_base_pose("unknown"));
    CHECK_THROWS(std::runtime_error, mi.get_body_pose("unknown"));
}

static void base_twist_matches_motion()
{
    DQ_MujocoInterface mi(HEADLESS);
    mi.load_model("floating_base.xml");
    // A rotated base away from the origin, so that mixing up the frames of the free joint
    // velocity (linear: world, angular: body) or forgetting cross(p, w) changes the result
    const DQ x0 = test_pose();
    mi.set_base_pose("base", x0);
    const DQ w = 0.3*i_ - 0.5*j_ + 0.8*k_;
    const DQ p_dot = 0.2*i_ + 0.1*j_ - 0.4*k_;
    const DQ xi = w + E_*(p_dot + cross(x0.translation(), w));
    mi.set_base_twist("base", xi);
    CHECK_NEAR(vec8(mi.get_base_twist("base") - xi).norm(), 0.0);

    // One step of the free motion must agree with x_dot = 0.5*xi*x (no gravity, small timestep)
    const double t0 = mi.get_simulation_time();
    mi.trigger_next_simulation_step();
    const double dt = mi.get_simulation_time() - t0;
    const DQ x_dot = (1.0/dt)*(mi.get_base_pose("base") - x0);
    CHECK(vec8(x_dot - 0.5*xi*x0).norm() < 1e-3);

    CHECK_THROWS(std::runtime_error, mi.set_base_twist("base", 1 + xi));  // not pure
}

static void base_twist_reference_frames()
{
    // Checks the frame conventions stated in the documentation of get/set_base_twist() against
    // one step of the free motion (no gravity, small timestep)
    DQ_MujocoInterface mi(HEADLESS);
    mi.load_model("floating_base.xml");
    const DQ x0 = test_pose();
    const DQ r0 = x0.P();
    const DQ p0 = x0.translation();
    const DQ w = 0.3*i_ - 0.5*j_ + 0.8*k_;

    // xi = w + E_*cross(p, w) spins the body in place: its origin does not move
    mi.set_base_pose("base", x0);
    mi.set_base_twist("base", w + E_*cross(p0, w));
    double t0 = mi.get_simulation_time();
    mi.trigger_next_simulation_step();
    double dt = mi.get_simulation_time() - t0;
    CHECK((mi.get_base_pose("base").translation() - p0).vec3().norm()/dt < 1e-3);

    // xi = w alone rotates the body about the world origin: its origin moves with velocity cross(w, p)
    mi.set_base_pose("base", x0);
    mi.set_base_twist("base", w);
    t0 = mi.get_simulation_time();
    mi.trigger_next_simulation_step();
    dt = mi.get_simulation_time() - t0;
    const DQ p_dot = (1.0/dt)*(mi.get_base_pose("base").translation() - p0);
    CHECK(vec4(p_dot - cross(w, p0)).norm() < 1e-3);

    // BODY_FRAME: the twist is Ad(conj(x), xi), made of the angular velocity and the velocity of
    // the body origin, both expressed in the body frame
    using RF = DQ_MujocoInterface::REFERENCE_FRAME;
    const DQ v = 0.2*i_ + 0.1*j_ - 0.4*k_;
    const DQ xi = w + E_*(v + cross(p0, w));
    mi.set_base_pose("base", x0);
    mi.set_base_twist("base", xi);
    const DQ xi_body = mi.get_base_twist("base", RF::BODY_FRAME);
    CHECK_NEAR(vec8(xi_body - Ad(conj(x0), xi)).norm(), 0.0);
    t0 = mi.get_simulation_time();
    mi.trigger_next_simulation_step();
    dt = mi.get_simulation_time() - t0;
    const DQ x1 = mi.get_base_pose("base");
    const DQ w_body = (2.0/dt)*(conj(r0)*x1.P() - 1);   // r1 = r0*(1 + 0.5*w_body*dt)
    const DQ v_body = (1.0/dt)*(conj(r0)*(x1.translation() - p0)*r0);
    CHECK(vec4(w_body - xi_body.P()).norm() < 1e-3);
    CHECK(vec4(v_body - xi_body.D()).norm() < 1e-3);

    // Setting in BODY_FRAME is the inverse, and gives the same motion as the world-frame twist
    mi.set_base_pose("base", x0);
    mi.set_base_twist("base", xi_body, RF::BODY_FRAME);
    CHECK_NEAR(vec8(mi.get_base_twist("base", RF::BODY_FRAME) - xi_body).norm(), 0.0);
    CHECK_NEAR(vec8(mi.get_base_twist("base", RF::WORLD_FRAME) - xi).norm(), 0.0);

    // In BODY_FRAME, an angular velocity alone spins the body in place
    mi.set_base_pose("base", x0);
    mi.set_base_twist("base", w, RF::BODY_FRAME);
    t0 = mi.get_simulation_time();
    mi.trigger_next_simulation_step();
    dt = mi.get_simulation_time() - t0;
    CHECK((mi.get_base_pose("base").translation() - p0).vec3().norm()/dt < 1e-3);

    CHECK_THROWS(std::runtime_error, mi.set_base_twist("base", 1 + xi, RF::BODY_FRAME));  // not pure
}

static void base_falls_under_gravity()
{
    DQ_MujocoInterface mi(HEADLESS);
    mi.load_model("test_scene.xml");
    // "box" starts at (1, 1, 1) at rest, with nothing below it
    const int steps = 100;
    for (int i = 0; i < steps; i++)
        mi.trigger_next_simulation_step();
    const double t = mi.get_simulation_time();
    // No rotation, so the twist is just the linear velocity -g*t (exact for MuJoCo's Euler integrator)
    CHECK_NEAR(vec8(mi.get_base_twist("box") - E_*(-9.81*t*k_)).norm(), 0.0);
    VectorXd p = mi.get_base_pose("box").translation().vec3();
    CHECK_NEAR(p(0), 1.0);
    CHECK_NEAR(p(1), 1.0);
    CHECK(std::abs(p(2) - (1.0 - 0.5*9.81*t*t)) < 1e-2);
}

static void joint_methods_reject_free_and_ball_joints()
{
    DQ_MujocoInterface mi(HEADLESS);
    mi.load_model("test_scene.xml");
    CHECK_THROWS(std::invalid_argument, mi.get_joint_positions({"box_free"}));
    CHECK_THROWS(std::invalid_argument, mi.set_joint_positions({"box_free"}, vec({0.1})));
    CHECK_THROWS(std::invalid_argument, mi.get_joint_velocities({"box_free"}));
    CHECK_THROWS(std::invalid_argument, mi.get_joint_forces({"box_free"}));
    try
    {
        mi.get_joint_positions({"box_free"});
    }
    catch (const std::invalid_argument& e)
    {
        // The message points to the base methods and the body to use
        CHECK(std::string(e.what()).find("get_base_pose") != std::string::npos);
        CHECK(std::string(e.what()).find("\"box\"") != std::string::npos);
    }

    mi.load_model("floating_base.xml");
    CHECK_THROWS(std::invalid_argument, mi.get_joint_positions({"tail_ball"}));
    CHECK_THROWS(std::invalid_argument, mi.set_joint_positions({"tail_ball"}, vec({0.1})));
}

static void joint_and_floating_base_names()
{
    DQ_MujocoInterface mi(HEADLESS);
    mi.load_model("test_scene.xml");
    CHECK((mi.get_joint_names() == std::vector<std::string>{"box_free", "hinge", "hinge2"}));
    CHECK((mi.get_joint_names(true) == std::vector<std::string>{"hinge", "hinge2"}));
    CHECK(mi.get_floating_base_names() == std::vector<std::string>{"box"});

    // The free joint is unnamed and "tail_ball" has 3 DoFs
    mi.load_model("floating_base.xml");
    CHECK(mi.get_joint_names(true) == std::vector<std::string>{"hip"});
    CHECK(mi.get_floating_base_names() == std::vector<std::string>{"base"});

    mi.load_model("pendulum.xml");
    CHECK(mi.get_floating_base_names().empty());
}

static void reset_to_keyframe()
{
    DQ_MujocoInterface mi(HEADLESS);
    mi.load_model("floating_base.xml");
    for (int i = 0; i < 10; i++)
        mi.trigger_next_simulation_step();

    mi.reset_simulation("home");
    CHECK_NEAR(mi.get_simulation_time(), 0.0);
    const DQ r = cos(M_PI/4) + k_*sin(M_PI/4);
    const DQ x_home = r + 0.5*E_*(0.5*i_ - 0.2*j_ + 2.0*k_)*r;
    CHECK_NEAR(pose_error(mi.get_base_pose("base"), x_home), 0.0);
    CHECK_NEAR(pose_error(mi.get_body_pose("base"), x_home), 0.0);
    CHECK_NEAR(mi.get_joint_positions({"hip"})(0), 0.3);

    // Without a keyframe, the reset goes back to the configuration in the XML
    mi.reset_simulation();
    CHECK_NEAR(pose_error(mi.get_base_pose("base"), 1 + 0.5*E_*k_), 0.0);
    CHECK_NEAR(mi.get_joint_positions({"hip"})(0), 0.0);

    CHECK_THROWS(std::runtime_error, mi.reset_simulation("unknown"));
}

static void camera_tracking()
{
    // Windowed, so that the tracking camera is actually used by the renderer
    DQ_MujocoInterface mi;
    mi.load_model("floating_base.xml");
    mi.set_camera_tracking("base");
    mi.set_base_pose("base", 1 + 0.5*E_*(5.0*i_ + 3.0*j_ + 2.0*k_));
    glfwGetError(nullptr);  // clear any previous error
    for (int i = 0; i < 3; i++)
    {
        mi.trigger_next_simulation_step();
        mi.render();
    }
    mi.set_camera_tracking("");  // back to a free camera
    mi.render();
    CHECK(glfwGetError(nullptr) == GLFW_NO_ERROR);
    CHECK_THROWS(std::runtime_error, mi.set_camera_tracking("unknown"));

    // The tracked body id must not leak into another model
    mi.set_camera_tracking("tail");
    mi.load_model("pendulum.xml");
    mi.render();
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
        {"reference_frames_api", reference_frames_api},
        {"reference_frames_more_than_scene_capacity", reference_frames_more_than_scene_capacity},
        {"base_pose_set_and_get", base_pose_set_and_get},
        {"base_twist_matches_motion", base_twist_matches_motion},
        {"base_twist_reference_frames", base_twist_reference_frames},
        {"base_falls_under_gravity", base_falls_under_gravity},
        {"joint_methods_reject_free_and_ball_joints", joint_methods_reject_free_and_ball_joints},
        {"joint_and_floating_base_names", joint_and_floating_base_names},
        {"reset_to_keyframe", reset_to_keyframe},
        {"camera_tracking", camera_tracking},
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
