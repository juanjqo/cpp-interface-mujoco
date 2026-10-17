# cpp-interface-mujoco

[![CI](https://github.com/juanjqo/cpp-interface-mujoco/actions/workflows/ci.yml/badge.svg)](https://github.com/juanjqo/cpp-interface-mujoco/actions/workflows/ci.yml)

An unofficial [DQ Robotics](https://dqrobotics.github.io) interface for C++17 to interact with [MuJoCo](https://mujoco.org).

It wraps a MuJoCo model and viewer behind an API that mirrors `DQ_CoppeliaSimInterface`, so controllers written with
DQ Robotics can drive a MuJoCo simulation using joint names and dual-quaternion poses.

## Features

- Load an MJCF (XML) model from a file or from memory, and draw it in a GLFW window (mouse orbit/pan/zoom, FPS,
  simulation-time and real-time-factor overlay). A viewer thread draws the window: no render calls are needed
- Real-time simulation: `start_simulation()` runs the physics in its own thread, in real time, like a real robot. The
  program reads the state and sends commands at its own rate, and never waits for a simulation step. Or step the
  simulation yourself with `trigger_next_simulation_step()`
- Joint space: read positions, velocities and actuator forces; teleport joints; command position, velocity or
  torque actuators
- Task space: get and set the pose of any site, and get the pose of any body, as a unit dual quaternion
- Floating-base robots (humanoids, quadrupeds): get and set the pose and twist of the base, reset to a keyframe, and
  make the camera follow the robot
- Reference frames: draw named frames at runtime from dual quaternion poses, without editing the XML model
- Visualization toggles: wireframe, joint axes, contact forces, center of mass, site frames
- Simulation reset, to the initial configuration or to a keyframe of the model
- Headless mode (no window) for automated tests, CI and batch experiments
- Robots library: robot models embedded in the library, loaded by including a header (no model files needed)
- Scene builder: build a scene in code (floor, lights, boxes, spheres, cylinders, capsules, sites and robots, posed
  with dual quaternions), and edit it while it runs: add or remove objects and robots without resetting the simulation
- CMake package: `find_package(dqrobotics-interface-mujoco)` and the targets `dqrobotics_extensions::interface-mujoco`
  and `dqrobotics_extensions::interface-mujoco-robots`

## Install

> [!NOTE]
> Non-sudo privileges? Create a custom prefix folder (e.g. `~/opt`) to hold `lib/` and `include/` without needing root.
> See [this guide](https://ros2-tutorial.readthedocs.io/en/latest/cmake/cmake_packages_without_sudo.html) for
> background.

### Prerequisites

| Dependency | How it is obtained |
|---|---|
| [MuJoCo](https://github.com/google-deepmind/mujoco) 3.9.0 | Downloaded and built automatically by CMake (`FetchContent`), and installed together with this library |
| [sas_cpp](https://github.com/MarinhoLab/sas_cpp) (`marinholab_sas_core`) | Its `ThreadManager` runs the physics and viewer threads. The installed package is used if found (`libmarinholab-sas-core` from the SmartArmStack apt repository); otherwise it is downloaded and built automatically, and installed together with this library |
| [DQ Robotics C++](https://github.com/dqrobotics/cpp) | Install manually (see below) |
| Eigen3, GLFW, OpenGL | System packages |

Ubuntu:

```shell
sudo apt install build-essential cmake git libeigen3-dev libglfw3-dev libgl1-mesa-dev
sudo add-apt-repository ppa:dqrobotics-dev/development
sudo apt update && sudo apt install libdqrobotics
```

macOS:

```shell
brew install cmake eigen glfw
```

Then build and install [DQ Robotics C++](https://github.com/dqrobotics/cpp) from source.

If you're installing any of the above without sudo, install them to the same custom prefix `~/opt`, and see the
non-sudo instructions below.

### Clone the repository

```shell
mkdir -p ~/git && cd ~/git
git clone https://github.com/juanjqo/cpp-interface-mujoco.git
cd ~/git/cpp-interface-mujoco
```

### Sudo users

```shell
# 1. Configure: choose Release, and (optionally) where to install it.
#    Omit -DCMAKE_INSTALL_PREFIX to use the system default (/usr/local on Linux).
#    To build only the library, add -DDQ_MUJOCO_BUILD_EXAMPLES=OFF -DDQ_MUJOCO_BUILD_TESTS=OFF
#    To skip the robots library (embedded robot models), add -DDQ_MUJOCO_BUILD_ROBOTS=OFF
cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local

# 2. Build the library, examples and tests.
cmake --build build -j$(nproc)

# 3. (Optional) Run the tests.
ctest --test-dir build --output-on-failure

# 4. Install the headers, the library, MuJoCo, and the exported CMake package, then refresh the linker cache.
sudo cmake --install build
sudo ldconfig
```

### Non-sudo users

```shell
# 1. Configure: choose Release, and install to your own prefix instead of a system path.
#    To build only the library, add -DDQ_MUJOCO_BUILD_EXAMPLES=OFF -DDQ_MUJOCO_BUILD_TESTS=OFF
#    To skip the robots library (embedded robot models), add -DDQ_MUJOCO_BUILD_ROBOTS=OFF
cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=$HOME/opt

# 2. Build the library, examples and tests.
cmake --build build -j$(nproc)

# 3. (Optional) Run the tests.
ctest --test-dir build --output-on-failure

# 4. Install the headers, the library, MuJoCo, and the exported CMake package. No sudo needed.
cmake --install build
```

> [!TIP]
> If you skipped exporting `CMAKE_PREFIX_PATH` (along with `LD_LIBRARY_PATH`, `LIBRARY_PATH`, and `CPATH`) in
> `~/.bashrc` (see [this guide](https://ros2-tutorial.readthedocs.io/en/latest/cmake/cmake_packages_without_sudo.html)),
> any project that later does `find_package(dqrobotics-interface-mujoco)` needs to be told where to look, since
> `$HOME/opt` isn't a default search path:
>
> ```shell
> cmake -S . -B build -DCMAKE_PREFIX_PATH=$HOME/opt
> ```

The first configure downloads and compiles MuJoCo, so it takes a few minutes. After updating the library (e.g.,
`git pull`), build and install it again.

## Examples

```shell
cd ~/git/cpp-interface-mujoco/build/examples
./joint_control
./site_poses
./reference_frames
./floating_base
./twist_open_loop
./scene_builder
```

| Example | What it shows |
|---|---|
| [`joint_control`](examples/joint_control.cpp) | Real-time simulation with a 500 Hz control loop: sinusoidal joint references tracked by position actuators; prints joint positions, velocities, torques and the real-time factor |
| [`site_poses`](examples/site_poses.cpp) | Moves a free-standing `target` frame with `set_site_pose` and reads the `end_effector` frame with `get_site_pose` |
| [`reference_frames`](examples/reference_frames.cpp) | Draws desired, measured and fixed reference frames, plus a trail of past poses created and removed on the fly |
| [`floating_base`](examples/floating_base.cpp) | A quadruped starts from its `home` keyframe, rocks its trunk with the leg actuators and gets pushed with `set_base_twist`; the camera follows it |
| [`scene_builder`](examples/scene_builder.cpp) | Builds a scene in code (a table, two quadrupeds, an arm and a kinematic block moved with `set_body_pose`) and edits it while it runs in real time: a ball is dropped every 2 s and the oldest is removed; the arm's kinematics, moved to where it was placed, are drawn on its end effector |
| [`unitree_b1`](examples/unitree_b1.cpp) | The Unitree B1 quadruped stands with its feet on the floor and moves its base (up and down, rolling, pitching): each leg follows its foot, relative to the desired base, with a kinematic controller that uses `UnitreeB1Robot::kinematics()` |
| [`unitree_z1`](examples/unitree_z1.cpp) | Two Unitree Z1 arms, with and without the gripper, placed with the scene builder: each end effector follows a moving target with a kinematic controller that uses `UnitreeZ1Robot::kinematics()`, and the gripper opens and closes |
| [`twist_open_loop`](examples/twist_open_loop.cpp) | Stepped simulation paced by a sas `Clock`: commands a floating box with constant twists (no feedback) in `WORLD_FRAME` and `BODY_FRAME`, before every step: rise, orbit the world z axis, spin in place, drive in a circle, descend |

The first three examples use `GenericManipulatorRobot`, a generic 6-DoF arm, `floating_base` uses
`GenericQuadrupedRobot`, `scene_builder` uses both, and `unitree_b1` and `unitree_z1` use `UnitreeB1Robot` and
`UnitreeZ1Robot`, all from the
[robots library](#robots-library), so they need no model files.
`twist_open_loop` uses [`examples/models/floating_box.xml`](examples/models/floating_box.xml), a box with gravity
compensation, which is copied next to the executables at build time.

## Tests

```shell
cd ~/git/cpp-interface-mujoco
ctest --test-dir build --output-on-failure
```

Most tests run in headless mode. The ones that check the window need a display; on a headless machine, run
`xvfb-run -a -s "-screen 0 1280x1024x24" ctest --test-dir build --output-on-failure`, as the
[CI workflow](.github/workflows/ci.yml) does. If valgrind is installed, an extra test checks the destructor for reads
of uninitialized memory.

## Usage

```cmake
find_package(dqrobotics-interface-mujoco REQUIRED)
target_link_libraries(${YOUR_LIBRARY} PRIVATE dqrobotics_extensions::interface-mujoco)
# for the robot models (GenericManipulatorRobot below); it also links the interface
target_link_libraries(${YOUR_LIBRARY} PRIVATE dqrobotics_extensions::interface-mujoco-robots)
```

```cpp
#include <dqrobotics_extensions/interfaces/mujoco/DQ_MujocoInterface.h>
#include <dqrobotics_extensions/interfaces/mujoco/robots/GenericManipulatorRobot.h>

using namespace DQ_robotics;             // DQ, i_, j_, k_, E_, ...
using namespace DQ_robotics_extensions;  // DQ_MujocoInterface
using namespace Eigen;

int main()
{
    const DQ_MujocoRobotDescription robot = GenericManipulatorRobot::description();

    DQ_MujocoInterface mi;
    mi.load_model(robot.model);  // or, for your own model file: mi.load_model("my_robot.xml")

    const std::vector<std::string>& joints = robot.joint_names;
    mi.reset_simulation(robot.home_keyframe);  // initial configuration from a keyframe
    const VectorXd q_home = mi.get_joint_positions(joints);

    mi.start_simulation();  // the physics runs in real time in its own thread; the window draws itself

    marinholab::sas::core::Clock clock(0.002);  // your control loop: 500 Hz
    clock.init();
    while (!mi.should_close_window())  // also processes the window events
    {
        VectorXd q = mi.get_joint_positions(joints);
        VectorXd q_ref = q_home + VectorXd::Constant(joints.size(), 0.2*sin(mi.get_simulation_time()));
        mi.set_joint_target_positions(joints, q_ref);  // drives the position actuators

        DQ x = mi.get_site_pose(robot.end_effector_sites.at(0));  // site pose as a unit dual quaternion
        clock.update_and_sleep();
    }
    mi.stop_simulation();
}
```

(`#include <marinholab/sas/core/sas_clock.hpp>` and link `marinholab::sas::core` for the `Clock`; any other way of
pacing the loop works too.)

### Real time and stepped simulation

There are two ways to run the simulation. In both, a viewer thread draws the window at 60 Hz, so there is nothing to
render; just call `should_close_window()` regularly from the main thread (e.g., as the loop condition), since GLFW
only processes the window events (mouse, resize, close) there.

| | Real time: `start_simulation()` ... `stop_simulation()` | Stepped: `trigger_next_simulation_step()` |
|---|---|---|
| Who advances the time | A physics thread, which keeps the simulation time equal to the wall-clock time | Your program, one timestep per call |
| Your loop | Runs at its own rate (e.g., 500 Hz with a sas `Clock`), like the control loop of a real robot | Must pace itself if it should run in real time; otherwise the simulation runs as fast as possible |
| Getters (`get_*`) | Read the state published after the last step: they never wait for a step (about 1 µs) | Read the current state |
| Commands (`set_joint_target_*`) | Applied before the next step | Applied at the next `trigger_next_simulation_step()` |
| Teleports (`set_joint_positions`, `set_base_pose`, `reset_simulation`, ...) | Take effect at once (they wait for the step in progress) | Take effect at once |
| Use it for | Demos, teleoperation, controllers that will run on a real robot | Deterministic experiments, CI, per-step control |

If the computer cannot keep up, the real-time simulation runs as fast as it can; `get_real_time_factor()` and the
window overlay show the simulated time per wall-clock time. `trigger_next_simulation_step()` throws while the
simulation runs in real time, and `load_model()` stops it.

In stepped mode, nothing slows the loop down: without pacing, the simulation runs as fast as the computer allows. To
step it in real time, pace the loop (e.g., with a sas `Clock`, as in [`twist_open_loop`](examples/twist_open_loop.cpp)),
or use `start_simulation()`.

### Window and headless mode

```cpp
DQ_MujocoInterface windowed;  // opens a 1280x920 window titled with the model name
DQ_MujocoInterface custom(DQ_MujocoInterface::RENDERING::WINDOW, "My simulation", 1920, 1080);
DQ_MujocoInterface headless(DQ_MujocoInterface::RENDERING::HEADLESS);  // no window, no display needed
```

In headless mode, no window is opened and `should_close_window()` always returns `false`, so the same simulation loop
works in both modes. Real-time and stepped simulation work in both.

### Joint commands and actuators

The `set_joint_target_*` methods write to the MuJoCo actuator (`d->ctrl`) attached to each joint, so their meaning
depends on the actuator defined in the XML:

| Method | Expected actuator |
|---|---|
| `set_joint_target_positions` | `<position>` (as in the example manipulator), or `<general>` with an affine bias |
| `set_joint_target_velocities` | `<velocity>` |
| `set_joint_target_forces` | `<motor>` |

`set_joint_positions` is different: it teleports the joints by writing `qpos` directly, bypassing the dynamics. Use it
for initialization and resets, not for control.

### Sites

`get_site_pose` and `set_site_pose` work with [sites](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-site),
which are frames with no physics. They are useful as end-effector frames or as visual targets. `set_site_pose` stores
the pose relative to the site's parent body: a site in the `worldbody` stays fixed in the world, and a site on a robot
link moves with that link. The generic manipulator of the robots library
([`generic_manipulator.xml`](robots/models/generic_manipulator/generic_manipulator.xml)) has both kinds:

```xml
<worldbody>
  <!-- a free-standing visual target -->
  <site name="target" type="sphere" size="0.03" rgba="1 0 0 0.5" pos="0.4 0 0.4"/>
  ...
      <body name="link6" pos="0 0 0.10">
        <!-- a frame attached to the last link -->
        <site name="end_effector" pos="0 0 0.06" size="0.01"/>
```

Site frames are drawn in the viewer by default; hide them with `set_site_frame_view(false)`.

### Floating-base robots

Humanoids, quadrupeds and free objects have a floating base: a body with a [free joint](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-freejoint).
A free joint has 7 positions and 6 velocities, so it is not handled by the joint methods (they throw if you pass it).
Instead, address the base by the name of its body, which also works for unnamed free joints (`<freejoint/>`, as in most
[MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) robots):

```cpp
mi.load_model(GenericQuadrupedRobot::description().model);  // see the robots library below
mi.reset_simulation("home");        // most legged robots need a standing keyframe to start
mi.set_camera_tracking("trunk");    // keep the robot in view

std::vector<std::string> bases = mi.get_floating_base_names();  // {"trunk"}
std::vector<std::string> joints = mi.get_joint_names(true);     // only the hinge/slide joints of the limbs

DQ x_base = mi.get_base_pose("trunk");     // pose of the base in the world
DQ xi_base = mi.get_base_twist("trunk");   // twist of the base in the world frame
VectorXd q = mi.get_joint_positions(joints);
DQ x_foot = mi.get_body_pose("FL_calf");   // any body, e.g., a foot or a hand
```

`x_base` and `q` are the full configuration of the robot: for instance, use `x_base` as the reference frame of the
`DQ_SerialManipulator` of each leg.

`set_base_pose` and `set_base_twist` write the state directly, like `set_joint_positions`. For a robot that moves
through its own actuators and contacts (a legged robot), use them only for initialization, resets or disturbances.
A body without dynamics of its own (e.g., a free-flying object with gravity compensation) can be commanded with a
twist in every step, as in the [`twist_open_loop`](examples/twist_open_loop.cpp) example.

#### Reference frame of the twist

`get_base_twist` and `set_base_twist` take an optional `DQ_MujocoInterface::REFERENCE_FRAME` that sets the frame of
the twist. 

```cpp
using RF = DQ_MujocoInterface::REFERENCE_FRAME;
const DQ x = mi.get_base_pose("box");
const DQ p = x.translation();

// velocities -> twist
mi.set_base_twist("box", w + E_*(p_dot + cross(p, w)));                  // w, p_dot in the world frame
mi.set_base_twist("box", w_body + E_*p_dot_body, RF::BODY_FRAME);        // w_body, p_dot_body in the body frame

// twist -> velocities
const DQ xi = mi.get_base_twist("box");                                  // same as RF::WORLD_FRAME
const DQ w_measured = xi.P();
const DQ p_dot_measured = xi.D() - cross(p, w_measured);

const DQ xi_body = mi.get_base_twist("box", RF::BODY_FRAME);             // equals Ad(conj(x), xi)
const DQ w_body_measured = xi_body.P();
const DQ p_dot_body_measured = xi_body.D();
```

MuJoCo stores the velocity of a free joint (`qvel`) in a third, mixed form: the linear velocity of the body origin in
the world frame, followed by the angular velocity in the body frame. The interface converts it, so you never need to
handle `qvel` directly.

### Robots library

The robots library holds robot models embedded in the library: you load a robot by including its header, with no
model files to ship or find at runtime. Each robot provides a `DQ_MujocoRobotDescription`, with the model and the
names needed to use it, so that they are not hardcoded:

```cmake
target_link_libraries(${YOUR_LIBRARY} PRIVATE dqrobotics_extensions::interface-mujoco-robots)
```

```cpp
#include <dqrobotics_extensions/interfaces/mujoco/robots/GenericQuadrupedRobot.h>

const DQ_MujocoRobotDescription robot = GenericQuadrupedRobot::description();
mi.load_model(robot.model);                                     // from memory
mi.reset_simulation(robot.home_keyframe);                       // "home"
VectorXd q = mi.get_joint_positions(robot.joint_names);         // FL_hip, FL_knee, ...
DQ x_base = mi.get_base_pose(robot.floating_base);              // "trunk"
DQ x_foot = mi.get_site_pose(robot.end_effector_sites.at(0));   // "FL_foot"
```

| Robot | Header | Description |
|---|---|---|
| `GenericManipulatorRobot` | [`GenericManipulatorRobot.h`](robots/include/dqrobotics_extensions/interfaces/mujoco/robots/GenericManipulatorRobot.h) | 6-DoF arm built from primitive shapes, with position actuators; fixed base, end effector `end_effector`, keyframe `home`. Also has a free-standing `target` site in the world. `kinematics()` returns its `DQ_SerialManipulatorDH` |
| `GenericQuadrupedRobot` | [`GenericQuadrupedRobot.h`](robots/include/dqrobotics_extensions/interfaces/mujoco/robots/GenericQuadrupedRobot.h) | Quadruped with 2 joints per leg, built from primitive shapes; floating base `trunk`, feet `FL_foot`, `FR_foot`, `RL_foot`, `RR_foot`, keyframe `home`. `kinematics()` returns one `DQ_SerialManipulatorDH` per leg, in the trunk frame |
| `UnitreeB1Robot` | [`UnitreeB1Robot.h`](robots/include/dqrobotics_extensions/interfaces/mujoco/robots/UnitreeB1Robot.h) | The Unitree B1, a quadruped with 3 joints per leg (with its meshes), with position actuators, velocity actuators or torque motors: `description(UnitreeB1Robot::ACTUATORS::POSITION)` (default), `ACTUATORS::VELOCITY` or `ACTUATORS::TORQUE`, all within the torque limits of the joints. Floating base `base`, feet `FL_foot`, `FR_foot`, `RL_foot`, `RR_foot`, keyframe `home` (standing). `kinematics()` returns one `DQ_SerialManipulatorDH` per leg, in the base frame |
| `UnitreeZ1Robot` | [`UnitreeZ1Robot.h`](robots/include/dqrobotics_extensions/interfaces/mujoco/robots/UnitreeZ1Robot.h) | The Unitree Z1, a 6-DoF arm, from MuJoCo Menagerie (with its meshes), with or without its gripper: `description(UnitreeZ1Robot::GRIPPER::WITH)` (default) or `GRIPPER::WITHOUT`. Fixed base, end effector `end_effector` (tip of the gripper, or flange), keyframe `home`; the gripper joint is in `gripper_joint_names`. `kinematics(gripper)` returns its `DQ_SerialManipulatorDH` (the DH model of [Adorno-Lab/unitree_kinematics](https://github.com/Adorno-Lab/unitree_kinematics)) |

Robots can also provide their DQ Robotics kinematic model, which matches the MuJoCo model (a test checks that
`fkm(q)` equals the pose of the end-effector site), so a controller and the simulation share the same geometry:

```cpp
DQ_SerialManipulatorDH arm = GenericManipulatorRobot::kinematics();
VectorXd q = mi.get_joint_positions(robot.joint_names);  // same joint order
DQ x = arm.fkm(q);                                       // = mi.get_site_pose("end_effector")
MatrixXd J = arm.pose_jacobian(q);
```

For a floating-base robot, the limbs are expressed in the frame of the floating base, so the world pose is the base
pose times the limb pose. `GenericQuadrupedRobot::kinematics()` returns the legs in the order of
`end_effector_sites` (FL, FR, RL, RR); leg `i` has the joints `joint_names[2i]` (hip) and `joint_names[2i+1]` (knee):

```cpp
std::vector<DQ_SerialManipulatorDH> legs = GenericQuadrupedRobot::kinematics();
const DQ x_trunk = mi.get_base_pose(robot.floating_base);
const VectorXd q_FL = mi.get_joint_positions({robot.joint_names.at(0), robot.joint_names.at(1)});

DQ x_foot = x_trunk*legs.at(0).fkm(q_FL);                       // = mi.get_site_pose("FL_foot")
```

The models of the generic robots live in [`robots/models`](robots/models), and the others (with their meshes, licenses
and sources) in [cpp-interface-mujoco-models](https://github.com/juanjqo/cpp-interface-mujoco-models), which the build
downloads at a pinned commit. To use a local checkout of it instead (e.g., to work offline or to change a model),
configure with `-DFETCHCONTENT_SOURCE_DIR_CPP_INTERFACE_MUJOCO_MODELS=<path>`. At build time, `dq_mujoco_embed_model()`
([`cmake/DQMujocoEmbedModel.cmake`](cmake/DQMujocoEmbedModel.cmake)) turns each model directory (the XML and its
meshes, textures and `<include>` files) into a C++ function that returns a `DQ_MujocoModel`. Steps to add a robot are
in [`robots/CMakeLists.txt`](robots/CMakeLists.txt). Configure with `-DDQ_MUJOCO_BUILD_ROBOTS=OFF` to skip the library.

### Scene builder

`DQ_MujocoSceneBuilder` builds a scene in code. Everything is posed with unit dual quaternions, and `build()` returns
a `DQ_MujocoModel` for `load_model()` (its `xml` can also be saved to a file):

```cpp
#include <dqrobotics_extensions/interfaces/mujoco/DQ_MujocoSceneBuilder.h>
using BODY = DQ_MujocoSceneBuilder::BODY;

DQ_MujocoSceneBuilder scene("pick and place");
scene.set_timestep(0.002);                                     // physics options belong to the scene
scene.add_floor();
scene.add_light("sun", 3*k_, -1.0*k_, true);                   // position, direction, directional
scene.add_box("table", x_table, {0.25, 0.25, 0.2});            // FIXED: welded to the world
scene.add_sphere("ball", x_ball, 0.04, BODY::FREE, {1, 0, 0, 1}, 0.1);  // FREE: a floating base
scene.add_box("pusher", x_pusher, {0.05, 0.05, 0.05}, BODY::KINEMATIC);  // moved by the program
scene.add_site("target", x_target);                            // movable with set_site_pose()

auto arm = scene.add_robot(GenericManipulatorRobot::description(), "arm_", x_arm);
auto dog = scene.add_robot(GenericQuadrupedRobot::description(), "dog_", x_dog);
// arm.joint_names = {"arm_joint1", ...}, dog.floating_base = "dog_trunk", ...

mi.load_model(scene.build());
mi.reset_simulation("home");   // every robot at its home configuration, where it was placed
```

- **Robots** are added with a prefix (not empty) for all their names, so a robot can be added several times. `add_robot()`
  returns the description of the placed robot (prefixed names). Only the robot itself is added: the floor, lights and
  sky of its model are not, nor its unused materials and textures, and the physics options are those of the scene.
- **The "home" keyframe** of the scene has each robot at its own home keyframe, moved to where it was placed (MuJoCo
  does not move the keyframes of attached floating-base robots), and every other element at its initial pose.
- **How objects move:**

  | `BODY` | Moved by | Use |
  |---|---|---|
  | `FIXED` (default) | Nothing: welded to the world | Tables, walls |
  | `KINEMATIC` | Your program, with `mi.set_body_pose(name, x)` (about 2 µs, also while running) | Moving obstacles, platforms, targets that collide |
  | `FREE` | The physics: it falls, collides and can be pushed | Objects to push, grasp or throw (floating bases: `get_base_pose`, `set_base_twist`, ...) |

  Kinematic objects are MuJoCo mocap bodies: gravity and contacts do not move them, but they push `FREE` objects and
  robots. MuJoCo treats them as having zero velocity in the contacts, so an object resting on a moving kinematic
  platform slides instead of being carried by friction.
- **Kinematics** of a placed robot: move its reference frame to where it was placed,
  `kinematics.set_reference_frame(x_arm*kinematics.get_reference_frame())`.
- **Robots with meshes and textures** (e.g., `UnitreeB1Robot`, `UnitreeZ1Robot`) must have their files in the
  assets of their model. The scene keeps them under `robot_assets/`, with the hash of their content in the name, so a
  file shared by several robots is stored once. `build()` returns them in its `assets`: to save the scene, write the
  `xml` and each asset at its path, relative to the XML file.

#### Editing a scene while it runs

`edit_scene()` edits the loaded scene with the same methods, also while the simulation runs in real time. The state is
kept: the scene changes without a reset, and new objects and robots start at their home configuration.

```cpp
mi.start_simulation();
...
mi.edit_scene([&](DQ_MujocoSceneBuilder& scene) {
    scene.add_sphere("ball_2", x, 0.04, BODY::FREE);
    scene.remove("ball_1");                                     // objects, sites, lights, the floor
    scene.add_robot(GenericManipulatorRobot::description(), "arm2_", x_arm2);
});
mi.edit_scene([](DQ_MujocoSceneBuilder& scene) { scene.remove("arm2_base"); });  // a robot, by its root body
```

- It works with any loaded model (from a file, from memory, or built), and keeps its "home" keyframe up to date.
- Edits are checked before they are applied: if one makes the scene invalid (e.g., a name already in use), or the
  function throws, the scene is left unchanged and the exception is rethrown.
- An edit recompiles the whole scene: about 10 ms with the scenes made of primitive shapes, but about 260 ms to add
  a Unitree Z1 to a running scene, and about 190 ms to add a ball to a scene with three of them, since their meshes are
  large. A real-time simulation keeps running: it waits while the model is recompiled (most of that time) and then
  catches up. Site poses set with `set_site_pose()`, the camera and the reference frames are kept.
- Call it from the main thread.

> **Recommendation:** build the scene before starting the control loop, and do not edit it while the loop runs.
> Editing on the go is possible, but each edit stalls the simulation while the model is recompiled (longer with
> meshes), and the controller sees a pause and a jump. Use edits for setup steps, and between experiments. To move
> objects during the loop, use `KINEMATIC` bodies with `set_body_pose()`, which take microseconds.

### Models in memory

`load_model` also accepts a `DQ_MujocoModel`: the XML text plus the files it refers to, keyed by their path relative
to the XML as MuJoCo resolves it (including `<compiler meshdir>`). Nothing is read from disk, so models can also be
generated or modified at runtime:

```cpp
DQ_MujocoModel model;
model.name = "robot.xml";
model.xml = R"(<mujoco>
                 <compiler meshdir="meshes"/>
                 <asset><mesh file="link.obj"/></asset>
                 ...
               </mujoco>)";
model.assets["meshes/link.obj"] = link_obj_bytes;  // std::vector<unsigned char>
mi.load_model(model);
```

### Reference frames

To plot a frame (a desired pose, a target, an intermediate frame of your kinematics, ...) you don't need a site in
the XML: create it at runtime. Reference frames are only drawn, so they don't affect the simulation.

```cpp
//                     name         axis length  axis width  opacity  label
mi.add_reference_frame("x_desired", 0.2,         0.009,      0.5);   // created at the world origin
mi.add_reference_frame("x");                                         // defaults: 0.1, 0.005, 1.0, true

while (!mi.should_close_window())
{
    mi.set_reference_frame_pose("x_desired", x_desired);             // any unit dual quaternion
    mi.set_reference_frame_pose("x", mi.get_site_pose("end_effector"));
    ...
}

mi.remove_reference_frame("x_desired");
```

The axes are drawn as arrows (x red, y green, z blue) with the name as a label at the origin. Use the opacity to tell
frames apart, e.g., a semi-transparent desired pose and an opaque measured pose. In headless mode the frames are stored
but not drawn.

## API

| Method | Description |
|---|---|
| `DQ_MujocoInterface(rendering, title, width, height)` | All arguments are optional (default: a 1280x920 window titled with the model name) |
| `load_model(xml)` | Load an MJCF file and open the viewer window (unless headless). Can be called again to switch models |
| `load_model(model)` | Same, from a `DQ_MujocoModel` held in memory (e.g., `GenericQuadrupedRobot::description().model` or `DQ_MujocoSceneBuilder::build()`) |
| `edit_scene(edit)` | Edit the loaded scene with a `DQ_MujocoSceneBuilder` (add/remove objects, lights, sites, robots), also while it runs; the state is kept |
| `start_simulation()` / `stop_simulation()` / `is_simulation_running()` | Run the simulation in real time in a physics thread / stop it (the state is kept) / whether it runs |
| `get_real_time_factor()` | Simulated time per wall-clock time over the last 0.5 s (0 when not running) |
| `trigger_next_simulation_step()` | Advance the simulation by one timestep (`mj_step`), when it is not running in real time |
| `get_simulation_time()` | Simulation time in seconds |
| `should_close_window()` | Process the window events and check whether the window was closed (`false` when headless) |
| `reset_simulation()` / `reset_simulation(keyframe)` | Reset the state to the model's initial configuration or to a keyframe |
| `get_joint_names(only_single_dof)` | Names of all joints in the model, or only of the named hinge and slide joints |
| `get_joint_positions/velocities/forces(names)` | Joint state and actuator forces (`qfrc_actuator`) |
| `set_joint_positions(names, q)` | Teleport joints (no dynamics) |
| `set_joint_target_positions/velocities/forces(names, u)` | Actuator commands |
| `get_site_pose(name)` / `set_site_pose(name, x)` | Site pose as a unit dual quaternion |
| `get_body_pose(name)` / `set_body_pose(name, x)` | Body pose as a unit dual quaternion / move a `KINEMATIC` body (a MuJoCo mocap body) |
| `get_floating_base_names()` | Bodies with a free joint |
| `get_base_pose(body)` / `set_base_pose(body, x)` | Floating-base pose as a unit dual quaternion (set: teleport) |
| `get_base_twist(body, frame)` / `set_base_twist(body, xi, frame)` | Floating-base twist in `WORLD_FRAME` (default, reference point at the world origin) or `BODY_FRAME` (reference point at the body origin) (set: teleport) |
| `set_camera_tracking(body)` | Make the camera follow a body (empty name: free camera) |
| `add_reference_frame(name, length, width, opacity, label)` / `remove_reference_frame(name)` | Create or remove a frame that is only drawn (no XML needed) |
| `set_reference_frame_pose(name, x)` / `get_reference_frame_pose(name)` | Pose of a reference frame as a unit dual quaternion |
| `set_wireframe_view`, `set_joint_view`, `set_contact_view`, `set_center_of_mass_view`, `set_site_frame_view` | Visualization toggles |

`DQ_MujocoSceneBuilder`:

| Method | Description |
|---|---|
| `DQ_MujocoSceneBuilder(name)` | An empty scene with a sky and a headlight (timestep 0.002 s, `IMPLICITFAST` integrator) |
| `set_timestep(dt)` / `set_integrator(integrator)` / `set_gravity(g)` | Physics options of the scene |
| `add_floor(name)` | An infinite checkered floor at z = 0 |
| `add_light(name, position, direction, directional)` | A spot or directional light |
| `add_box/sphere/cylinder/capsule(name, x, sizes..., body, rgba, mass)` | A primitive object, `FIXED`, `KINEMATIC` or `FREE` |
| `add_site(name, x, size, rgba)` | A site in the world (a frame without physics) |
| `add_robot(robot, prefix, x)` | A robot (e.g., of the robots library), placed at `x`; returns its description in the scene |
| `remove(name)` | Remove an object, site, light, the floor, or a robot (by its prefixed root body) |
| `build()` | The scene as a `DQ_MujocoModel`, with a "home" keyframe |

## License

Copyright (C) 2025-2026 Juan Jose Quiroz Omana. Licensed under the [LGPL-3.0-or-later](LICENSE).

This project uses [DQ Robotics](https://github.com/dqrobotics/cpp) (LGPL-3.0-or-later) but is not part of the DQ
Robotics project. The robots library embeds the models of the Unitree B1 and Z1 of
[cpp-interface-mujoco-models](https://github.com/juanjqo/cpp-interface-mujoco-models), BSD-3-Clause, Copyright (c)
2016-2022 Unitree Robotics (the Z1 comes from [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie));
their licenses are installed with the library.
It also uses [MuJoCo](https://github.com/google-deepmind/mujoco) (Apache-2.0), which is downloaded at
build time and installed with the library; the mouse and scroll handling of the viewer is adapted from MuJoCo's
`sample/basic.cc`. Its threads are run by [sas_cpp](https://github.com/MarinhoLab/sas_cpp) (LGPL-3.0), which prints
its license notice when a thread is created.
