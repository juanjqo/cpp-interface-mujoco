# cpp-interface-mujoco

[![CI](https://github.com/juanjqo/cpp-interface-mujoco/actions/workflows/ci.yml/badge.svg)](https://github.com/juanjqo/cpp-interface-mujoco/actions/workflows/ci.yml)

An unofficial [DQ Robotics](https://dqrobotics.github.io) interface for C++17 to interact with [MuJoCo](https://mujoco.org).

It wraps a MuJoCo model and viewer behind an API that mirrors `DQ_CoppeliaSimInterface`, so controllers written with
DQ Robotics can drive a MuJoCo simulation using joint names and dual-quaternion poses.

## Features

- Load an MJCF (XML) model and render it in a GLFW window (mouse orbit/pan/zoom, FPS and simulation-time overlay)
- Joint space: read positions, velocities and actuator forces; teleport joints; command position, velocity or
  torque actuators
- Task space: get and set the pose of any site as a unit dual quaternion
- Visualization toggles: wireframe, joint axes, contact forces, center of mass
- Simulation reset
- Headless mode (no window) for automated tests, CI and batch experiments
- CMake package: `find_package(dqrobotics-interface-mujoco)` and the target `dqrobotics_extensions::interface-mujoco`

## Dependencies

| Dependency | How it is obtained |
|---|---|
| [MuJoCo](https://github.com/google-deepmind/mujoco) 3.9.0 | Downloaded and built automatically by CMake (`FetchContent`) |
| [DQ Robotics C++](https://github.com/dqrobotics/cpp) | Install manually (see below) |
| Eigen3, GLFW, OpenGL | System packages |

### Ubuntu

```bash
sudo apt install build-essential cmake git libeigen3-dev libglfw3-dev libgl1-mesa-dev
```

```bash
sudo add-apt-repository ppa:dqrobotics-dev/development && sudo apt update && sudo apt install libdqrobotics
```

### macOS

```bash
brew install cmake eigen glfw
```

Then build and install [DQ Robotics C++](https://github.com/dqrobotics/cpp) from source.

## Build

```bash
mkdir -p ~/git && cd ~/git && git clone https://github.com/juanjqo/cpp-interface-mujoco.git
```

```bash
cd ~/git/cpp-interface-mujoco && cmake -S . -B build && cmake --build build -j
```

This builds the library, the examples and the tests (in Release mode, unless `CMAKE_BUILD_TYPE` is given). The first
configure downloads and compiles MuJoCo, so it takes a few minutes. To build only the library, add
`-DDQ_MUJOCO_BUILD_EXAMPLES=OFF -DDQ_MUJOCO_BUILD_TESTS=OFF`.

### Examples

```bash
cd ~/git/cpp-interface-mujoco/build/examples && ./joint_control
```

```bash
cd ~/git/cpp-interface-mujoco/build/examples && ./site_poses
```

| Example | What it shows |
|---|---|
| [`joint_control`](examples/joint_control.cpp) | Sinusoidal joint references tracked by position actuators; prints joint positions, velocities and torques |
| [`site_poses`](examples/site_poses.cpp) | Moves a free-standing `target` frame with `set_site_pose` and reads the `end_effector` frame with `get_site_pose` |

The examples use [`examples/models/manipulator.xml`](examples/models/manipulator.xml), a generic 6-DoF arm built from
primitive shapes (no mesh files), which is copied next to the executables at build time.

### Tests

```bash
cd ~/git/cpp-interface-mujoco && ctest --test-dir build --output-on-failure
```

Most tests run in headless mode. The two that check the window need a display; on a headless machine, run
`xvfb-run -a -s "-screen 0 1280x1024x24" ctest --test-dir build --output-on-failure`, as the
[CI workflow](.github/workflows/ci.yml) does. If valgrind is installed, an extra test checks the destructor for reads
of uninitialized memory.

### Install and use from another project

After [building](#build), install the library once:

```bash
cd ~/git/cpp-interface-mujoco && sudo cmake --install build && sudo ldconfig
```

This installs the library, its header, and MuJoCo 3.9.0 (which it depends on) to `/usr/local`. To install elsewhere,
configure with `-DCMAKE_INSTALL_PREFIX=<dir>` (and pass `-DCMAKE_PREFIX_PATH=<dir>` when configuring your project).
Reinstall after updating the library. Then, in your project's `CMakeLists.txt`:

```cmake
find_package(dqrobotics-interface-mujoco REQUIRED)
target_link_libraries(my_app PRIVATE dqrobotics_extensions::interface-mujoco)
```

## Usage

```cpp
#include <dqrobotics_extensions/interfaces/mujoco/DQ_MujocoInterface.h>

using namespace DQ_robotics;             // DQ, i_, j_, k_, E_, ...
using namespace DQ_robotics_extensions;  // DQ_MujocoInterface
using namespace Eigen;

int main()
{
    DQ_MujocoInterface mi;
    mi.load_model("manipulator.xml");

    std::vector<std::string> joints = mi.get_joint_names();
    VectorXd q_home(6);
    q_home << 0.0, 0.5, 1.2, 0.0, 1.0, 0.0;
    mi.set_joint_positions(joints, q_home);  // initial configuration (teleport)

    while (!mi.should_close_window())
    {
        // Advance the physics in real time at 60 Hz rendering
        double t0 = mi.get_simulation_time();
        while (mi.get_simulation_time() - t0 < 1.0/60.0)
        {
            VectorXd q = mi.get_joint_positions(joints);
            VectorXd q_ref = q_home + VectorXd::Constant(joints.size(), 0.2*sin(mi.get_simulation_time()));
            mi.set_joint_target_positions(joints, q_ref);  // drives the position actuators
            mi.trigger_next_simulation_step();
        }

        DQ x = mi.get_site_pose("end_effector");  // pose of a site as a unit dual quaternion
        mi.render();
    }
}
```

### Window and headless mode

```cpp
DQ_MujocoInterface windowed;  // opens a 1280x920 window titled with the model name
DQ_MujocoInterface custom(DQ_MujocoInterface::RENDERING::WINDOW, "My simulation", 1920, 1080);
DQ_MujocoInterface headless(DQ_MujocoInterface::RENDERING::HEADLESS);  // no window, no display needed
```

In headless mode, `render()` does nothing and `should_close_window()` always returns `false`, so the same simulation
loop works in both modes.

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
link moves with that link. The example manipulator has both kinds:

```xml
<worldbody>
  <!-- a free-standing visual target -->
  <site name="target" type="sphere" size="0.03" rgba="1 0 0 0.5" pos="0.4 0 0.4"/>
  ...
      <body name="link6" pos="0 0 0.10">
        <!-- a frame attached to the last link -->
        <site name="end_effector" pos="0 0 0.06" size="0.01"/>
```

Site frames are drawn in the viewer by default.

## API

| Method | Description |
|---|---|
| `DQ_MujocoInterface(rendering, title, width, height)` | All arguments are optional (default: a 1280x920 window titled with the model name) |
| `load_model(xml)` | Load an MJCF file and open the viewer window (unless headless). Can be called again to switch models |
| `trigger_next_simulation_step()` | Advance the simulation by one timestep (`mj_step`) |
| `get_simulation_time()` | Simulation time in seconds |
| `render()` / `should_close_window()` | Draw a frame and process window events / check whether the window was closed (no-op / `false` when headless) |
| `reset_simulation()` | Reset the state to the model's initial configuration |
| `get_joint_names()` | Names of all joints in the model |
| `get_joint_positions/velocities/forces(names)` | Joint state and actuator forces (`qfrc_actuator`) |
| `set_joint_positions(names, q)` | Teleport joints (no dynamics) |
| `set_joint_target_positions/velocities/forces(names, u)` | Actuator commands |
| `get_site_pose(name)` / `set_site_pose(name, x)` | Site pose as a unit dual quaternion |
| `set_wireframe_view`, `set_joint_view`, `set_contact_view`, `set_center_of_mass_view` | Visualization toggles |

## License

[LGPL-3.0-or-later](LICENSE), like [DQ Robotics](https://github.com/dqrobotics/cpp).
