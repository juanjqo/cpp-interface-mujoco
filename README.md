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
- Reference frames: draw named frames at runtime from dual quaternion poses, without editing the XML model
- Visualization toggles: wireframe, joint axes, contact forces, center of mass, site frames
- Simulation reset
- Headless mode (no window) for automated tests, CI and batch experiments
- CMake package: `find_package(dqrobotics-interface-mujoco)` and the target `dqrobotics_extensions::interface-mujoco`

## Install

> [!NOTE]
> Non-sudo privileges? Create a custom prefix folder (e.g. `~/opt`) to hold `lib/` and `include/` without needing root.
> See [this guide](https://ros2-tutorial.readthedocs.io/en/latest/cmake/cmake_packages_without_sudo.html) for
> background.

### Prerequisites

| Dependency | How it is obtained |
|---|---|
| [MuJoCo](https://github.com/google-deepmind/mujoco) 3.9.0 | Downloaded and built automatically by CMake (`FetchContent`), and installed together with this library |
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
```

| Example | What it shows |
|---|---|
| [`joint_control`](examples/joint_control.cpp) | Sinusoidal joint references tracked by position actuators; prints joint positions, velocities and torques |
| [`site_poses`](examples/site_poses.cpp) | Moves a free-standing `target` frame with `set_site_pose` and reads the `end_effector` frame with `get_site_pose` |
| [`reference_frames`](examples/reference_frames.cpp) | Draws desired, measured and fixed reference frames, plus a trail of past poses created and removed on the fly |

The examples use [`examples/models/manipulator.xml`](examples/models/manipulator.xml), a generic 6-DoF arm built from
primitive shapes (no mesh files), which is copied next to the executables at build time.

## Tests

```shell
cd ~/git/cpp-interface-mujoco
ctest --test-dir build --output-on-failure
```

Most tests run in headless mode. The two that check the window need a display; on a headless machine, run
`xvfb-run -a -s "-screen 0 1280x1024x24" ctest --test-dir build --output-on-failure`, as the
[CI workflow](.github/workflows/ci.yml) does. If valgrind is installed, an extra test checks the destructor for reads
of uninitialized memory.

## Usage

```cmake
find_package(dqrobotics-interface-mujoco REQUIRED)
target_link_libraries(${YOUR_LIBRARY} PRIVATE dqrobotics_extensions::interface-mujoco)
```

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

Site frames are drawn in the viewer by default; hide them with `set_site_frame_view(false)`.

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
    mi.render();
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
| `trigger_next_simulation_step()` | Advance the simulation by one timestep (`mj_step`) |
| `get_simulation_time()` | Simulation time in seconds |
| `render()` / `should_close_window()` | Draw a frame and process window events / check whether the window was closed (no-op / `false` when headless) |
| `reset_simulation()` | Reset the state to the model's initial configuration |
| `get_joint_names()` | Names of all joints in the model |
| `get_joint_positions/velocities/forces(names)` | Joint state and actuator forces (`qfrc_actuator`) |
| `set_joint_positions(names, q)` | Teleport joints (no dynamics) |
| `set_joint_target_positions/velocities/forces(names, u)` | Actuator commands |
| `get_site_pose(name)` / `set_site_pose(name, x)` | Site pose as a unit dual quaternion |
| `add_reference_frame(name, length, width, opacity, label)` / `remove_reference_frame(name)` | Create or remove a frame that is only drawn (no XML needed) |
| `set_reference_frame_pose(name, x)` / `get_reference_frame_pose(name)` | Pose of a reference frame as a unit dual quaternion |
| `set_wireframe_view`, `set_joint_view`, `set_contact_view`, `set_center_of_mass_view`, `set_site_frame_view` | Visualization toggles |

## License

[LGPL-3.0-or-later](LICENSE), like [DQ Robotics](https://github.com/dqrobotics/cpp).
