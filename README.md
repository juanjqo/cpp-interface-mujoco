# cpp-interface-mujoco

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

## Build and run the examples

```bash
mkdir -p ~/git && cd ~/git && git clone https://github.com/juanjqo/cpp-interface-mujoco.git
```

```bash
cd ~/git/cpp-interface-mujoco/examples && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j
```

```bash
cd ~/git/cpp-interface-mujoco/examples/build && ./joint_control
```

```bash
cd ~/git/cpp-interface-mujoco/examples/build && ./site_poses
```

| Example | What it shows |
|---|---|
| [`joint_control`](examples/joint_control.cpp) | Sinusoidal joint references tracked by position actuators; prints joint positions, velocities and torques |
| [`site_poses`](examples/site_poses.cpp) | Moves a free-standing `target` frame with `set_site_pose` and reads the `end_effector` frame with `get_site_pose` |

The first configure downloads and compiles MuJoCo, so it takes a few minutes. The examples use
[`examples/models/manipulator.xml`](examples/models/manipulator.xml), a generic 6-DoF arm built from primitive shapes
(no mesh files), which is copied next to the executables at build time.

## Tests

```bash
cd ~/git/cpp-interface-mujoco/tests && cmake -S . -B build && cmake --build build -j
```

```bash
cd ~/git/cpp-interface-mujoco/tests/build && ctest --output-on-failure
```

The tests use small models in `tests/models` and open a window, so they need a display. On a headless machine
(e.g. CI), run them with `xvfb-run ctest --output-on-failure`. If valgrind is installed, an extra test checks the
destructor for reads of uninitialized memory.

## Usage

```cpp
#include <dqrobotics/interfaces/mujoco/DQ_MujocoInterface.h>

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
| `load_model(xml)` | Load an MJCF file and open the viewer window |
| `trigger_next_simulation_step()` | Advance the simulation by one timestep (`mj_step`) |
| `get_simulation_time()` | Simulation time in seconds |
| `render()` / `should_close_window()` | Draw a frame and process window events / check whether the window was closed |
| `reset_simulation()` | Reset the state to the model's initial configuration |
| `get_joint_names()` | Names of all joints in the model |
| `get_joint_positions/velocities/forces(names)` | Joint state and actuator forces (`qfrc_actuator`) |
| `set_joint_positions(names, q)` | Teleport joints (no dynamics) |
| `set_joint_target_positions/velocities/forces(names, u)` | Actuator commands |
| `get_site_pose(name)` / `set_site_pose(name, x)` | Site pose as a unit dual quaternion |
| `set_wireframe_view`, `set_joint_view`, `set_contact_view`, `set_center_of_mass_view` | Visualization toggles |

## License

LGPL. See [LICENSE](LICENSE). The robot models keep their original licenses (see the `LICENSE` file in each model folder).
