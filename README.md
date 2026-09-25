# Minitrone MuJoCo · ROS 2 Workspace

Developed by the **Mobile Robotics Laboratory (MRL), Seoul National University of Science and Technology (SeoulTech)**.

**Laboratory website: [mrl.seoultech.ac.kr](https://mrl.seoultech.ac.kr/index.do)**

This ROS 2 workspace supports flight and wall contact experiments with Minitrone, a drone equipped with four motors and four tilt servos. It includes MuJoCo physics simulation, position and attitude control, actuator allocation, external wrench observers, center of pressure (CoP) estimation, admittance control, and passive alignment control.

## 1. Environment and MuJoCo Installation

ROS 2 is assumed to be installed already. The commands below target **Ubuntu 22.04 / ROS 2 Humble / system Python 3.10**. MuJoCo is pinned to **3.3.7**, the version installed in the current development environment.

### Install dependencies

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y \
  build-essential python3-pip python3-colcon-common-extensions \
  python3-numpy python3-pyqt5 python3-pyqtgraph \
  libglfw3 libgl1-mesa-dri libeigen3-dev \
  ros-humble-eigen3-cmake-module ros-humble-rosidl-default-generators \
  ros-humble-rosidl-runtime-py ros-humble-rosbag2

/usr/bin/python3 -m pip install --user "mujoco==3.3.7" "numpy<2"
```

The official MuJoCo Python package includes the engine library, so no separate engine download is required. See the [MuJoCo installation documentation](https://mujoco.readthedocs.io/en/3.3.7/python.html#installation). Deactivate Conda or other Python environments before installation and building so that the workspace uses the same system Python as ROS 2.

```bash
/usr/bin/python3 -c "import mujoco, rclpy, pyqtgraph; print('MuJoCo:', mujoco.__version__)"
```

The MuJoCo viewer and plotting windows require a graphical desktop; the viewer also requires OpenGL support. For simulation without a viewer, see the `minitrone_plant` instructions below.

## 2. Build and Terminal Setup

These examples assume the repository is located at `~/ros2_project/minitrone_ws`. Adjust the path if needed.

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_project/minitrone_ws
colcon build --executor sequential
source install/setup.bash
```

Run the following in **each terminal used to launch nodes**:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_project/minitrone_ws
source install/setup.bash
```

Rebuild after editing Python code or XML models to update the installed files:

```bash
colcon build --packages-select minitrone_plant --executor sequential
source install/setup.bash
```

## 3. Package Overview

| Package | Language / Build System | Purpose |
| --- | --- | --- |
| `minitrone_plant` | Python / ament_python | MuJoCo model loading, physics, state and contact data publication, wall teleoperation, and plotting |
| `minitrone_controller` | C++ / ament_cmake | Position and attitude control, motor and servo allocation, wrench observers, EKF, and contact control |
| `minitrone_cmd` | C++ and Python / ament_cmake | Position, attitude, and external wrench commands, keyboard teleoperation, and launch configuration |
| `minitrone_interfaces` | ROS messages / ament_cmake | Custom messages shared by the packages |

```text
minitrone_ws/
├── src/
│   ├── minitrone_plant/
│   │   ├── minitrone_plant/     # Simulator and GUI nodes
│   │   └── xml/                # Drone, wall, and scene models
│   ├── minitrone_controller/src/
│   ├── minitrone_cmd/
│   │   ├── src/
│   │   ├── scripts/
│   │   └── launch/
│   └── minitrone_interfaces/msg/
├── matlabscripts/              # Visualization of experiment CSV data
├── bags/                       # rosbag recordings
├── build/                      # Build artifacts
├── install/                    # Installed packages used by ROS 2
└── log/                        # Build logs
```

### Models and control flow

The plant loads [`src/minitrone_plant/xml/Scene.xml`](src/minitrone_plant/xml/Scene.xml). The filename is case-sensitive.

- `minitrone_430x430x370_armature_walls.xml`: drone body, tilt servos, thrust actuators, and contact plate on the +X side.
- `palm.xml`: the `hand_palm` body, which acts as a wall with adjustable position and orientation.
- The current `Scene.xml` includes `Follower.xml`, `palm.xml`, and `contact_box.xml`. `Leader.xml` is a separate identical drone model; neither file creates a second vehicle automatically.

The original single-wall drone body mass is 2.5 kg; the currently included `Follower.xml` has a combined body mass of 2.86 kg, including two 180 g plates. The plant sets the physics timestep to **0.0025 s (400 Hz)**. Its input contains four motor speed commands and four servo angle commands. Motor thrust is computed as `0.02 × omega²`. The allocator limits each motor to 15 N and servo commands to ±65°.

```text
Position/attitude commands -> wrench_controller -> allocator_controller -> plant
                                    ^                      ^                |
                                    +----- /minitrone/state +----------------+

plant -> /minitrone/mob_observer_input -> External wrench observer
                                                    |
                                           Wrench / CoP estimates
                                                    |
                                           Optional contact control
```

The default controllers use `/minitrone/state`. Starting the EKF does not automatically switch controller feedback to the EKF output.

## 4. Quick Start

### Wall contact experiments: `arm_launch.py`

Run in terminal 1:

```bash
ros2 launch minitrone_cmd arm_launch.py
```

This launches the following processes:

| Process | Function |
| --- | --- |
| `minitrone_plant` | MuJoCo simulation and viewer |
| `minitrone_wrench_controller` | Converts position and attitude references into force and moment commands |
| `minitrone_allocator_controller` | Converts wrench commands into motor and servo commands |
| `minitrone_second_wrench_observer` | Second-order external wrench observation and CoP estimation |
| `ros2 bag record -a` | Records all topics |

**Position, attitude, and contact control command nodes are not started automatically.** Add a position command node in terminal 2:

```bash
ros2 run minitrone_cmd minitrone_position_teleop
```

Optionally start wall teleoperation and plotting in separate terminals:

```bash
ros2 run minitrone_plant minitrone_palm_teleop
```

```bash
ros2 run minitrone_plant minitrone_external_wrench_plot
```

The launch file saves recordings to **`~/ros2_project/minitrone_ws/bags/bag_all_YYYYMMDD_HHMMSS`**. Moving the repository does not change this path automatically. Update `workspace_dir` in `arm_launch.py` if needed.

### Running nodes

- Avoid starting a second instance of a node already started by the launch file.
- Activate only one position command source: `minitrone_position_cmd` or `minitrone_position_teleop`. Both publish to `/minitrone/cmd`.
- Run keyboard nodes with `ros2 run` in separate terminals and keep the relevant terminal focused. The viewer's `F` shortcut is handled in the viewer window.
- Stop processes with `Ctrl+C` in their terminals. Closing only the viewer window can leave ROS nodes and physics running.

## 5. Nodes in `minitrone_plant`

### `minitrone_plant` — Physics simulator

Applies actuator commands from `/minitrone/input` and publishes drone state, actual actuator forces and moments, contact forces, and CoP. It also handles external wrench and wall pose commands. Running this node alone does not start a flight controller.

```bash
ros2 run minitrone_plant minitrone_plant
```

Run without the viewer:

```bash
ros2 run minitrone_plant minitrone_plant --ros-args -p enable_viewer:=false
```

| Parameter | Default | Function |
| --- | --- | --- |
| `enable_viewer` | `true` | Opens the MuJoCo viewer |
| `viewer_show_propellers` | `true` | Displays propeller visualization groups |
| `viewer_show_contact_forces` | `true` | Displays the resultant contact force arrow; toggle with `F` in the viewer |
| `viewer_contact_force_scale` | `0.03` | Contact force arrow length scale |
| `random_seed` | `1` | Sensor noise random seed |
| `contact_test_case` | `B` | Selects the contact configuration |
| `isolate_plate_wall_contact` | `true` | Isolates plate–wall contact from other drone–wall collisions |
| `solver_iterations` | `0` | Overrides the solver iteration count when positive |

Contact configurations are A: frictionless `condim=1`; B: `condim=3` with sliding friction; C: `condim=6` with very small torsional and rolling friction; D: keeps the XML contact dimension and friction settings. The default is B.

```bash
ros2 run minitrone_plant minitrone_plant --ros-args \
  -p contact_test_case:=B -p solver_iterations:=100
```

The current launch file does not expose launch arguments such as `enable_viewer`. For headless operation, start the plant using the command above and launch controllers separately, or edit the launch file's `Node(parameters=...)` configuration.

### `minitrone_palm_teleop` — Wall pose teleoperation

Publishes wall position and orientation commands to `/minitrone/palm_pose_cmd`. Default translation and rotation speeds are 0.05 m/s and 1°/s.

```bash
ros2 run minitrone_plant minitrone_palm_teleop --ros-args \
  -p speed_xyz:=0.05 -p speed_rpy_deg:=1.0
```

| Keys | Function |
| --- | --- |
| `W/S`, `A/D`, `Q/E` | Increase/decrease x, y, and z, respectively |
| `R/F`, `T/G`, `Y/H` | Increase/decrease yaw, pitch, and roll, respectively |
| `Space` / `0` / `X` | Stop motion / restore the initial command / quit |

### `minitrone_external_wrench_plot` — External wrench and CoP plots

A Qt/PyQtGraph GUI for second-order wrench observer output and estimated/contact-derived CoP. The observer and plant must be running to supply these data.

```bash
ros2 run minitrone_plant minitrone_external_wrench_plot --ros-args \
  -p window_sec:=15.0 -p refresh_hz:=20.0 \
  -p plot_rows:=3 -p plot_columns:=3
```

`plot_rows` and `plot_columns` accept values from 1 to 3. The default displayed plate dimensions are `plate_size_y=0.40` and `plate_size_z=0.38` m. Additional settings include `cop_force_min=0.5` N, `cop_trail_length=100`, `cop_y_sign=1.0`, and `cop_z_sign=-1.0`.

### `minitrone_topic_plot` — General topic plotter

```bash
ros2 run minitrone_plant minitrone_topic_plot
```

Select numeric fields from the topic list and add them to a plot. Variable-length arrays expand into individual element fields after messages arrive.

## 6. Nodes in `minitrone_cmd`

### `minitrone_position_cmd` — Target position commands

Publishes position references to `/minitrone/cmd` at 200 Hz. It is disabled by default; toggle it with `O` or the `enabled` parameter. By default, x motion starts after 5 seconds and y motion after 10 seconds, with axis-specific speed limits. The initial z reference is set directly to `Z_CMD`.

```bash
ros2 run minitrone_cmd minitrone_position_cmd --ros-args \
  -p enabled:=true -p X_CMD:=0.0 -p Y_CMD:=0.0 -p Z_CMD:=1.0
```

Change targets while running:

```bash
ros2 param set /minitrone_position_cmd X_CMD 0.5
ros2 param set /minitrone_position_cmd Z_CMD 1.2
```

Default speed limits are `max_speed_x=0.10`, `max_speed_y=0.10`, and `max_speed_z=0.30` m/s. Configure motion start times with `approach_start_sec` and `y_start_sec`.

### `minitrone_position_teleop` — Keyboard position commands

Selects motion velocities through keyboard input and integrates them into position commands. The default initial command is `[0, 0, 1] m`.

```bash
ros2 run minitrone_cmd minitrone_position_teleop --ros-args \
  -p speed_xy:=0.3 -p speed_z:=0.2
```

| Keys | Function |
| --- | --- |
| `W/S`, `A/D`, `R/F` | Increase/decrease x, y, and z, respectively |
| Arrow keys / `[` / `]` | Move in x/y / ascend / descend |
| `Space` / `0` / `Q` | Stop motion / restore the initial command / quit |

### `minitrone_attitude_sweep_cmd` — Keyboard attitude commands

Adjusts attitude references through keyboard input. Despite its name, it does not run an automatic periodic sweep. It publishes `/minitrone/att_cmd` at 50 Hz by default, with 0.5° increments and a default limit of ±60° per axis.

```bash
ros2 run minitrone_cmd minitrone_attitude_sweep_cmd --ros-args -p max_abs_deg:=30.0
```

| Keys | Function |
| --- | --- |
| `q/w`, `e/r`, `t/y` | Increase/decrease roll, pitch, and yaw, respectively |
| `z` / `x` | Reset attitude commands to zero / quit |

### `minitrone_external_wrench_cmd` — External force and moment commands

Publishes disturbances in the drone body frame to `/minitrone/external_wrench_cmd`. Default increments are 0.5 N for force and 0.1 N·m for moment. The plant clears the disturbance if no command arrives for more than 0.2 seconds.

```bash
ros2 run minitrone_cmd minitrone_external_wrench_cmd --ros-args \
  -p force_step:=0.5 -p moment_step:=0.1 \
  -p moment_disturbance_amplitude:=0.3 \
  -p moment_disturbance_frequency_hz:=0.5
```

| Keys | Function |
| --- | --- |
| `q/a`, `w/s`, `e/d` | Increase/decrease Mx, My, and Mz, respectively |
| `i/k`, `j/l` | Increase/decrease Fx and Fy, respectively |
| `u` | Increase Fz |
| `o`, `p`, `y` | Toggle sinusoidal Mx, My, and Mz disturbances, respectively |
| `z` / `x` | Reset all disturbances / quit |

The current implementation assigns `o` to the sinusoidal Mx disturbance. There is no key assigned to decreasing Fz.

## 7. Nodes in `minitrone_controller`

### `minitrone_wrench_controller` — Position and attitude control

Computes body-frame force and moment commands on `/minitrone/wrench_cmd` using cascaded position/velocity PID, attitude control, and gravity compensation. Control updates are triggered by fresh state messages. When admittance control is active, it uses the corresponding position and attitude references.

```bash
ros2 run minitrone_controller minitrone_wrench_controller
```

Default parameters are `mass=2.5` kg and `gravity=9.81` m/s².

The translation cascade runs entirely in the world frame:
`position reference → position PID → velocity reference → velocity PID → acceleration reference`.
The published body force is `R_WBᵀ · mass · (acceleration reference + [0, 0, gravity])`.
Attitude control, wrench topics, allocator/passive-alignment interfaces, and admittance
reference selection are unchanged. Both cascade integrators reset on admittance transitions.

Startup parameters are `position_kp_x`, `position_ki_x`, `position_kd_x`,
`velocity_kp_x`, `velocity_ki_x`, `velocity_kd_x` (also `_y` and `_z`).
Defaults split the old force PID into an outer PI and inner P, reproducing its
force law, integral clamps, and ±200 N per-world-axis feedback limit:

| Gain | x / y | z |
| --- | --- | --- |
| Position Kp | 28 / 6 | 24 / 10 |
| Position Ki | 1.5 / 6 | 1.2 / 10 |
| Position Kd | 0 | 0 |
| Velocity Kp | 6 / mass | 10 / mass |
| Velocity Ki / Kd | 0 / 0 | 0 / 0 |

`velocity_limit_x/y/z` optionally clamps the outer-loop speed output in m/s;
zero (default) disables this clamp to preserve the original response. Position
integral limits are `[-5, 100] / old_Kd` m/s; velocity integral limits are
`[-5, 100] / mass` m/s². With custom gains, these fixed bounds still apply.
Derivative terms use measured velocity (outer) and finite differences of world
velocity (inner), avoiding reference-step derivative kicks and the different
`state.acc` conventions in legacy and high-fidelity simulation. Inner Kd defaults
to zero; enabling it with noisy or held velocity measurements requires tuning.


### `minitrone_allocator_controller` — Actuator allocation

Uses commanded forces and moments together with measured servo angles to compute four motor speeds and four servo angles, published on `/minitrone/input`.

```bash
ros2 run minitrone_controller minitrone_allocator_controller
```

Enable saturation diagnostics:

```bash
ros2 param set /minitrone_allocator_controller enable_saturation_debug true
```

The input topic is selected with `input_wrench_topic`, which defaults to `/minitrone/wrench_cmd`.

### `minitrone_first_wrench_observer` — First-order external wrench observer

Estimates the external wrench from `/minitrone/mob_observer_input`, which bundles plant state and actual actuator forces and moments. Publishes `/minitrone/external_wrench_hat`.

```bash
ros2 run minitrone_controller minitrone_first_wrench_observer
```

### `minitrone_second_wrench_observer` — Second-order wrench observer and CoP estimation

Uses the same observer input to estimate external forces and moments. Publishes `/minitrone/external_wrench_hat_second_order` and `/minitrone/cop_hat`. This observer supplies the admittance and passive alignment controllers.

```bash
ros2 run minitrone_controller minitrone_second_wrench_observer
```

Axis-specific observer parameters include `omega_n_force_x/y/z`, `omega_n_moment_x/y/z`, `zeta_force_x/y/z`, and `zeta_moment_x/y/z`. CoP settings include `plate_size_y`, `plate_size_z`, `cop_force_min`, and `r_com_to_contact_x/y/z`. Check these settings when changing the drone or contact plate model.

### `minitrone_ekf_state_estimator` — State estimation

Estimates a 15-dimensional state containing position, velocity, attitude, and IMU biases. The default input is `/minitrone/state`, and the output is `/minitrone/state_ekf`. Additional diagnostic outputs are `/ekf/state_vector` and `/ekf/bias`.

```bash
ros2 run minitrone_controller minitrone_ekf_state_estimator
```

**The current acceleration interface has a mismatch.** The EKF and message comments interpret `acc` as body-frame IMU specific force, while the plant currently supplies acceleration obtained by differentiating world-frame velocity. Align these definitions before using the EKF output for control feedback. The default controllers do not use the EKF output.

### `minitrone_admittance_controller` — Six-DOF admittance control

Adjusts position and attitude references using external wrench estimates from the second-order observer. It integrates with the wrench controller through `/minitrone/cmd_admittance`, `/minitrone/att_cmd_admittance`, and `/minitrone/admittance_active`.

With `arm_launch.py` and a position command node running, start this node in a separate terminal:

```bash
ros2 run minitrone_controller minitrone_admittance_controller --ros-args \
  -p f_normal_des:=5.0
```

| Keys | Function |
| --- | --- |
| `O` | Enable admittance / disable it and hold the current pose (HOLD) |
| `P` | Return to ordinary position and attitude command passthrough |
| `U/J` | Increase/decrease the desired normal force, in 0.1 N increments by default |

It can also be enabled through a topic:

```bash
ros2 topic pub --once /minitrone/admittance_enable std_msgs/msg/Bool "{data: true}"
```

Defaults are `enabled=false` and `f_normal_des=5.0` N. Configure virtual mass, damping, and stiffness per axis using `adm_m_*`, `adm_d_*`, and `adm_k_*`.

### `passive_aligning_controller` — Passive contact alignment

Filters `/minitrone/wrench_cmd` to control normal contact force and tangential motion while reducing control moments on the rotational axes used for passive contact alignment. Publishes `/minitrone/wrench_passive_align`.

```bash
ros2 run minitrone_controller passive_aligning_controller --ros-args \
  -p f_normal_des:=5.0
```

Use `O` to enable/disable the controller and `U/J` to adjust the desired normal force. Alternatively, send `std_msgs/msg/Bool` to `/minitrone/passive_align_enable`.

**Starting this node alone does not change the allocator's input.** For passive alignment experiments, stop the existing launch and run the following commands in separate terminals to connect the complete control path:

```bash
# Terminal 1
ros2 run minitrone_plant minitrone_plant
# Terminal 2
ros2 run minitrone_controller minitrone_wrench_controller
# Terminal 3
ros2 run minitrone_controller minitrone_second_wrench_observer
# Terminal 4: connect the filter output to the allocator
ros2 run minitrone_controller minitrone_allocator_controller --ros-args \
  -p input_wrench_topic:=/minitrone/wrench_passive_align
# Terminal 5: press O after startup to enable
ros2 run minitrone_controller passive_aligning_controller
# Terminal 6
ros2 run minitrone_cmd minitrone_position_teleop
```

## 8. Messages and Main Topics

`minitrone_interfaces` provides messages only; it has no executable nodes for `ros2 run`.

| Message | Main Fields and Units |
| --- | --- |
| `Cmd` | `pos_cmd[3]`: world-frame position [m] |
| `AttitudeCmd` | `roll_ref`, `pitch_ref`, `yaw_ref`: attitude references [deg] |
| `Wrench` | `force[3]` [N], `moment[3]` [N·m]; frame and reference point depend on the topic |
| `Input` | `u[0:4]`: motor speed commands; `u[4:8]`: servo commands [rad] |
| `MinitroneState` | `pos`, `vel`: world frame; `rpy`: rad; `w_rpy`: body angular velocity [rad/s]; `servo`: deg. See the EKF section for `acc` |
| `MobObserverInput` | `step`, `sim_time`, state, and actual body-frame actuator forces and moments |
| `CenterOfPressure` | Contact-frame `y`, `z` [m], `normal_force` [N], `valid`, and `corner_forces[4]` |

| Topic | Purpose |
| --- | --- |
| `/minitrone/cmd`, `/minitrone/att_cmd` | Position and attitude references |
| `/minitrone/att_ref` | Effective attitude reference used by the wrench controller, `AttitudeCmd` in degrees; publishes `(0, 0, 0)` during ordinary flight when no attitude command has arrived |
| `/minitrone/att_cmd_admittance` | Admittance attitude offset in degrees; added to `/minitrone/att_cmd` only while admittance is active |
| `/minitrone/wrench_cmd` | Commanded control forces and moments |
| `/minitrone/input` | Motor and servo commands |
| `/minitrone/state` | Plant state |
| `/minitrone/mob_observer_input` | Bundled state and actuation input for wrench observers |
| `/minitrone/actuation_wrench_body` | Actual actuator forces and moments |
| `/minitrone/external_wrench_cmd` | User-specified disturbances |
| `/minitrone/external_wrench_hat_second_order` | Second-order external wrench estimate |
| `/minitrone/cop_real`, `/minitrone/cop_hat` | Contact-derived / observer-estimated CoP |
| `/minitrone/palm_pose_cmd`, `/minitrone/palm_pose_state` | Wall pose command / state |
| `/contact_method1/contact_wrench_ground_truth_com` | Contact wrench about the center of mass |
| `/contact_method1/contact_wrench_ground_truth_C` | Contact wrench about the contact face center |
| `/contact_method1/contact_count`, `/contact_method1/debug/*` | Contact count, force decomposition, contact region, and other diagnostics |

The CoP contact frame C is located at the center of the drone's +X contact face. Its +x axis is the outward normal pointing from the drone toward the wall. A CoP with `valid=false` should not be interpreted as a valid contact location.

```bash
ros2 node list
ros2 topic list
ros2 topic hz /minitrone/state
ros2 topic echo /minitrone/cop_hat
ros2 interface show minitrone_interfaces/msg/MobObserverInput
```


# High-Fidelity Simulation Model

## Purpose and scope

This is a **hardware-like simulator, not an identified digital twin**. The purpose
is to include the dominant non-idealities now and replace their **provisional /
nominal / placeholder** values when hardware experiments become available. No new
number in this section is claimed to have been experimentally identified. The
observed roughly 21–23 ms actuator response and 60–70 ms Dynamixel response are
only motivation for initial model orders and parameter magnitudes. In particular,
a response duration is not automatically a first-order time constant.

`minitrone_plant/high_fidelity.py` implements independent numerical models without
ROS dependencies. `plant.py` loads parameters, configures MuJoCo, advances each
model, and publishes the existing messages. Physics remains **400 Hz / 0.0025 s**.
Controller, allocator, observer algorithms and ROS message definitions are unchanged.

### Architecture and signal flow

```text
/minitrone/cmd + /minitrone/att_cmd
  -> wrench_controller -> /minitrone/wrench_cmd
  -> allocator_controller -> /minitrone/input [omega(4), angle_rad(4)]
  -> separate command transport queues
  -> lumped motor speed state / servo internal reference lag and slew limit
  -> per-motor thrust gain -> final physical thrust saturation
  -> MuJoCo thrust actuators + existing servo position-actuated hinges
  -> rigid-body physics with fixed plant J/CoM and contact uncertainty
       + explicit /minitrone/external_wrench_cmd
       + independent colored external residual
  -> true state -> independent sensor acquisition, bias/noise
  -> timestamped delivery queues -> hold last received sample
  -> /minitrone/state -> existing controller and allocator
  -> /minitrone/mob_observer_input -> existing momentum observers
```

Bias/noise are generated at **acquisition** and delivered with that sample, as in a
sensor packet. This is equivalent to delaying the measured sample, including its
bias and noise, rather than drawing new noise at each 400 Hz ROS publication.

`MobObserverInput` retains a current physics `step` and `sim_time`, current actual
**motor-generated** wrench, and the same delayed/held measured state used by
`MinitroneState`. Different channels may represent different acquisition times;
the existing message has no per-channel timestamps or validity flags. The observer
must therefore tolerate their latency; its algorithm is not retimed here.
`/minitrone/actuation_wrench_body` is computed from MuJoCo `actuator_force`, site
orientation and moment arms. It includes thrust/reaction torque and actual servo
tilt, **never** the explicit or colored external wrench or contact forces.

Body-frame state and actuator-wrench reference remain the existing **nominal body
origin**, so randomized CoM is not silently supplied to the controller. External
wrenches act at the **actual base CoM**, expressed in body axes before world
rotation. Contact ground-truth `..._com` uses actual CoM; `..._C`, contact points,
corner gaps and CoP continue to use the physical plate/body geometry. The default
A/B/C/D diagnostic pair remains the +X plate and palm; ground, box and other
contacts retain their existing configuration.

### Running and configuring

The node defaults to `enable_high_fidelity=false` for existing launch behavior.
Load the installed example to turn it on:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run minitrone_plant minitrone_plant --ros-args \
  --params-file install/minitrone_plant/share/minitrone_plant/config/high_fidelity.yaml \
  -p enable_viewer:=false
```

The source configuration is
[`src/minitrone_plant/config/high_fidelity.yaml`](src/minitrone_plant/config/high_fidelity.yaml).
Build after changing the installed source config, or pass the source path directly.
For a launch `Node`, pass `parameters=[path_to_yaml]`. The existing `arm_launch.py`
remains compatible and defaults to HF OFF. To compare with OFF while keeping the
same configuration, append `-p enable_high_fidelity:=false`. Each individual
`enable_*` switch can also be disabled independently.

All new physical parameters are **startup-only/read-only ROS parameters**. Restart
the plant after editing them; this prevents a misleading successful runtime
parameter update that did not rebuild the actuator/sensor state. Use `random_seed`
for reproducible experiments. Startup logs show switches, realized gains, actual
CoM/principal inertia, contact friction/solref, sensor rates/initial biases, and
residual settings. There is no per-step logging.

## Models and physical meaning

### Motor and propeller

```text
tau_m * d(omega_actual)/dt = omega_cmd_delayed - omega_actual
tau_m = tau_up when command > actual, otherwise tau_down
omega_next = omega + (1 - exp(-dt/tau_m)) * (omega_cmd_delayed - omega)
T_i = g_i * k_T * omega_actual_i^2
0 <= T_i <= min(motor_thrust_max, XML actuator limit)
```

We choose **speed state** because the existing allocator and `Input.u[0:4]` use
omega; thrust is quadratic and a thrust step is therefore not itself a simple
exponential with the same time constant. This is **one lumped ESC + motor +
propeller approximation**. No additional ESC or propeller first-order stage is
cascaded. Exact exponential integration is stable for small positive tau.
22 ms rising / 25 ms falling are provisional; thrust-stand identification must
separate the transport latency from the aggregate response.

The inherited `k_T=0.02` is consistent with allocator command scaling. Its omega
channel follows the simulator's rad/s convention, but this number has **not** been
calibrated against actual hardware shaft speed. Do not interpret the resulting
maximum omega (~27.39 at 15 N) as a hardware motor-speed specification.
Reaction torque per thrust remains the XML `gear` convention (nominal magnitude
0.02 m); this layer does not separately identify `k_Q`.

`g_i = motor_gains[i] * (1 + Uniform(-range, range))` is sampled once at startup.
Set `motor_gain_random_range=0` for fixed-only gains. The default ±3% range is a
small manufacturing/propeller/ESC mismatch placeholder. Gains are logged.

The allocator's 15 N saturation restricts its **nominal requested** wrench. Plant
saturation restricts **actual delivered** thrust after gain mismatch and lag;
a weaker actuator can therefore differ from allocation predictions. The plant
also bounds attainable speed before filtering to prevent saturated state windup.
NaN/Inf input components become zero; negative speed becomes zero; finite huge
speed is bounded before squaring. XML safety limits remain active even with the
HF saturation switch off. Raising a plant limit above XML cannot raise delivered
thrust; intentionally raising capability requires corresponding XML/allocator
review. Existing controller constants are not automatically changed.

### Servo

```text
tau_s * d(alpha_ref)/dt = alpha_cmd_delayed - alpha_ref
|d(alpha_ref)/dt| <= servo_rate_limit
servo_angle_min <= alpha_ref <= servo_angle_max
alpha_measured = MuJoCo hinge angle, not alpha_ref
```

A 5 ms transport delay, 40 ms reference time constant and 5 rad/s reference slew
are provisional. The **existing XML position servo, joint damping and armature
remain physical**, so the actual hinge follows this internal reference with its
own load-dependent mechanics. The reference has a hard slew bound; the actual
hinge can overshoot or move faster under external forces. This is not a hard
kinematic clamp on `qpos/qvel`, nor a claim that actual measured tau is exactly
40 ms. It avoids teleporting the joint and retains contact/load reactions.
Total command-to-measured response includes both layers; the reported 60–70 ms
hardware lag is **not** independently assigned to each layer. **추후
step-response identification으로 교체 예정**: jointly fit delay, reference tau/rate
and XML `kp`, `kv`, joint damping/armature from Dynamixel commands and measurements.
If hardware response can be explained by the XML mechanics alone, disable the
extra servo dynamics or fit a short reference tau. Limits default to ±65° to match
the allocator, inside the XML ±90° travel. Message servo feedback remains **degrees**;
all internal parameters/noise are radians.

### Transport and sensor sampling

Command/sensor queues use **simulation time**, without blocking latency sleeps.
Requested delays in `[0, dt)` are treated as **zero**, including 0.15 ms. Longer
nonintegral delays round **up** to a physics tick; exact tick multiples are exact.
For example, 6 ms becomes 7.5 ms. No sub-step interpolation is claimed.

```text
y_acquired[k] = x_true[k] + b[k] + noise_std * N(0,I)
b[0] = configured_bias + initial_bias_std * N(0,I)
b[k+1] = b[k] + bias_random_walk_std * sqrt(delta_t_sample) * N(0,I)
y_published(t) = most recent acquired sample whose delivery_time <= t
```

Bias random-walk units are channel-unit/√s, white noise is standard deviation
**per sample**, and initial bias is sampled only once. Acquisition rates must be
positive and at most 400 Hz. Rates that do not divide 400 retain phase and acquire
on the next available physics tick (up to one tick of timing jitter). Every
channel holds its last delivered sample between updates. Before its first packet
arrives it holds the initial true value because the message lacks an invalid flag.
Noise never modifies MuJoCo state, external wrench, or contact truth.

The HF acceleration channel uses truth velocity differences at 400 Hz and then
`R_WB.T * (a_W - gravity_W)` to supply **body specific force**, as the existing
message/EKF expects. Angular acceleration also differentiates true gyro before
sampling/noise. The OFF path retains legacy white noise and world-acceleration
semantics for compatibility, but uses deterministic simulation dt. Attitude
noise is a small additive RPY approximation, not a full inertial-navigation filter.

### Inertia and CoM uncertainty

```text
J_principal_actual = J_principal_XML * inertia_scale * (1 + delta_J)
r_com_actual_body = body_ipos_XML + com_offset + delta_r
```

Default startup half-ranges are ±5% for principal inertia and ±3 mm for CoM.
These represent small, unknown mass-distribution differences, not identified
errors. Zero random ranges select deterministic offsets/scales. Body mass is
preserved. The controller/observer mass, J and CoM parameters are **not updated**.

MuJoCo `body_inertia` contains principal moments **about the inertial CoM**, in
`body_iquat` axes. `body_ipos` is that CoM position in the body frame. We preserve
`body_iquat`, scale principal moments, and shift `body_ipos`; this defines inertia
about the new CoM without adding a second parallel-axis term. For nonidentity
`body_iquat`, axis scales are principal-frame scales, not body-X/Y/Z scales.
Finite positive moments and the triangle inequality are required; invalid draws
fail startup with a message rather than silently rebalancing them. `mj_setConst`
and `mj_forward` update derived physics after mutation. In 3.3.7 the compiled
`body_sameframe` shortcut must also be invalidated, otherwise an initially zero
CoM offset can still produce `xipos == xpos` after editing `body_ipos`. The code
uses general kinematics (and disables simple-inertia shortcuts if necessary),
and tests the actual world CoM displacement. XML BODY frame sensors refer to the
inertial CoM; when uncertainty is active, body-origin position/velocity are obtained
from `xpos` / `mj_objectVelocity(mjOBJ_XBODY)` to keep the tracker reference fixed. The existing combined
Follower inertia is the nominal starting point; controller mismatch already
exists if controller defaults still describe the older 2.5 kg model.

Conventions were checked against the MuJoCo 3.3.7
[model fields](https://mujoco.readthedocs.io/en/3.3.7/APIreference/APItypes.html)
and [simulation guidance](https://mujoco.readthedocs.io/en/3.3.7/programming/simulation.html).

### Contact uncertainty

Select the existing A/B/C/D nominal contact case **first**, then apply one fixed
startup set of scales to the palm and +X plate. This retains collision masks,
priority, `condim`, `solimp`, margin, CoP and visualization logic. Case A's zero
friction remains zero. Case D selects XML values and may then be perturbed if
contact uncertainty is enabled; disable that switch for the unperturbed D baseline.

```text
friction_actual = friction_nominal * sampled_friction_scale
positive solref: timeconst *= sampled_time_scale; dampratio *= sampled_damping_scale
negative solref: stiffness /= sampled_time_scale^2; damping *= sampled_damping_scale
```

Positive time constants are floored at `2*dt` (5 ms) for solver stability; actual
solref is logged. Negative `solref` is MuJoCo's direct `(-stiffness,-damping)`
format, so applying a time constant multiplier directly to its first entry would
be incorrect. Mixed-sign formats are rejected for uncertainty. The damping
multiplier acts on direct damping in negative format, not on an inferred damping
ratio. Defaults are unit scales with ±5% random variation. This is a small layer
over existing contact, not a new contact law; values require contact experiments.
See MuJoCo's [contact model](https://mujoco.readthedocs.io/en/3.3.7/modeling.html#solver-parameters).

### Colored external wrench

```text
d = [Fx, Fy, Fz, Mx, My, Mz]  # body axes, torque about actual base CoM
rho = exp(-dt / residual_time_constant)
d[k+1] = rho*d[k] + stationary_std*sqrt(1-rho^2)*N(0,I)
```

This is an exact discretization of an Ornstein–Uhlenbeck process with stationary
standard deviation `stationary_std`; equivalently its continuous diffusion is
`stationary_std*sqrt(2/tau)`. It starts at zero. Defaults are 0.05 N per force axis,
0.002 N·m per moment axis and 0.2 s correlation. They represent small correlated
unmodeled effects, not measured turbulence or an aerodynamic model. There is no
hard amplitude cap on the Gaussian OU distribution.

The residual is added to `xfrc_applied` **alongside**, not in place of, the explicit
external command. The existing explicit-command wall-clock timeout remains
unchanged; replaying ROS arrival timing is outside seeded stochastic determinism.
Neither external term appears in `actuation_force` / `actuation_moment`.

## Parameters

Every entry below is exposed as a ROS parameter and in the example YAML. Defaults
are code defaults except the YAML sets the master switch to true. Array values
follow motor order 1–4 or axis order X/Y/Z. `random_seed` defaults to 1 (integer,
configuration rather than a physical identified parameter).

| Parameter name | Unit | Default | Physical meaning | Status | Future identification |
| --- | --- | --- | --- | --- | --- |
| `enable_high_fidelity` | - | `false` | Master switch; example YAML enables it | configuration | configuration |
| `enable_motor_dynamics` | - | `true` | Enable motor dynamics | configuration | configuration |
| `enable_servo_dynamics` | - | `true` | Enable servo dynamics | configuration | configuration |
| `enable_motor_mismatch` | - | `true` | Enable motor mismatch | configuration | configuration |
| `enable_thrust_saturation` | - | `true` | Enable thrust saturation | configuration | configuration |
| `enable_sensor_model` | - | `true` | Enable sensor model | configuration | configuration |
| `enable_inertial_uncertainty` | - | `true` | Enable inertial uncertainty | configuration | configuration |
| `enable_contact_uncertainty` | - | `true` | Enable contact uncertainty | configuration | configuration |
| `enable_residual_wrench` | - | `true` | Enable residual wrench | configuration | configuration |
| `legacy_input_delay` | s | `0.0075` | OFF-path effective legacy delay (old 4-slot ring delayed 3 ticks) | provisional / nominal, not identified | transport timestamp test |
| `motor_command_delay` | s | `0.01` | Motor transport delay, separate from lumped response | provisional / nominal, not identified | command/ESC timestamp test |
| `motor_time_constant_up` | s | `0.022` | Lumped motor+prop speed rising time constant | provisional / nominal, not identified | thrust stand step test |
| `motor_time_constant_down` | s | `0.025` | Lumped motor+prop speed falling time constant | provisional / nominal, not identified | thrust stand step test |
| `motor_thrust_coefficient` | N/(rad/s)^2 | `0.02` | Existing allocator omega convention; not calibrated physical shaft speed | provisional / nominal, not identified | thrust stand and command/shaft-speed calibration |
| `motor_gains` | - | `[1.0, 1.0, 1.0, 1.0]` | Fixed per-motor gain multiplier | provisional / nominal, not identified | individual thrust tests |
| `motor_gain_random_range` | - | `0.03` | Startup uniform fractional gain half-range; zero selects fixed only | provisional / nominal, not identified | individual thrust tests |
| `motor_thrust_max` | N | `15.0` | Final per-motor plant limit, intersected with XML limit | provisional / nominal, not identified | maximum continuous thrust test |
| `servo_time_constant` | s | `0.04` | First-order internal servo reference lag, before XML joint response | provisional / nominal, not identified | Dynamixel step-response identification |
| `servo_command_delay` | s | `0.005` | Servo transport delay | provisional / nominal, not identified | Dynamixel step-response identification |
| `servo_rate_limit` | rad/s | `5.0` | Internal servo reference slew limit | provisional / nominal, not identified | Dynamixel step-response identification |
| `servo_angle_min` | rad | `-1.1344640137963142` | Minimum reference angle (-65 deg) | provisional / nominal, not identified | servo travel test |
| `servo_angle_max` | rad | `1.1344640137963142` | Maximum reference angle (+65 deg) | provisional / nominal, not identified | servo travel test |
| `inertia_scale` | - | `[1.0, 1.0, 1.0]` | Fixed multipliers in XML principal-inertia frame | provisional / nominal, not identified | pendulum/system identification |
| `inertia_random_range` | - | `0.05` | Startup uniform fractional principal-inertia half-range | provisional / nominal, not identified | pendulum/system identification |
| `com_offset` | m | `[0.0, 0.0, 0.0]` | Fixed CoM displacement in body axes relative to XML ipos | provisional / nominal, not identified | balancing/suspension test |
| `com_random_range` | m | `0.003` | Startup uniform CoM half-range per body axis | provisional / nominal, not identified | balancing/suspension test |
| `contact_friction_scale` | - | `1.0` | Multiplier on nominal geom friction | provisional / nominal, not identified | sliding contact experiment |
| `contact_time_constant_scale` | - | `1.0` | Positive solref timeconst multiplier; direct stiffness divided by scale squared | provisional / nominal, not identified | contact compliance test |
| `contact_damping_scale` | - | `1.0` | Positive solref dampratio or direct damping multiplier | provisional / nominal, not identified | contact damping test |
| `contact_random_range` | - | `0.05` | Startup uniform fractional half-range on each contact scale | provisional / nominal, not identified | contact experiments |
| `residual_time_constant` | s | `0.2` | OU correlation time | provisional / nominal, not identified | real-versus-sim residual autocorrelation |
| `residual_force_std` | N | `[0.05, 0.05, 0.05]` | Stationary OU force standard deviation in body axes | provisional / nominal, not identified | real-versus-sim residual logs |
| `residual_moment_std` | N*m | `[0.002, 0.002, 0.002]` | Stationary OU moment standard deviation about actual CoM | provisional / nominal, not identified | real-versus-sim residual logs |
| `sensor_position_update_rate_hz` | Hz | `100.0` | Acquisition rate; held between samples | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_position_latency_sec` | s | `0.01` | Simulation-time sample delivery latency | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_position_noise_std` | m | `0.001` | Per-sample white noise standard deviation per axis | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_position_bias` | m | `[0.0, 0.0, 0.0]` | Constant bias vector | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_position_initial_bias_std` | m | `0.001` | Startup Gaussian bias standard deviation per axis | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_position_bias_random_walk_std` | m/sqrt(s) | `0.0001` | Bias diffusion; increments scale with sqrt(sample dt) | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_velocity_update_rate_hz` | Hz | `100.0` | Acquisition rate; held between samples | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_velocity_latency_sec` | s | `0.01` | Simulation-time sample delivery latency | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_velocity_noise_std` | m/s | `0.001` | Per-sample white noise standard deviation per axis | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_velocity_bias` | m/s | `[0.0, 0.0, 0.0]` | Constant bias vector | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_velocity_initial_bias_std` | m/s | `0.001` | Startup Gaussian bias standard deviation per axis | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_velocity_bias_random_walk_std` | m/s/sqrt(s) | `0.0001` | Bias diffusion; increments scale with sqrt(sample dt) | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_attitude_update_rate_hz` | Hz | `400.0` | Acquisition rate; held between samples | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_attitude_latency_sec` | s | `0.0025` | Simulation-time sample delivery latency | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_attitude_noise_std` | rad | `0.001` | Per-sample white noise standard deviation per axis | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_attitude_bias` | rad | `[0.0, 0.0, 0.0]` | Constant bias vector | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_attitude_initial_bias_std` | rad | `0.0005` | Startup Gaussian bias standard deviation per axis | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_attitude_bias_random_walk_std` | rad/sqrt(s) | `1e-05` | Bias diffusion; increments scale with sqrt(sample dt) | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_gyro_update_rate_hz` | Hz | `400.0` | Acquisition rate; held between samples | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_gyro_latency_sec` | s | `0.0025` | Simulation-time sample delivery latency | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_gyro_noise_std` | rad/s | `0.001` | Per-sample white noise standard deviation per axis | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_gyro_bias` | rad/s | `[0.0, 0.0, 0.0]` | Constant bias vector | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_gyro_initial_bias_std` | rad/s | `0.001` | Startup Gaussian bias standard deviation per axis | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_gyro_bias_random_walk_std` | rad/s/sqrt(s) | `0.0001` | Bias diffusion; increments scale with sqrt(sample dt) | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_acceleration_update_rate_hz` | Hz | `400.0` | Acquisition rate; held between samples | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_acceleration_latency_sec` | s | `0.0025` | Simulation-time sample delivery latency | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_acceleration_noise_std` | m/s^2 | `0.02` | Per-sample white noise standard deviation per axis | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_acceleration_bias` | m/s^2 | `[0.0, 0.0, 0.0]` | Constant bias vector | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_acceleration_initial_bias_std` | m/s^2 | `0.01` | Startup Gaussian bias standard deviation per axis | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_acceleration_bias_random_walk_std` | m/s^2/sqrt(s) | `0.001` | Bias diffusion; increments scale with sqrt(sample dt) | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_angular_acceleration_update_rate_hz` | Hz | `400.0` | Acquisition rate; held between samples | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_angular_acceleration_latency_sec` | s | `0.0025` | Simulation-time sample delivery latency | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_angular_acceleration_noise_std` | rad/s^2 | `0.01` | Per-sample white noise standard deviation per axis | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_angular_acceleration_bias` | rad/s^2 | `[0.0, 0.0, 0.0]` | Constant bias vector | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_angular_acceleration_initial_bias_std` | rad/s^2 | `0.001` | Startup Gaussian bias standard deviation per axis | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_angular_acceleration_bias_random_walk_std` | rad/s^2/sqrt(s) | `0.0001` | Bias diffusion; increments scale with sqrt(sample dt) | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_servo_update_rate_hz` | Hz | `50.0` | Acquisition rate; held between samples | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_servo_latency_sec` | s | `0.005` | Simulation-time sample delivery latency | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_servo_noise_std` | rad | `0.0001` | Per-sample white noise standard deviation per axis | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_servo_bias` | rad | `[0.0, 0.0, 0.0, 0.0]` | Constant bias vector | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_servo_initial_bias_std` | rad | `0.0001` | Startup Gaussian bias standard deviation per axis | provisional / nominal, not identified | static sensor logs / timestamp test |
| `sensor_servo_bias_random_walk_std` | rad/sqrt(s) | `1e-05` | Bias diffusion; increments scale with sqrt(sample dt) | provisional / nominal, not identified | static sensor logs / timestamp test |

## Uncertainty versus time-varying noise

**Fixed for a run:** motor gain, principal-inertia scales, CoM offset, contact
scales, constant/initial sensor biases. **Time-varying:** per-acquisition white
sensor noise, bias random walks and colored external residual. Stable independent
NumPy seed streams separate motor, inertia, contact, residual and each sensor.
The same seed/configuration and simulation-step input history reproduce samples.
Toggling a sensor does not change motor mismatch or residual noise. Bitwise
trajectory determinism across different MuJoCo/NumPy versions or nondeterministic
ROS command arrival schedules is not promised. No randomization occurs in the
viewer or according to wall-clock callback frequency.

## Compatibility, limitations and known pre-existing issues

- HF OFF (or all individual features off at defaults) retains legacy command
  scaling, the effective 7.5 ms delay, original XML servo mechanics and white
  sensor noise. The old ring buffer was labelled 10 ms but delivered after three
  2.5 ms intervals. New delays are explicitly configured, without that off-by-one.
- The previous catch-up loop could step physics several times with one actuator
  update and publish duplicate current samples. It now advances and publishes
  exactly once per physics step, including during catch-up. `mj_forward` refreshes
  integrated-state sensors. Together with simulation-time differentiation this
  means OFF is compatible, not bit-for-bit equivalent to wall-clock-jittered logs.
- The previous `acc` publication differentiated noisy world velocity using wall
  time, contrary to the message's body-specific-force comment. The HF sensor
  path fixes this; OFF deliberately preserves the old frame meaning. The existing
  EKF is optional and controllers are not automatically redirected to its output.
- `Scene.xml` currently selects Follower (2.86 kg); wrench controller and second
  observer defaults still assume 2.5 kg and older J. The first observer has still
  older 4 kg/J defaults. The allocator also contains fixed geometry and thrust
  constants. They are intentionally not changed by this plant-only work; account
  for these baseline mismatches when interpreting observer residuals.
- Rotor interaction, ground effect, battery voltage/sag, detailed ESC electronics,
  structural flexibility, backlash and aerodynamic drag identification are not
  included. Residual wrench is phenomenological and does not replace those models.
- The servo reference rate is limited, not externally forced hinge motion. Existing
  joint dynamics add lag; total measured step response must be fitted jointly.
- No sub-physics-step delay is resolved. No per-channel sensor timestamp/validity
  or estimator pipeline is added to the existing messages. Latency-induced state/
  current-wrench misalignment is intentionally visible to the unchanged observer.
- J/CoM/contact values remain unmeasured; contact randomization currently targets
  the existing +X plate/palm diagnostic pair, not every floor/box/back-plate contact.
- Startup validation rejects negative/nonfinite tau, rate, standard deviation,
  invalid array shapes, nonpositive gains/scales, impossible sample rates and
  nonphysical inertia. Hardware plausibility still requires experimental review.

## Identification roadmap

1. **Thrust stand:** calibrate command versus actual shaft omega, `k_T`, XML
   reaction-torque gear (`k_Q/k_T`), per-motor gains, max thrust and asymmetric
   lumped time constants. Keep transport delay separate from response lag.
2. **Dynamixel step test:** fit transport delay, reference tau/rate, travel and
   XML joint `kp`, `kv`, damping/armature jointly against actual angle logs.
3. **Pendulum / flight system identification:** replace XML nominal principal J
   and its orientation; use the uncertainty ranges for remaining error, not to
   masquerade as the identified nominal value.
4. **Balancing / suspension:** replace nominal XML inertial position or measured
   fixed `com_offset`; reduce the random range when uncertainty is quantified.
5. **Static sensor and timestamp logs:** estimate each channel's bias, per-sample
   white noise, random-walk diffusion, sample rate and delivery latency. Do not
   conflate diffusion per √s with noise per acquisition.
6. **Contact tests:** fit nominal friction/compliance/damping in existing contact
   settings, then use the scale ranges to describe remaining run variation.
7. **Matched hardware/simulation experiments:** fit stationary wrench residual
   magnitude and correlation time after the identifiable mechanisms are calibrated.

Edit `config/high_fidelity.yaml` and restart for high-fidelity parameter changes.
Change the selected drone XML for nominal mass/J and servo mechanics; controller
nominal assumptions must be reviewed separately and are never auto-synchronized.

## Validation

Numerical behavioral tests cover analytical asymmetric motor steps, servo reference
slew/travel, causal transport, sample/hold and biased latency, invalid input handling,
legacy OFF equivalence, deterministic independent streams, physical inertia,
positive/direct contact solref scaling and OU RMS/correlation. Run:

```bash
PYTHONPATH=src/minitrone_plant python3 -m pytest -q src/minitrone_plant/test/test_high_fidelity.py
python3 -m py_compile src/minitrone_plant/minitrone_plant/plant.py src/minitrone_plant/minitrone_plant/high_fidelity.py
source /opt/ros/humble/setup.bash
colcon build --packages-select minitrone_interfaces minitrone_plant minitrone_controller minitrone_cmd
```

Validation completed on Ubuntu 22.04 / ROS 2 Humble / MuJoCo 3.3.7:

| Check | Result |
| --- | --- |
| `py_compile` for both model modules and all three added test files | PASS |
| Selected ROS build: interfaces, plant, controller, cmd | All 4 packages built |
| Numerical model tests | 22 passed |
| Actual plant integration tests | 16 passed; HF OFF/ON, all A/B/C/D cases, actual plate contact, servo hinge lag, external-wrench separation, actual CoM shift, identical-seed trajectories |
| Installed `ros2 run`, installed YAML, viewer off | 2 passed, OFF/ON each run for 4 wall seconds; about 3.58 seconds of received simulation data after startup |
| Live topic reception per final OFF/ON run | 1432 observer messages and 1433 state messages each; finite values, monotonic step/time at 0.0025 s per step |
| Motor step comparison | OFF jumps after transport; ON has multiple intermediate thrust samples |
| MuJoCo numerical warnings / Python crashes | None in successful tests |

The ROS smoke test uses an isolated ROS domain. The sandbox restricted UDP DDS
sockets, so the final socket-enabled smoke validation ran outside the sandbox.
GUI rendering, closed-loop flight performance and agreement with real hardware
were not validated by these tests.

After sourcing ROS and the built workspace, run the integration checks with:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -s \
  src/minitrone_plant/test/test_high_fidelity_integration.py \
  src/minitrone_plant/test/test_high_fidelity_smoke.py
```
