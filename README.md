# Minitrone MuJoCo ROS 2 Workspace

This workspace is derived from `PPP_sim` and loads the single MuJoCo model:

```text
src/minitrone_plant/xml/scene.xml
```

The model is copied from:

```text
/home/parkjeongsu/ros2_project/minitrone/minitrone_400x400x300_armature.xml
```

## Build

```bash
source /opt/ros/humble/setup.bash
cd /home/parkjeongsu/ros2_project/minitrone_ws
colcon build --executor sequential
source install/setup.bash
```

## Run

```bash
ros2 launch minitrone_cmd pt_launch.py
```

For headless use, run the plant without the MuJoCo viewer:

```bash
ros2 run minitrone_plant minitrone_plant --ros-args -p enable_viewer:=false
```

When the MuJoCo viewer opens, propeller visual groups and the contact-plate
resultant-force arrow are enabled by default. Press `F` in the viewer to toggle
the red wall-on-plate force arrow. Initial visibility can also be configured:

```bash
ros2 run minitrone_plant minitrone_plant --ros-args \
  -p viewer_show_propellers:=true \
  -p viewer_show_contact_forces:=true
```

For a live PyQtGraph view of `/minitrone/external_wrench_hat_second_order`:

```bash
sudo apt install python3-pyqt5 python3-pyqtgraph
ros2 run minitrone_plant minitrone_external_wrench_plot
```

Optional plot settings:

```bash
ros2 run minitrone_plant minitrone_external_wrench_plot --ros-args -p window_sec:=15.0 -p refresh_hz:=40.0
```

The plot layout can be changed in the window from `1 x 1` through `3 x 3`.
To choose the initial layout from the command line, set `plot_rows` and
`plot_columns` (each accepts 1 through 3):

```bash
ros2 run minitrone_plant minitrone_external_wrench_plot --ros-args -p plot_rows:=3 -p plot_columns:=3
```

The external-wrench view also shows the Center of Pressure (CoP) on a BODY
`+X` contact surface. Its plate size, minimum valid normal force, trail length,
and sign convention can be adjusted with ROS parameters:

```bash
ros2 run minitrone_plant minitrone_external_wrench_plot --ros-args \
  -p plate_size_y:=0.40 -p plate_size_z:=0.43 \
  -p cop_force_min:=0.5 -p cop_trail_length:=100 \
  -p cop_y_sign:=1.0 -p cop_z_sign:=-1.0
```

For a generic topic picker like `rqt_plot`, use:

```bash
ros2 run minitrone_plant minitrone_topic_plot
```

Select a topic on the left, choose one or more numeric fields, and add them to the plot.
Variable-length arrays appear as `field[]` first and expand to `field[0]`, `field[1]`, ... after the first message is received for that topic.

## Minitrone-Specific Changes

- `minitrone_plant` maps XML actuators `prop1..prop4` and `servo1..servo4`.
- `minitrone_wrench_controller` uses the XML body mass `2.5 kg` by default.
- `minitrone_allocator_controller` uses the minitrone arm geometry:
  - prop xy radius: `0.137593847 m`
  - prop height relative to CoM: `0.08348805 m`
  - servo limit: `+/-0.51 rad`
- `/minitrone/input` keeps the PPP_sim convention:
  - `u[0..3]`: motor speed command
  - `u[4..7]`: servo angle command in radians
