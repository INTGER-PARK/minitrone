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
