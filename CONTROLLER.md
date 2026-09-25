# Palletrone MuJoCo four-layer controller

## Source of truth and architecture

The hardware reference is `~/HIH_PX4`. Its position and velocity layers are in
`src/modules/mc_pos_control/PositionControl/PositionControl.cpp` and
`MulticopterPositionControl.cpp`; attitude is in
`src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp`; rate is in
`src/lib/rate_control/rate_control.cpp`. Parameter defaults are in their
`*_params.yaml` files. The actual firmware does **not** expose 36 independent
PID gains: position is PD with shared horizontal gains, velocity is PID with
shared horizontal gains, attitude is PD, and rate is PID. That is 4 + 6 + 6 + 9
= 25 independent P/I/D scalar parameters. The rate loop also has separate
`MC_*RATE_K` multipliers and feedforward parameters, which are not additional
P/I/D gains. This simulator retains the four layers and expands every layer
to independent XYZ or roll/pitch/yaw PID channels as requested.

| PX4 layer | Confirmed P/I/D parameter names | Independent scalar count |
| --- | --- | ---: |
| Position | `MPC_XY_P`, `MPC_Z_P`, `MPC_XY_D`, `MPC_Z_D` | 4 |
| Velocity | `MPC_XY_VEL_P_ACC`, `MPC_Z_VEL_P_ACC`, `MPC_XY_VEL_I_ACC`, `MPC_Z_VEL_I_ACC`, `MPC_XY_VEL_D_ACC`, `MPC_Z_VEL_D_ACC` | 6 |
| Attitude | `MC_ROLL_P`, `MC_PITCH_P`, `MC_YAW_P`, `MC_ROLL_D`, `MC_PITCH_D`, `MC_YAW_D` | 6 |
| Rate | `MC_ROLLRATE_P/I/D`, `MC_PITCHRATE_P/I/D`, `MC_YAWRATE_P/I/D` | 9 |
| **Total** | | **25** |

| Stage | PX4 Palletrone | Previous MuJoCo | Current MuJoCo |
| --- | --- | --- | --- |
| Position | PD, XY gain sharing; velocity setpoint | XYZ PID | XYZ PID, 9 gains |
| Velocity | PID, XY gain sharing; body force | XYZ PID | XYZ PID, 9 gains; world force |
| Attitude | Quaternion error PD; body rate setpoint | Euler error PID directly to torque | Quaternion error PID; body rate setpoint, 9 gains |
| Rate | PID; body torque | absent | XYZ body rate PID, 9 gains |
| Allocation | PX4 control allocator | MuJoCo allocator | existing MuJoCo allocator retained |

The complete simulation path is

```text
/minitrone/state + /minitrone/cmd + /minitrone/att_cmd
  -> Position PID -> velocity setpoint -> Velocity PID -> world force
  -> Attitude PID -> body rate setpoint -> Rate PID -> body torque
  -> body wrench /minitrone/wrench_cmd
  -> existing allocator -> /minitrone/input -> 4 BLDC + 4 tilt servos
  -> plant actuator dynamics, sensor model, and MuJoCo physics
```

Admittance position replaces the position reference while active. Its attitude
message is an angular offset added to the ordinary attitude reference.
Translation and rotation remain independent, consistent with a fully actuated
thrust-vectoring vehicle.

## All 36 scalar gains

The single default table is `kGainDefaults` near the top of
`src/minitrone_controller/src/wrench_controller.cpp`. Each row declares three
separate ROS parameters, named `KP_<LAYER>_<AXIS>`, `KI_...`, and `KD_...`.
Every value is independently tunable at startup with `--ros-args -p` or while
running with `ros2 param set /minitrone_wrench_controller NAME VALUE`.
Changing a gain resets the four PID integrators and derivative filters.

| Controller | Axis | Kp | Ki | Kd | Output unit / input unit | Source / initial value |
| --- | --- | ---: | ---: | ---: | --- | --- |
| Position | X | 4.666667 | 0.25 | 0.01 | (m/s)/m | MuJoCo legacy split; D provisional |
| Position | Y | 4.666667 | 0.25 | 0.01 | (m/s)/m | MuJoCo legacy split; D provisional |
| Position | Z | 2.4 | 0.12 | 0.01 | (m/s)/m | MuJoCo legacy split; D provisional |
| Velocity | X | 2.097902 | 0.01 | 0.01 | (m/s²)/(m/s) | MuJoCo legacy force split at compiled 2.86 kg; I/D provisional |
| Velocity | Y | 2.097902 | 0.01 | 0.01 | (m/s²)/(m/s) | MuJoCo legacy force split at compiled 2.86 kg; I/D provisional |
| Velocity | Z | 3.496503 | 0.01 | 0.01 | (m/s²)/(m/s) | MuJoCo legacy force split at compiled 2.86 kg; I/D provisional |
| Attitude | Roll | 6.0 | 0.02 | 0.8 | (rad/s)/rad | MuJoCo prior torque gains mapped to rate; I provisional |
| Attitude | Pitch | 6.0 | 0.02 | 0.8 | (rad/s)/rad | MuJoCo prior torque gains mapped to rate; I provisional |
| Attitude | Yaw | 6.0 | 0.02 | 0.8 | (rad/s)/rad | MuJoCo prior torque gains mapped to rate; I provisional |
| Rate | Roll | 1.0 | 0.001 | 0.001 | (N m)/(rad/s) | Unity mapping of previous attitude output; I/D provisional |
| Rate | Pitch | 1.0 | 0.001 | 0.001 | (N m)/(rad/s) | Unity mapping of previous attitude output; I/D provisional |
| Rate | Yaw | 1.0 | 0.001 | 0.001 | (N m)/(rad/s) | Unity mapping of previous attitude output; I/D provisional |

Each P, I, and D column contains one scalar per row: 12 × 3 = **36 independent
PID gains**. I and D units include the time dimension implied by integration
and differentiation, respectively. No PX4 numerical gain was copied blindly.
The old MuJoCo controller used `mass=2.5` kg, while the compiled free-body
model reports 2.86 kg after its geometries are included. The new nominal mass
is 2.86 kg; velocity Kp values were rescaled from `6/2.5, 6/2.5, 10/2.5` to
`6/2.86, 6/2.86, 10/2.86` to retain the legacy force gain product. This also
corrects hover gravity compensation instead of hiding the mass error in I gain.

The 36 ROS names are:

```text
KP_POS_X KI_POS_X KD_POS_X    KP_POS_Y KI_POS_Y KD_POS_Y    KP_POS_Z KI_POS_Z KD_POS_Z
KP_VEL_X KI_VEL_X KD_VEL_X    KP_VEL_Y KI_VEL_Y KD_VEL_Y    KP_VEL_Z KI_VEL_Z KD_VEL_Z
KP_ATT_ROLL KI_ATT_ROLL KD_ATT_ROLL    KP_ATT_PITCH KI_ATT_PITCH KD_ATT_PITCH    KP_ATT_YAW KI_ATT_YAW KD_ATT_YAW
KP_RATE_ROLL KI_RATE_ROLL KD_RATE_ROLL    KP_RATE_PITCH KI_RATE_PITCH KD_RATE_PITCH    KP_RATE_YAW KI_RATE_YAW KD_RATE_YAW
```

For comparison, PX4's default position P is XY/Z = 0.95/1.0 and D = 0/0;
velocity P/I/D is XY = 1.8/0.4/0.2 and Z = 4/2/0; attitude P is
roll/pitch/yaw = 4/4/2.8 with D = 0/0/0; and rate P/I/D is
roll/pitch = 0.15/0.2/0.003 and yaw = 0.2/0.1/0. Position I and attitude I
are absent from that firmware. The MuJoCo initial values above preserve the
prior simulation response approximately; they require closed-loop tuning.

## Equations, frames, and update frequency

For each axis, each `PidLayer` computes `P = Kp*e`, `I = Ki*integral(e)`,
`D = -Kd*filtered derivative(measurement)`, then clamps `P+I+D`.

- Position: `e_p = p_sp - p`, output `v_sp` in world frame.
- Velocity: `e_v = v_sp - v`, output `a_cmd` in world frame.
- Force: `F_W = m*(a_cmd - g_W)`, with `g_W = [0,0,-9.81]` m/s²;
  `F_B = R_WBᵀ F_W`. MuJoCo world +Z is up.
- Attitude: `R_WB = Rz(yaw) Ry(pitch) Rx(roll)` and the body-frame shortest
  quaternion error is `e_R = 2*imag(canonical(quat(R_WBᵀ R_sp)))`. MuJoCo's
  source quaternion is scalar-first `wxyz`; the state message supplies XYZ
  Euler radians derived from that quaternion. The PID output is body `omega_sp`.
- Rate: `e_omega = omega_sp - omega_B`; output body torque `tau_B`.
- The `Wrench` message carries `force=F_B` and `moment=tau_B`. The existing
  allocator reorders these fields internally before calculating motor speeds
  and servo angles.

The plant overrides the XML timestep to 1/400 s. Controller callbacks run on
fresh state messages, nominally 400 Hz. The four PID states are kept separately
so their update rates can be split later.

## Limits, integral handling, and derivative handling

Default per-axis limits are velocity `[2,2,1]` m/s, acceleration `[20,20,20]`
m/s², body rate `[4,4,3]` rad/s, and torque `[5,5,5]` N m. The corresponding
ROS limit parameters are declared beside the gain table. Each layer clamps
its output and its error integral; the four scalar integral magnitude limits
default to 10, 10, 1, and 1 in position, velocity, attitude, and rate order.
Conditional integration stops accumulation further into a local output limit.
The allocator publishes `/minitrone/allocator_saturated`; on the next control
step, the controller freezes all integrators while actuator saturation persists.
The allocator keeps its existing 0–15 N motor and ±65° servo limits. Because
the allocator is nonlinear, its `/minitrone/allocated_wrench_estimate` is a
prediction of the saturated command before plant dynamics.

PX4 uses measured acceleration for velocity and rate D terms. The simulator's
`state.acc` changes convention between its nominal and high-fidelity paths,
so this controller differentiates measured world velocity and body gyro,
then low-pass filters both at `derivative_cutoff_hz=30`. Position D uses
filtered measured world velocity; attitude D uses filtered body gyro. This
avoids setpoint derivative kicks and keeps the four D gains independently
tunable. Controller reset and admittance transitions clear all four integrals
and derivative filters. Publish `true` to `/minitrone/controller_reset` to
reset explicitly; invalid or discontinuous controller time also resets them.

## Tuning and logging

Use `ros2 param list /minitrone_wrench_controller` to inspect the 36 gains.
For example, `ros2 param set /minitrone_wrench_controller KP_RATE_YAW 0.8`
changes only yaw-rate proportional gain. Start with the hover reference at the
initial position, roll/pitch zero, and initial yaw, then tune one axis at a
time. Source values and initial values differ because the PX4 airframe and
MuJoCo dynamics are not identical.

`/minitrone/controller_debug/{position,velocity,attitude,rate}` each publish
seven consecutive XYZ triples in a `Float64MultiArray`: setpoint, measured,
error, P, I, D, output. Attitude's error triple is the body rotation error,
while setpoint and measured triples are Euler radians for display.
`/minitrone/controller_debug/wrench` publishes world force, body force, and
body torque triples. Record `/minitrone/wrench_cmd`,
`/minitrone/allocated_wrench_estimate`, `/minitrone/allocator_saturated`, and
`/minitrone/input` for desired wrench, predicted allocated wrench, motor
commands, and servo commands. Plant state and existing bags provide the
measured response.

## Validation

`python3 -m pytest -q src/minitrone_controller/test/test_wrench_cascade.py`
checks all 36 names through ROS parameter services, independent cascade
channels, live gain update, saturation, admittance offset, and static hover
wrench. The bounded headless end-to-end check runs the actual MuJoCo plant,
wrench controller, and allocator:

```bash
python3 src/minitrone_controller/test/hover_closed_loop.py
HOVER_HIGH_FIDELITY=true python3 src/minitrone_controller/test/hover_closed_loop.py
```

The checker starts the base at 1 m, commands its initial position and yaw,
and checks 8 s of closed-loop hover for finite state and commands, position,
velocity, attitude and rate convergence, bounded excursion, and actuator
limits. In the September 2026 runs, the final position error was about 2.6 mm
nominal and 2.6 mm with high fidelity enabled; allocator saturation was 0% in
both. The high-fidelity run had roughly 14 cm maximum startup excursion due to
its provisional actuator and disturbance model, so further tuning and broader
flight-envelope tests remain necessary.
