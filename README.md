# rov_pid_controller

Cascaded 6-DoF PID controller for the BlueROV2 Heavy, targeted at ROS 2 Jazzy.
Consumes a `nav_msgs/Odometry` state estimate, runs a per-axis *outer* pose
loop that feeds an *inner* velocity loop, and publishes either a
`geometry_msgs/Wrench` (for simulation) or a normalized `geometry_msgs/Twist`
(for `bluerov2_controller`/MAVROS on the real vehicle).

Each of the six axes (surge, sway, heave, roll, pitch, yaw) can independently
be **OFF**, **INNER_ONLY**, or **FULL**, so you can mix "hold depth + yaw,
joystick for the rest" without any separate modes/code paths.

## Architecture

```
           ┌─────────────────┐                       ┌──────────────┐
odometry ─▶│ outer PID (pose)│── velocity setpoint ─▶│ inner PID    │── effort ─▶
           └─────────────────┘                       │ (velocity)   │
           setpoint & measured pose                  └──────────────┘
                    ▲                                        ▲
                    │                                        │
              per-axis mode                          measured velocity
                                                     + feed-forward velocity
```

- **OFF** – the PID is bypassed; `cmd_vel_in` is passed through (normalized by
  `max_velocity` for the Twist path, scaled by `max_effort_norm` for wrench).
- **INNER_ONLY** – only the inner velocity loop runs, tracking `cmd_vel_in`.
- **FULL** – outer pose loop produces the velocity setpoint, inner loop tracks
  it. Angular axes wrap their error to (-π, π].

Key implementation details (in `src/`):

- Anti-windup on both loops via conditional integration.
- Derivative-on-measurement by default (prevents derivative kicks on setpoint
  changes).
- All gains and limits are dynamic parameters — changes apply on the next
  control tick without restart — and carry `floating_point_range` descriptors
  so rqt_reconfigure and Foxglove render them as sliders.

## Topics

| Direction | Topic (node-relative) | Type | Notes |
|-----------|-----------------------|------|-------|
| sub       | `odometry_topic` (param) | `nav_msgs/Odometry` | state estimate; body-frame twist |
| sub       | `cmd_vel_in`             | `geometry_msgs/Twist` | physical units (m/s, rad/s); joystick / teleop |
| pub       | `wrench`                 | `geometry_msgs/Wrench` | force/torque; enabled by `publish_wrench` |
| pub       | `cmd_vel`                | `geometry_msgs/Twist` | normalized to ±1; enabled by `publish_cmd_vel` |
| pub       | `status`                 | `rov_pid_controller/ControllerStatus` | modes, setpoints, pose, velocity, effort, errors @ 10 Hz |

The node runs under the namespace `/rov_pid_controller`, so in the default
launch the topics above live at `/rov_pid_controller/<name>`. Remappings in
the launch files point them at the real workspace topics.

## Services

All under `/rov_pid_controller/`:

- `set_axis_mode` (`SetAxisMode`) — set one axis's mode, optionally capturing
  the current pose as the setpoint.
- `set_all_modes` (`SetAllModes`) — atomically set all 6 axes (used by the
  Foxglove preset buttons and the Qt panel).
- `capture_setpoint` (`CaptureSetpoint`) — latch the current pose as the
  setpoint for one or more axes without changing modes.

### Setpoint handling

Pose setpoints are only used by the outer loop, which runs only in
`MODE_FULL`. The three ways to set one:

1. **Capture + hold atomically** — `set_axis_mode` (or `set_all_modes`) with
   `mode=FULL` and `capture_current=true`. The current measured pose is
   latched and the axis starts tracking it immediately. This is what the
   Qt Capture button and the Foxglove "All HOLD" preset do.
2. **Set an explicit value** — same call, `capture_current=false`, provide
   a finite number in `setpoint`.
3. **Pre-arm with capture, activate later** — call `capture_setpoint` now to
   latch the current pose (axis mode unchanged); later flip to FULL with
   `set_axis_mode` using `setpoint=NaN` and `capture_current=false`, which
   keeps the stored setpoint.

`NaN` in `setpoint`/`setpoints[i]` is the "keep what's stored" sentinel;
any finite value overwrites it.

## Parameters

See `config/controller.yaml` for a commented template. Groups:

- `control_rate` — PID tick rate (Hz). Default 50.
- `odometry_topic` — odometry source; default `/mimosa_node/imu/manager/odometry`.
- `publish_wrench`, `publish_cmd_vel` — select output channel(s).
- `gains.<axis>.outer.{kp,ki,kd,i_max}` — outer (pose) loop gains.
- `gains.<axis>.inner.{kp,ki,kd,i_max}` — inner (velocity) loop gains.
- `limits.<axis>.max_velocity` — outer-loop velocity clamp and joystick scale
  for OFF passthrough.
- `limits.<axis>.max_effort` — inner-loop force/torque clamp.
- `limits.<axis>.max_effort_norm` — full-scale effort mapped to ±1 on the
  Twist output.
- `mode.<axis>.default` — startup mode for this axis (0/1/2).

## Build

```
cd ~/jazzy_ws
colcon build --merge-install --packages-select rov_pid_controller
source install/setup.bash
```

## Run

### Simulation (OceanSim)

```
ros2 launch rov_pid_controller sim.launch.py
```

Publishes wrench to `/oceansim/robot/force_cmd`. In the OceanSim Sensor Example
UI, pick `ROS control` (or `ROS + Manual control`) and switch the control-mode
dropdown to **force control** — OceanSim's `ROS2ControlReceiver` then feeds
our wrench through its thruster allocator and hydrodynamics.

### Real BlueROV2 Heavy

```
ros2 launch rov_pid_controller real.launch.py
```

Publishes a normalized Twist to `/cmd_vel`, which `bluerov2_controller` maps
to per-DoF PWM via MAVROS. Joystick input is expected on `/cmd_vel_joy`.

## UIs

### Qt axis panel

```
ros2 run rov_pid_controller axis_panel
# or: ros2 run rov_pid_controller axis_panel --ns /my_node
```

Per-axis OFF/INNER/FULL radio buttons, setpoint spinboxes, Capture buttons,
and five preset buttons (All OFF, All HOLD, Depth+Yaw hold, Attitude hold,
Velocity cmd).

### Foxglove Studio

Import `foxglove/rov_pid_controller.json` via **Layouts → Import from file**.
See `foxglove/README.md` for what the layout contains and how to add
per-axis service calls.

### rqt

`rqt` out-of-the-box plugins work without configuration: Dynamic Reconfigure
for live slider tuning, Service Caller for mode switching, Topic Monitor for
`status`, and Plot for pose/effort traces.

## Typical workflow

1. Start the odometry source (mimosa) and either the simulator or the real
   vehicle bridge.
2. Launch `sim.launch.py` or `real.launch.py`.
3. Open the Foxglove layout or Qt panel.
4. Drive with joystick (all axes OFF) to verify the passthrough path.
5. Click **All HOLD (capture)** to latch the current pose and verify each
   axis regulates back to its setpoint.
6. Tune gains live via rqt_reconfigure / Foxglove sliders; values hot-reload
   on the next control tick.
