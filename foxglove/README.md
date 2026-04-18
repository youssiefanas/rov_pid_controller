# Foxglove + rqt controls for `rov_pid_controller`

## Foxglove Studio

Import `rov_pid_controller.json` via **Layouts → Import from file**. It gives:

- Five preset buttons across the top (calls `/rov_pid_controller/set_all_modes`).
- `RawMessages` panel on `/rov_pid_controller/status` (current mode, setpoint,
  pose, effort per axis).
- `Parameters` panel for live gain tuning. The Parameters panel shows *all*
  node parameters; type `gains` into its filter box to isolate the PID gains
  (Foxglove doesn't expose a config-time filter at this time).
- `3D` panel rendering `/mimosa_node/graph/path` (the mimosa trajectory) and
  the live odometry pose, following `base_link`.
- Three `Plot` panels: pose vs setpoint, pose/velocity error, and effort.

Parameters now declare `floating_point_range` descriptors, so rqt_reconfigure
and Foxglove render sliders for all PID gains and limits.

For per-axis toggling, add a `Service Call` panel, point it at
`/rov_pid_controller/set_axis_mode`, and set the request to
`{"axis": 2, "mode": 2, "capture_current": true}` (example: heave hold).

Foxglove's layout schema changes between versions. If this layout fails to
import cleanly, import what it accepts, then re-save the arrangement you like.

## rqt

Out-of-the-box plugins already work with this package — no configuration needed:

```
rqt   # then: Plugins → Configuration → Dynamic Reconfigure   (edit gains live)
      #       Plugins → Services → Service Caller             (call set_axis_mode / set_all_modes)
      #       Plugins → Topics → Topic Monitor                (watch /rov_pid_controller/status)
      #       Plugins → Visualization → Plot                  (plot pose[i] vs setpoints[i])
```

## Qt panel (simplest)

For per-axis toggling with one click:

```
ros2 run rov_pid_controller axis_panel                 # defaults to /rov_pid_controller
ros2 run rov_pid_controller axis_panel --ns /my_node   # other namespace
```
