# Controller state snapshot, published at status_rate for UIs and diagnostics.
# Arrays ordered [surge, sway, heave, roll, pitch, yaw].

std_msgs/Header header

uint8[6]   modes          # 0=OFF, 1=INNER_ONLY, 2=FULL
float64[6] setpoints      # pose setpoint (meaningful only in FULL mode)
float64[6] pose           # measured pose (x,y,z, roll,pitch,yaw)
float64[6] velocity       # measured body-frame velocity
float64[6] effort         # last-published force/torque (N or Nm)
float64[6] pose_error     # wrapped (setpoint - pose); 0 when not in FULL mode
float64[6] velocity_error # (commanded vel - measured vel); 0 when mode is OFF
