# Flight controller

`flight-controller.c` implements the application-level attitude controller used
by `main.c`. It combines accelerometer and gyroscope measurements with a
complementary filter, applies roll/pitch attitude PID and yaw-rate PID control,
and mixes the corrections into four quad-X motor outputs.

Motor order is front-left, front-right, rear-right, rear-left. Verify this
physical order, IMU axis orientation, motor rotation direction, PID gains, and
the open-loop hover throttle with propellers removed before flight.

The web controller maps:

- left X to yaw rate;
- left Y from zero through full collective throttle, with stick center at 50%;
- right X to desired roll angle;
- right Y to desired pitch angle.

The controller starts disarmed. The browser sends a heartbeat while armed; a
750 ms command timeout or an IMU read failure disarms the controller and drives
all motor outputs to minimum.

The Settings → PID tuning submenu exposes the Kp, Ki, and Kd gains for roll
attitude, pitch attitude, yaw-rate, and vertical-velocity control. Gains can
only be changed while disarmed and are persisted in NVS.

The vertical controller tilt-compensates the accelerometer Z axis, removes
gravity, and integrates the result with leakage to obtain a rudimentary
vertical-velocity estimate. Center throttle requests zero vertical velocity;
positions above and below center request climb and descent. At minimum stick,
vertical control is disabled but the armed-idle motor floor and attitude
corrections can remain active. Both the armed-idle output and minimum-throttle
stabilization are configurable in the PID tuning submenu. Disarming or a
failsafe always stops all motors.

An accelerometer does not directly measure altitude, and integration drift
cannot be eliminated this way. This mechanism can only provide short-term
vertical damping, not reliable altitude hold. Safe altitude hold needs an
additional barometer, range sensor, or other vertical-position input.
