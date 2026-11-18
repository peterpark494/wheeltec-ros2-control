# FT-AG waypoint controller: first real-robot stage

This package implements the **kinematic waypoint outer loop** in equations
(10), (15), and (18) of Himanshu et al., *Fault tolerant self-reconfigurable
waypoint guidance for mobile robots under actuator faults* (2025). Each
waypoint is treated as the complete paper reference pose `(x_r, y_r, theta_r)`.
It reads `nav_msgs/Odometry` on `/odom` and publishes `geometry_msgs/Twist` on
`/cmd_vel`. The vendor's STM32 velocity loop remains in charge of the motors.

This stage does **not** implement the paper's four-wheel force allocation,
bounded QP in equations (20)-(22), or fault diagnosis/reconfiguration. The
heading gate is an explicit real-robot adaptation: the robot turns first when
the target lies far to its side or rear.

## Build on the robot

Place the package in `~/wheeltec_ros2/src/ftag_waypoint_controller` on the
robot. Run:

```bash
cd ~/wheeltec_ros2
source /opt/ros/humble/setup.bash
colcon build --packages-select ftag_waypoint_controller
source install/setup.bash
```

The same package can be built in a ROS 2 Humble development workspace, such
as `~/ros_car_dev` on the Ubuntu VM.

## First real-robot run

1. Start the vendor's chassis and odometry node as usual. Ensure `/odom`
   publishes fresh data and `/cmd_vel` is the correct command topic.
2. Keep the robot in an open area. Stop other nodes that publish `/cmd_vel`,
   such as teleoperation and Nav2 velocity controllers.
3. Start this controller in one terminal:

   ```bash
   source /opt/ros/humble/setup.bash
   source ~/wheeltec_ros2/install/setup.bash
   ros2 launch ftag_waypoint_controller controller.launch.py
   ```

   The default `enabled=false` does not send movement commands.
4. In another terminal, check the inputs and configured goals:

   ```bash
   ros2 topic hz /odom
   ros2 param get /ftag_waypoint_controller waypoints_x
   ros2 param get /ftag_waypoint_controller waypoints_y
   ros2 param get /ftag_waypoint_controller enabled
   ```

5. Enable the 0.5-metre forward test:

   ```bash
   ros2 param set /ftag_waypoint_controller enabled true
   ```

   To stop and reset the route:

   ```bash
   ros2 param set /ftag_waypoint_controller enabled false
   ```

`Ctrl+C` also stops the node and publishes a zero Twist while ROS is active.
   Stale odometry (>0.5 s by default) causes zero commands.

## Change the route

Edit `config/waypoints.yaml`, then rebuild and restart the node. `waypoints_x`
and `waypoints_y` must have the same length. With `relative_to_start: true`,
the route coordinates are in a frame fixed to the robot's pose when enabled:
`x` points forward, `y` points left. Example: `x: [0.5, 0.5]`,
`y: [0.0, 0.5]` first moves forward and then turns left. With
`relative_to_start: false`, the coordinates are in the `/odom` frame.

This is **waypoint tracking**, not image-based following of a painted line.
For map-drawn paths, the pose and goals must both use the `map` frame; that
requires a localization/TF version of this node before running map goals.

Control parameters are in `config/controller.yaml`; begin at the provided
low-speed limits. The default acceptance radius is 0.12 m. To compare with
another controller, use the same waypoint coordinates, starting pose, velocity
limits and localization method, and record `/odom` plus `/cmd_vel`.

## Odometry-relative straight-line experiment

`odom_line.launch.py` uses equations (10), (15), and (18) directly, but it
does not require a saved map or AMCL. When `enabled` changes to `true`, the
node captures the current `/odom` pose and creates a 3.6 m line in the current
forward direction. Every waypoint uses that captured direction as the fixed
reference heading `theta_r`. The velocity command at sample `k` uses the
stored errors `e1(k-1)` and `e3(k-1)`. Disabling and enabling again captures a
new start pose.

The paper's simplified kinematic law does not feed the lateral error `e2`
directly into either velocity command. Therefore a robot that is parallel to
the reference line can retain a lateral offset. This behavior is intentional
in this strict reproduction; there is no bearing-to-waypoint correction.

Place the robot centre on the physical line and align its heading, then run:

```bash
ros2 launch ftag_waypoint_controller odom_line.launch.py
```

After confirming `/odom` is fresh and no other node publishes `/cmd_vel`:

```bash
ros2 param set /odom_line_controller enabled true
```

Stop and reset with:

```bash
ros2 param set /odom_line_controller enabled false
```

## Front-left motor fault approximation

`odom_line_fault.launch.py` preserves the normal odometry line controller and
inserts one always-on velocity allocation path. The controller publishes its
unmodified command on `/cmd_vel_nominal`. The allocator applies equation (65),
`lambda_1=max(0.70, 1-0.04*t)`, and solves the bounded equivalent left/right
speed allocation before projecting effective wheel speeds to the `/cmd_vel`
accepted by the vendor chassis. The allocated pre-fault command is published
on `/cmd_vel_allocated`. This is a velocity-layer equivalent of bounded force
allocation; it cannot independently drive one physical wheel through the
STM32 Twist interface.

Launch with both motion and fault injection disabled:

```bash
ros2 launch ftag_waypoint_controller odom_line_fault.launch.py
```

Arm the fault first, then enable the line controller.  The fault timer starts
on the first nonzero nominal command:

```bash
ros2 param set /front_left_fault_injector fault_enabled true
ros2 param set /odom_line_controller enabled true
```

Stop and reset both parts after a run:

```bash
ros2 param set /odom_line_controller enabled false
ros2 param set /front_left_fault_injector fault_enabled false
```

Record `/odom`, `/cmd_vel_nominal`, `/cmd_vel_allocated`, `/cmd_vel`, and
`/fault/effectiveness` for
comparison with the healthy `odom_line.launch.py` run.  Only the injector may
publish `/cmd_vel` during this experiment.

## Fixed line in the saved map

`map_line.launch.py` runs the paper's kinematic outer loop on automatically
generated collinear waypoints in the `map` frame. It implements equation (10)
for body-frame pose error and equations (15) and (18) for desired linear and
angular velocity. The selected line is stored in `config/map_line.yaml`:

```text
start = (-0.05283617973327637,  0.005418464541143524)
end   = ( 1.9724102020263672,  -0.0378357470035553)
```

The line is divided into equally spaced pose waypoints internally; no RViz
point list is required. Every waypoint has the fixed line heading as
`theta_r`. Equations (15) and (18) use the errors saved from the previous
control sample. The resulting desired velocities are published as `/cmd_vel`,
while the vendor chassis performs the inner velocity control. This remains a
velocity-layer reproduction: the
paper's equations (20)-(22), wheel-force QP, ARR diagnosis, and force-level
reconfiguration are not implemented. Starting the node never moves the robot
because `enabled` defaults to false.

Intermediate waypoints use only the paper's 0.2 m radius of acceptance; the
previous pass-the-waypoint shortcut has been removed. Enabling is rejected unless the
robot is within 0.10 m of the start and within 0.20 rad of the line heading.

After map localization is running and visually verified, launch it with:

```bash
ros2 launch ftag_waypoint_controller map_line.launch.py
```

Before enabling, confirm that `map -> base_link` is updating, the robot is
within 0.30 m of the stored start point, its front faces approximately toward
the end, the full line is clear, and no teleoperation or Nav2 controller is
publishing motion commands. Then enable from another terminal:

```bash
ros2 param set /map_line_controller enabled true
```

Stop and reset the run with:

```bash
ros2 param set /map_line_controller enabled false
```

The controller stops at the end, when TF is missing/stale, when cross-track
error exceeds 0.40 m, or after 90 s. `cmd_vel_topic` is a parameter so a later
experiment can send the nominal command to a separate fault-injection node.
