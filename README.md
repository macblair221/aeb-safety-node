# Automatic Emergency Braking Safety Node

A ROS 2 node in C++ that halts a vehicle before collision, using time-to-collision
computed from 2D LiDAR and odometry. Built for the F1Tenth message interface.

## How it works

Distance alone is a poor braking criterion — two metres is fine at walking pace and
far too late at speed. This node uses **time-to-collision** instead.

For each LiDAR beam, the distance is closing at the vehicle's forward speed projected
onto that beam's direction: a beam pointing straight ahead closes at full speed, one
pointing sideways closes at zero.

    closing_rate = speed * cos(beam_angle)
    ttc          = range / closing_rate

The minimum TTC across all beams is compared against a threshold. Below it, the node
publishes zero speed on `/drive`, overriding whatever the controller wanted.

## Interface

| | Topic | Type |
|---|---|---|
| Subscribe | `/scan` | `sensor_msgs/LaserScan` |
| Subscribe | `/ego_racecar/odom` | `nav_msgs/Odometry` |
| Publish | `/drive` | `ackermann_msgs/AckermannDriveStamped` |

Parameter: `ttc_threshold` (seconds, default 1.0).

## Design decisions

**Braking latches.** Once triggered, the node keeps braking until restarted rather than
releasing when TTC recovers. A production AEB releases, but it also has sensor fusion and
temporal filtering; without those, a single noisy beam could toggle the vehicle between
braking and accelerating. A system that oscillates is worse than one that is
conservatively stuck.

**Logic runs in the scan callback, not on a timer.** A 20 Hz timer would add up to 50 ms
between the scan showing an obstacle and the brake command — a quarter metre of travel at
5 m/s, for no benefit. The scan callback is the moment new evidence arrives.

**Two guard clauses carry the correctness.** Non-finite ranges are skipped, since one
`inf` or `NaN` from a real LiDAR would poison the minimum. Beams with a non-positive
closing rate are skipped, since a stationary vehicle would divide by zero and a beam
pointing away would divide by a negative — producing a small *negative* TTC that compares
as less than the threshold and fires a phantom brake.

## Testing

No simulator required. `test_publisher` synthesises a LaserScan and Odometry stream with
a flat wall closing at a configurable speed, and logs the true TTC each tick as ground
truth.

The wall is modelled as perpendicular to the vehicle, so beams at an angle report a
longer range (`distance / cos(angle)`). This matters: an implementation that ignored beam
angle would compute wrong TTCs for the side beams, and this fixture catches it.

    ros2 launch safety_node aeb.launch.py

Verified cases:

- **Closing at 2 m/s from 10 m** — computed min TTC matches the fixture's true TTC at
  every tick; brake fires on the first sample below threshold.
- **Stationary (`-p speed:=0.0`)** — all beams report infinite TTC, no brake. Confirms
  the closing-rate guard prevents division by zero.

## Building

    sudo apt install ros-jazzy-ackermann-msgs
    cd aeb_ws
    colcon build
    source install/setup.bash
    ros2 launch safety_node aeb.launch.py