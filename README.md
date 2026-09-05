# Automatic Emergency Braking Safety Node

A ROS 2 node in C++ that halts a vehicle before collision, using time-to-collision
computed from 2D LiDAR and odometry. Built for the F1Tenth message interface.

## How it works

Distance alone is a poor braking criterion — two metres is fine at walking pace and
far too late at speed. This node uses **time-to-collision** instead.

Each LiDAR return is treated as a point obstacle. For a point at range `r` and bearing
`θ`, with the vehicle moving forward at speed `v`, the range shrinks at `v·cos(θ)`: a
beam pointing straight ahead closes at full speed, one pointing sideways closes at zero.

    closing_rate = speed * cos(beam_angle)
    ttc          = range / closing_rate

The minimum TTC across all beams is compared against a threshold. Below it, the node
publishes zero speed on `/drive`.

## Interface

| Direction | Topic | Type |
|---|---|---|
| Subscribe | `/scan` | `sensor_msgs/LaserScan` |
| Subscribe | `/ego_racecar/odom` | `nav_msgs/Odometry` |
| Publish | `/drive` | `ackermann_msgs/AckermannDriveStamped` |

Parameter: `ttc_threshold` (seconds, default 1.0). Read on every scan, so it can be
changed at runtime with `ros2 param set` as well as at launch.

## Design decisions

**Braking latches.** Once triggered, the node keeps braking until restarted rather than
releasing when TTC recovers. A production AEB releases, but it also has sensor fusion and
temporal filtering; without those, a single noisy beam could toggle the vehicle between
braking and accelerating. A system that oscillates is worse than one that is
conservatively stuck.

**Logic runs in the scan callback, not on a timer.** A 20 Hz timer would add up to 50 ms
between the scan showing an obstacle and the brake command — a quarter metre of travel at
5 m/s, for no benefit. The scan callback is the moment new evidence arrives.

**Three guard clauses.** :

- *Non-finite ranges are skipped.* Defensive rather than load-bearing. With `min_ttc`
  seeded to `+inf`, an `inf` range yields `ttc = inf` and loses every comparison, and a
  `NaN` fails `ttc < min_ttc` under IEEE semantics and is never stored. Neither can
  corrupt the minimum as the code stands. The guard documents intent and survives a
  future change to the seed.
- *Ranges outside `[range_min, range_max]` are skipped.* This one is load-bearing. Real
  LiDARs return near-zero values below `range_min`, not `inf`. A single 0.02 m reading on
  a forward beam gives `ttc ≈ 0.01 s`, and because braking latches, one bad sample ends
  the run. `NaN` also passes both window comparisons untouched, so the finite check
  above is what catches it.
- *Non-positive closing rates are skipped.* A stationary vehicle would divide by zero,
  and a beam pointing away would divide by a negative — producing a negative TTC, which
  compares as less than the threshold and fires a phantom brake.

## Testing

No simulator required. `test_publisher` synthesises a LaserScan and Odometry stream with
a point obstacle at a configurable `(x, y)` offset, closing at a configurable speed. Open
space is published as `inf`, per the `LaserScan` convention. Each tick logs the true TTC
as ground truth, derived by differentiating the range from the fixture's own kinematics
(`ṙ = -v·x/r`, so `ttc = r²/(v·x)`) rather than by reusing the node's projection — so the
two numbers are reached by independent routes.

    ros2 launch safety_node aeb.launch.py

### Verified cases

**On-axis, closing at 2 m/s from 10 m.** Computed min TTC matches the fixture's ground
truth at every tick to the printed precision (4.95/4.950 down to 0.95/0.950). Brake fires
on the first sample below threshold: 1.00 does not trigger, 0.95 does.

**Off-axis, obstacle 3 m to the side (`-p lateral_offset:=3.0`).** At x = 9.90 the
obstacle sits at range 10.34 m, bearing 16.86°. The node picks the nearest ray (17.00°)
and reports 5.41 s against a true 5.405 s. The on-axis case at the same x gives 4.95 s,
so the `cos(θ)` term is shifting the answer by 9% and is genuinely under test. Residual
error stays within beam quantization (≤0.3% over the run) and flips sign as the true
bearing crosses each ray, which is the signature of discretization rather than a formula
error.

**Stationary (`-p speed:=0.0`).** All beams report infinite TTC, no brake, for the whole
run. Confirms the closing-rate guard prevents division by zero.

**Threshold changed at runtime.** Launched at the default 1.0, then
`ros2 param set /safety_node ttc_threshold 2.0` mid-run. The brake fired on the next
scan, at 1.75 s — above the launch-time threshold, which is only possible if the
parameter is being re-read per scan.

### Not yet covered

- `ttc_threshold` set at launch time (the mechanism is shared with `lateral_offset`,
  which is verified, but the braking behaviour itself hasn't been observed).
- The odometry staleness check. The fixture publishes scan and odom from the same tick,
  so there is no way to stop one without stopping the other.


### A bug the first fixture could not catch

The original fixture modelled a flat perpendicular wall, with beams at an angle reporting
`distance / cos(angle)`. The tests passed cleanly. They passed for the wrong reason.

Working the geometry by hand: for a wall at perpendicular distance `d`, the reported
range of an off-axis beam is `d/cos(θ)` and shrinks at `v/cos(θ)`, because as the vehicle
advances the beam sweeps across a different point on the wall. Every beam reaches the
wall at the same instant, so the true TTC is `d/v` for all of them.

Run the node's point-obstacle formula against that fixture and the two errors do not
cancel — they compound:

    node ttc = (d/cos θ) / (v·cos θ) = d / (v·cos²θ)
    true ttc = d / v

The node overestimates by `sec²(θ)`. At θ = 60°, d = 10 m, v = 1 m/s, that is 40 s
against a true 10 s.

So why did the tests pass? Because `sec²(0) = 1`. The straight-ahead beam is exact under
both models, and since `sec²(θ) ≥ 1` everywhere, it is also the minimum. The assertion
was on the minimum, so it only ever looked at the one beam where a wall and a point are
indistinguishable. Every off-axis beam was wrong by a factor never checked.


## Known limitations

- **No mux.** The node publishes zero speed on `/drive`, but if a controller publishes to
  the same topic the messages interleave and last-write-wins. Real override needs a
  priority mux.
- **The brake is only asserted while scans arrive.** Detection and publication both live
  in the scan callback, so if the LiDAR stops, a latched brake silently stops being
  published. A watchdog timer that republishes the brake independently would fix this.
- **Field of view.** The formula generalizes to reverse without modification — `v < 0`
  and `cos(θ) < 0` on a rear beam multiply to a positive closing rate — but coverage
  does not. The fixture spans ±90°, and an F1Tenth Hokuyo covers 270°, so nothing
  directly behind the vehicle is seen.
- **No temporal filtering.** A single anomalous sample is enough to latch the brake.

## Building

    sudo apt install ros-jazzy-ackermann-msgs
    cd aeb_ws
    colcon build --symlink-install
    source install/setup.bash
    ros2 launch safety_node aeb.launch.py

