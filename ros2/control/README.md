# CobraFlex Test Control

`cobraflex_test_control.py` is the GUI-based ROS 2 test controller used to execute repeatable CobraFlex motion profiles and record rosbag2 data for simulation and physical-robot experiments.

The controller publishes `geometry_msgs/msg/Twist` commands on `/cmd_vel`, displays live telemetry, records selected ROS 2 topics, and supports ten predefined test profiles.

## Run

Use a sourced ROS 2 environment with the required message packages, PyQt5, and Matplotlib:

```bash
python3 ros2/control/cobraflex_test_control.py
```

## GUI configuration parameters

| Parameter | Meaning | Used by |
| --- | --- | --- |
| **Test Profile** | Selects one of the ten predefined command sequences described below. | All tests |
| **Target Linear Vel (m/s)** | Requested longitudinal velocity `V` sent through `Twist.linear.x`. | Tests 1, 2, 3, 5, 6, 7, 8, 9 |
| **Target Angular Vel (rad/s)** | Requested yaw rate `W` sent through `Twist.angular.z`. | Tests 3, 4, 6, 7 |
| **Profile Duration (s)** | User-adjustable duration of the main command phase. Its exact meaning depends on the selected profile. Test 6 does not use this field. | Tests 1, 2, 3, 4, 5, 7, 8, 9, 10 |
| **Square Side Length (m)** | Requested straight-line distance for each UMBmark/square edge. The controller converts this to straight-driving time using `side_length / V`. | Test 6 |
| **Turn Factor** | Multiplier applied to the ideal 90-degree turn time, `(pi/2)/W`. A value of 1.0 uses the nominal kinematic turn time. | Test 6 |
| **Pulse Cycles** | Number of forward/reverse excitation cycles. One cycle contains one `+V` phase and one `-V` phase. | Test 9 |
| **Repeats** | Number of independent settled trials. Each repeat receives its own rosbag2 directory. | Tests 1, 2, 3, 4, 5, 8, 10 |
| **Software Version** | Free-text tag appended to the generated bag name for experiment traceability. It does not change controller behavior. | All tests |
| **Profile Timing Source** | Selects how profile durations are measured: automatic, simulation `/clock`, or monotonic real time. | All tests |
| **Telemetry Stream Logging Options** | Selects which ROS 2 topics are recorded. Core topics are checked by default; secondary topics are optional. | All tests |

### Timing-source options

- **Auto**: use an advancing ROS `/clock` when available; otherwise use monotonic time.
- **Simulation**: require an advancing ROS `/clock`. This is the intended mode for Isaac Sim.
- **Real robot**: use monotonic time and do not require `/clock`.

The selected timing source affects all profile durations, including settle timing and auxiliary waits that use the profile clock.

## Test profiles

The table below documents the command sequence implemented by the controller. `T` is the user-entered **Profile Duration**.

| Test | Profile | Implemented command sequence | Time control |
| ---: | --- | --- | --- |
| **1** | Acceleration Test | `(0,0) -> (V,0)` | `(V,0)` is held for user-adjustable `T`. |
| **2** | Full Braking Testing | `(0,0) -> (V,0) -> (0,0)` | Drive phase uses user-adjustable `T`; the zero-command braking observation is currently fixed at 3.0 s in the source. |
| **3** | Steady-State Circular Driving | `(0,0) -> (V,W)` | Combined linear/angular command is held for user-adjustable `T`. |
| **4** | In-place Skid-Steer Test | `(0,0) -> (0,W)` | In-place command is held for user-adjustable `T`. |
| **5** | Steady-State Max Velocity Test | `(0,0) -> (V,0)` | Straight command is held for user-adjustable `T`. The command sequence is intentionally similar to Test 1; the experimental purpose and KPI interpretation differ. |
| **6** | Square Trajectory / UMBmark | Four straight segments with an in-place turn after the first three segments | Straight time is `side_length / V`; turn time is `((pi/2)/W) * turn_factor`. The **Profile Duration** field is disabled for this test. |
| **7** | Step Steer Input | `(V,0)` baseline -> `(V,W)` step | Straight baseline is currently fixed at 3.0 s; the step phase uses user-adjustable `T`. |
| **8** | Coasting Testing | `(0,0) -> (V,0) -> (0,0)`, then observation | Driven phase uses user-adjustable `T`; zero-command observation is currently fixed at 6.0 s. The script sends a zero velocity command; it does not explicitly command zero motor torque. |
| **9** | Weight Transfer / Pitch Test | Repeated `(+V,0) -> (-V,0)` pulses | Each forward phase lasts user-adjustable `T`, and each reverse phase also lasts `T`. Number of cycles is set by **Pulse Cycles**. |
| **10** | Baseline Noise Floor Test | `(0,0)` throughout | Stationary observation lasts user-adjustable `T`. |

## Which times are adjustable?

The main test duration is adjustable from the GUI through **Profile Duration (s)** for every test except Test 6.

Several supporting timings are intentionally separate from the main profile duration and are currently source-level constants or fixed values rather than GUI fields:

| Timing | Current value | Adjustable from GUI? | Purpose |
| --- | ---: | --- | --- |
| Sequence zero-command pre-buffer | 2.0 s | No | Initial idle period before the selected test logic begins |
| Per-repeat recorded zero-command pre-roll | 2.0 s | No | Captures a clean baseline immediately before each repeated trial |
| Test 2 braking observation | 3.0 s | No | Records the response after commanding `(0,0)` |
| Test 7 straight baseline | 3.0 s | No | Establishes forward motion before the yaw-rate step |
| Test 8 zero-command observation | 6.0 s | No | Records the deceleration/response after the non-zero command |
| Settle confirmation window | 0.3 s | No | Requires low velocity to persist before accepting the vehicle as settled |
| Settle timeout | 5.0 s | No | Maximum wait before proceeding after a settle attempt |
| Final sequence zero-command hold | 1.0 s | No | Final stop command during sequence cleanup |

These values can be changed in the Python source. They are not currently exposed as GUI parameters.

The recorder-discovery and `/clock` watchdog timeouts are also source-level safety/robustness settings rather than experiment-profile parameters.

## Repeat and recording logic

Tests 1, 2, 3, 4, 5, 8, and 10 use the shared repeated-trial workflow:

1. Command zero velocity and wait for the chassis to settle.
2. Require approximately `|V| < 0.02 m/s` and `|W| < 0.02 rad/s`.
3. Require the settled state to persist for 0.3 s, with a 5.0 s settle timeout.
4. Start a dedicated rosbag2 recording for that repeat.
5. Record a 2.0 s zero-command pre-roll.
6. Execute the selected test profile.
7. Stop and validate the rosbag2 recording.
8. Repeat from a settled state when additional repeats are requested.

Generated repeat bags use the pattern:

```text
<base_name>_rep1ofN
<base_name>_rep2ofN
...
```

Tests 6, 7, and 9 use a single continuous profile rather than the shared settled-repeat workflow.

## Recording behavior

For simulation-timed runs, `/clock` and `/odom_truth` are added to the recording set automatically. For real-robot timing, those simulation-only topics are removed.

The controller waits for rosbag2 subscriptions to appear before the recorded pre-roll begins. After recording, it checks the selected topic set and reports any streams that contain zero messages.

Raw rosbag2 recordings are intentionally not stored in the GitHub handover repository.

## Emergency stop

**Emergency Stop** immediately:

- marks the active test as interrupted;
- publishes `(0,0)` on `/cmd_vel`;
- allows the worker thread to complete normal recorder cleanup and file flushing.

The GUI thread does not terminate the recorder process directly, avoiding a race with the recording lifecycle thread.

## Protocol notes

- **Test 1 and Test 5 use the same basic straight-line command profile.** They are separated because their intended experimental interpretation differs.
- **Test 6 currently performs four straight segments and three in-place turns.** It does not execute a final fourth turn after the last edge.
- **Test 8 sends a zero velocity command rather than explicitly disabling motor torque.** Its result should therefore be interpreted as a zero-command deceleration observation unless the downstream drive implementation is independently verified to coast freely.
- The GUI exposes the main profile duration, but not every auxiliary timing constant. This distinction is deliberate in this handover documentation so that the recorded protocol can be reproduced exactly.