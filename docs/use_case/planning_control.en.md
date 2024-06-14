# Evaluate Planning Control

Evaluate whether Planning / Control metrics are output at specified times and conditions

## Evaluation Method

The planning_control evaluation is executed by launching the `planning_control.launch.py` file.
Launching the file executes the following steps:

1. Execute launch of evaluation node (`planning_control_evaluator_node`), `logging_simulator.launch` file and `ros2 bag play` command
2. Autoware receives sensor data output from input rosbag and the perception module performs recognition.
3. Using the results of perception, Autoware output Metrics to `/diagnostic/planning_evaluator/metrics(temporary)` for planning and `/diagnostic/control_evaluator/metrics` for control.
4. The evaluation node subscribes to the topic and evaluates data. The result is dumped into a file.
5. When the playback of the rosbag is finished, Autoware's launch is automatically terminated, and the evaluation is completed.

## Evaluation Result

The output from control_evaluator is in the form of the following sample TOPIC.
[control sample](https://github.com/tier4/driving_log_replayer/blob/main/sample/planning_control/control_diag.txt)

If there is a condition to be evaluated at the time of the header, it is evaluated and the judgment result is output.
If there is no evaluation condition corresponding to the time of the header, it is discarded and no log is output.

### Normal

If more than (current time - evaluation start time)\*Hertz\*AllowableRate(=0.95) topics are obtained that satisfy the evaluation conditions.

In the following example, if the current time is 2, starting from 1 second, at the 2 second mark, (2-1)*10.0=10 topics should be retrieved.
However, the output rate is not exactly the specified rate.
Therefore, if the AllowableRate is applied and floor(10*0.95)=9 or more autonomous_emergency_braking: aeb_emergency_stop decision value stop is obtained, it is considered a success.
At 3 seconds, floor(20\*0.95)=19 or more is success.

```yaml
Conditions:
  Hertz: 10.0 # How many Hz the METRICS will come in. (current time - evaluation start time)* Hertz * AllowableRate(=0.95) or more TOPICS that match the condition must be output. AllowableRate is currently fixed.
  ControlConditions:
    - TimeRange: { start: 1, end: 3 } # Evaluation start time and end time, end is optional and if omitted, sys.float_info.max
      Module: "autonomous_emergency_braking: aeb_emergency_stop" # Modules to be evaluated
      Value0Key: decision # Key to be evaluated
      Value0Value: stop # Value to be evaluated
      DetailedConditions: null # Additional conditions to be determined, such as position, speed, etc. If null, succeeds when Value0Value is matched. If not null, DetailedConditions must also satisfy the condition.
```

### Error

When the normal condition is not met

## Topic name and data type used by evaluation node

Subscribed topics:

| Topic name                                | Data type                             |
| ----------------------------------------- | ------------------------------------- |
| /diagnostic/control_evaluator/metrics        | diagnostic_msgs::msg::DiagnosticArray |
| /diagnostic/planning_evaluator/metrics (仮) | diagnostic_msgs::msg::DiagnosticArray |

Published topics:

| Topic name | Data type |
| ---------- | --------- |
| N/A        | N/A       |

## Arguments passed to logging_simulator.launch

To make Autoware processing less resource-consuming, modules that are not relevant to evaluation are disabled by passing the `false` parameter as a launch argument.

- sensing: true / false (Specified by the launch argument. Currently fixed to true.)
- localization: false

## About simulation

State the information required to run the simulation.

### Topic to be included in the input rosbag

| Topic name                         | Data type                                    |
| -------------------------------------- | -------------------------------------------- |
| /gsm8/from_can_bus                     | can_msgs/msg/Frame                           |
| /localization/kinematic_state          | nav_msgs/msg/Odometry                        |
| /localization/acceleration             | geometry_msgs/msg/AccelWithCovarianceStamped |
| /sensing/lidar/concatenated/pointcloud | sensor_msgs/msg/PointCloud2                  |
| /tf                                    | tf2_msgs/msg/TFMessage                       |
| /planning/mission_planning/route       | autoware_planning_msgs/msg/LaneletRoute      |

The vehicle topics can be included instead of CAN.

| Topic name                         | Data type                                    |
| -------------------------------------- | --------------------------------------------------- |
| /localization/kinematic_state          | nav_msgs/msg/Odometry                               |
| /localization/acceleration             | geometry_msgs/msg/AccelWithCovarianceStamped        |
| /sensing/lidar/concatenated/pointcloud | sensor_msgs/msg/PointCloud2                         |
| /tf                                    | tf2_msgs/msg/TFMessage                              |
| /planning/mission_planning/route       | autoware_planning_msgs/msg/LaneletRoute             |
| /vehicle/status/control_mode           | autoware_auto_vehicle_msgs/msg/ControlModeReport    |
| /vehicle/status/gear_status            | autoware_auto_vehicle_msgs/msg/GearReport           |
| /vehicle/status/steering_status        | autoware_auto_vehicle_msgs/SteeringReport           |
| /vehicle/status/turn_indicators_status | autoware_auto_vehicle_msgs/msg/TurnIndicatorsReport |
| /vehicle/status/velocity_status        | autoware_auto_vehicle_msgs/msg/VelocityReport       |

### Topics that must not be included in the input rosbag

| Topic name                         | Data type                                    |
| -------------------------------------- | -------------------------------------------------
| -------- | ----------------------- |
| /clock   | rosgraph_msgs/msg/Clock |

The clock is output by the --clock option of ros2 bag play, so if it is recorded in the bag itself, it is output twice, so it is not included in the bag.

## About Evaluation

State the information necessary for the evaluation.

### Scenario Format

See [sample](https://github.com/tier4/driving_log_replayer/blob/main/sample/planning_control/scenario.ja.yaml).

### Evaluation Result Format

See [sample](https://github.com/tier4/driving_log_replayer/blob/main/sample/planning_control/result.json).

The result format is shown below.
**NOTE: common part of the result file format, which has already been explained, is omitted.**

Success is determined when all evaluation conditions set in planning and control are met.

```json
{
  "Frame": {
    "CONDITION_INDEX": {
      // Results are output for each evaluation condition.
      "Result": { "Total": "Success or Fail", "Frame": "Success or Fail" },
      "Info": {
        "TotalPassed": "Total number of topics that passed the evaluation criteria",
        "RequiredSuccess": "Number of successes required at current time (TotalPassed >= RequiredSuccess makes Total a success)"
      }
    }
  }
}
```
