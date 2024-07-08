# Evaluate Eagleye estimation

Evaluate whether Eagleye, a GNSS-IMU based map-less localization, is working stably.

## Evaluation method

The localization's evaluation is executed by launching the `eagleye.launch.py` file.
Launching the file executes the following steps:

1. Execute launch of evaluation node (`eagleye_evaluator_node`), `logging_simulator.launch` file and `ros2 bag play` command
2. Autoware receives sensor data input from prepared rosbag and performs localization estimation
3. Evaluation node subscribes to Autoware's output topics, determines whether the outputs meet the criteria, and outputs the results
4. When the playback of the rosbag is finished, Autoware's launch is automatically terminated, and the evaluation is completed.

### Availability of Eagleye

We use the output from `eagleye_monitor` via `/diagnostics` to evaluate whether Eagleye is available.

- `/diagnostics`

## Evaluation Result

The results are calculated for each subscription. The format and available states are described below.

### Eagleye Availability Normal

Information related to the monitored topic is extracted from `/diagnostics` which Component State Monitor outputs. If the most recent information is "OK", it is considered as pass.

### Eagleye Availability Error

The Eagleye availability evaluation output is marked as `Error` when conditions for `Eagleye Availability Normal` are not met.

## Topic name and data type used by evaluation node

Subscribed topics:

| Topic name   | Data type                             |
| ------------ | ------------------------------------- |
| /diagnostics | diagnostic_msgs::msg::DiagnosticArray |

Published topics:

| Topic name | Data type |
| ---------- | --------- |
| N/A        | N/A       |

## Service name and data type used by the evaluation node

| Service name                 | Data type              |
| ---------------------------- | ---------------------- |
| /api/localization/initialize | InitializeLocalization |

## Arguments passed to logging_simulator.launch

To make Autoware processing less resource-consuming, modules that are not relevant to evaluation are disabled by passing the `false` parameter as a launch argument.
The following parameters are set to `false`:

- perception: false
- planning: false
- control: false

## About simulation

State the information required to run the simulation.

### Topic to be included in the input rosbag

The following example shows the topic list available in evaluation input rosbag.

| Topic name                      | Data type                                |
| ------------------------------- | ---------------------------------------- |
| /sensing/gnss/ublox/nav_sat_fix | sensor_msgs/msg/NavSatFix                |
| /sensing/gnss/ublox/navpvt      | ublox_msgs/msg/NavPVT                    |
| /sensing/imu/tamagawa/imu_raw   | sensor_msgs/msg/Imu                      |
| /vehicle/status/velocity_status | autoware_vehicle_msgs/msg/VelocityReport |

### Topics that must NOT be included in the input rosbag

| Topic name | Data type               |
| ---------- | ----------------------- |
| /clock     | rosgraph_msgs/msg/Clock |

The clock is output by the --clock option of ros2 bag play, so if it is recorded in the bag itself, it is output twice, so it is not included in the bag.

## About Evaluation

State the information necessary for the evaluation.

### Scenario Format

See [sample](https://github.com/tier4/log_evaluator/blob/main/sample/eagleye/scenario.yaml).

### Evaluation Result Format

See [sample](https://github.com/tier4/log_evaluator/blob/main/sample/eagleye/result.json).

Examples of each evaluation are described below.
**NOTE: common part of the result file format, which has already been explained, is omitted.**

Availability Result example:

```json
{
  "Availability": {
    "Result": { "Total": "Success or Fail", "Frame": "Success, Fail, or Warn" },
    "Info": {}
  }
}
```
