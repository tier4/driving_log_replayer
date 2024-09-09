# Evaluate diagnostics

Evaluate whether Autoware's diagnostics functionality behaves as expected.

The following subjects of evaluation are currently supported:

- visibility: function to determine if visibility is impaired by fog, rain, etc.
- blockage: function to determine if leaves or other objects are attached to LiDAR and obstructing the measurement.

## Evaluation Method

The diagnostics evaluation is executed by launching the `performance_diag.launch.py` file.
Launching the file executes the following steps:

1. Execute launch of evaluation node (`performance_diag_evaluator_node`), `logging_simulator.launch` file and `ros2 bag play` command.
2. Autoware receives sensor data output from the input rosbag and outputs the`/diagnostics` topic.
3. The evaluation node subscribes to `/diagnostics` topic, and evaluates data. The result is dumped into a file.
4. When the playback of the rosbag is finished, Autoware's launch is automatically terminated, and the evaluation is completed.

### visibility evaluation

The evaluation process confirms that more than a specified rate of ERRORs for limited visibility are generated for rosbag input data obtained under rainy conditions (naturally or artificially generated).
Also, using data obtained during sunny weather, it is confirmed that ERRORs are never generated.

The `status.name` in `/diagnostics` corresponding to `dual_return_filter: /sensing/lidar/.*: visibility_validation` is used for judgment.

### blockage evaluation

For blockage evaluation, acquire data with LiDAR intentionally covered with a material that will not pass the laser beam (for example box). The evaluation confirms that ERROR for blockage is output more than a certain rate for the considered situation.
The node will also confirm that no ERROR is generated for not covered LiDAR.

The `status.name` in `/diagnostics` corresponding to `blockage_return_diag: /sensing/lidar/.*: blockage_validation` is used for judgment.

## Evaluation Result

For each LiDAR diagnostic subscription, the evaluation judgment will be published on the topics described below:

Each output of the evaluation can be considered a success or a failure depending on what you want to evaluate. You can change this by describing the type in the scenario.

- If the scenario type is TP (true positive), it succeeds if the `Diag` state is ERROR.
- If the scenario type is FP (false positive), it succeeds if the `Diag` state is not ERROR.
- If the scenario type is null, the test is omitted.

### TP Normal

If the scenario type is TP and the level in the diagnostic information is ERROR

### TP Error

If the scenario type is TP and the level in the diagnostic information is not ERROR.

### FP Normal

If the scenario type is FP and the level in the diagnostic information is not ERROR.

### FP Error

If the scenario type is FP and the level in the diagnostic information is ERROR.

## Topic name and data type used by evaluation node

Subscribed topics:

| Topic name                                   | Data type                             |
| -------------------------------------------- | ------------------------------------- |
| /perception/obstacle_segmentation/pointcloud | sensor_msgs::msg::PointCloud2         |
| /diagnostics                                 | diagnostic_msgs::msg::DiagnosticArray |
| /tf                                          | tf2_msgs/msg/TFMessage                |

Published topics:

| Topic name                                               | Data type                        |
| -------------------------------------------------------- | -------------------------------- |
| /driving_log_replayer/visibility/value                   | example_interfaces::msg::Float64 |
| /driving_log_replayer/visibility/level                   | example_interfaces::msg::Byte    |
| /driving_log_replayer/blockage/{lidar_name}/ground/ratio | example_interfaces::msg::Float64 |
| /driving_log_replayer/blockage/{lidar_name}/sky/ratio    | example_interfaces::msg::Float64 |
| /driving_log_replayer/blockage/{lidar_name}/level        | example_interfaces::msg::Byte    |

{lidar_name} contains the name of the mounted lidar.

## Service name and data type used by the evaluation node

| Service name                 | Data type              |
| ---------------------------- | ---------------------- |
| /api/localization/initialize | InitializeLocalization |

## Arguments passed to logging_simulator.launch

To make Autoware processing less resource-consuming, modules that are not relevant to evaluation are disabled by passing the `false` parameter as a launch argument.
The following parameters are set to `false`:

- planning: false
- control: false
- localization: false / true (default value is false. Specify in scenario)

## About simulation

State the information required to run the simulation.

### Topic to be included in the input rosbag

The vehicle's ECU CAN and sensors data topics are required for the evaluation to be run correctly.
The following example shows the topic list available in evaluation input rosbag when multiple LiDARs are used in a real-world vehicle configuration.

| Topic name                         | Data type                                    |
| ---------------------------------- | -------------------------------------------- |
| /pacmod/from_can_bus               | can_msgs/msg/Frame                           |
| /sensing/gnss/ublox/fix_velocity   | geometry_msgs/msg/TwistWithCovarianceStamped |
| /sensing/gnss/ublox/nav_sat_fix    | sensor_msgs/msg/NavSatFix                    |
| /sensing/gnss/ublox/navpvt         | ublox_msgs/msg/NavPVT                        |
| /sensing/imu/tamagawa/imu_raw      | sensor_msgs/msg/Imu                          |
| /sensing/lidar/\*/velodyne_packets | velodyne_msgs/VelodyneScan                   |
| /tf                                | tf2_msgs/msg/TFMessage                       |

The vehicle topics can be included instead of CAN.

| Topic name                             | Data type                                      |
| -------------------------------------- | ---------------------------------------------- |
| /sensing/gnss/ublox/fix_velocity       | geometry_msgs/msg/TwistWithCovarianceStamped   |
| /sensing/gnss/ublox/nav_sat_fix        | sensor_msgs/msg/NavSatFix                      |
| /sensing/gnss/ublox/navpvt             | ublox_msgs/msg/NavPVT                          |
| /sensing/imu/tamagawa/imu_raw          | sensor_msgs/msg/Imu                            |
| /sensing/lidar/\*/velodyne_packets     | velodyne_msgs/VelodyneScan                     |
| /tf                                    | tf2_msgs/msg/TFMessage                         |
| /vehicle/status/control_mode           | autoware_vehicle_msgs/msg/ControlModeReport    |
| /vehicle/status/gear_status            | autoware_vehicle_msgs/msg/GearReport           |
| /vehicle/status/steering_status        | autoware_vehicle_msgs/SteeringReport           |
| /vehicle/status/turn_indicators_status | autoware_vehicle_msgs/msg/TurnIndicatorsReport |
| /vehicle/status/velocity_status        | autoware_vehicle_msgs/msg/VelocityReport       |

**NOTE: If localization is false (false by default), /tf is required.**

### Topics that must not be included in the input rosbag

| Topic name | Data type               |
| ---------- | ----------------------- |
| /clock     | rosgraph_msgs/msg/Clock |

The clock is output by the --clock option of ros2 bag play, so if it is recorded in the bag itself, it is output twice, so it is not included in the bag.

## About Evaluation

State the information necessary for the evaluation.

### Scenario Format

See [sample](https://github.com/tier4/driving_log_replayer/blob/main/sample/performance_diag/scenario.yaml).

### Evaluation Result Format

See [sample](https://github.com/tier4/driving_log_replayer/blob/main/sample/performance_diag/result.json).

In `performance_diag` evaluation scenario visibility and blockage are evaluated.
The `Result` is `true` if both visibility and blockage evaluation steps have passed. Otherwise, the `Result` is `false`.

The result format is shown below.
**NOTE: common part of the result file format, which has already been explained, is omitted.**

Visibility Result example:

```json
{
  "Visibility": {
    "Result": { "Total": "Success or Fail", "Frame": "Success, Fail, or Invalid" },
    "Info": {
      "Level": "diagのレベル",
      "Visibility": "visibilityの値"
    }
  }
}
```

Blockage Result example:

```json
{
  "Blockage": {
    "name of LiDAR1": {
      "Result": { "Total": "Success or Fail", "Frame": "Success or Fail" },
      "Info": {
        "Level": "Level of diag",
        "GroundBlockageRatio": "Ground blockage ratio",
        "GroundBlockageCount": "Ground blockage count. Reference",
        "SkyBlockageRatio": "Sky blockage ratio",
        "SkyBlockageCount": "Sky blockage count. Reference"
      }
    },
    "name of LiDAR2": {
      "Result": { "Total": "Success or Fail", "Frame": "Success or Fail" },
      "Info": {
        "Level": "Level of diag",
        "GroundBlockageRatio": "Ground blockage ratio",
        "GroundBlockageCount": "Ground blockage count. Reference",
        "SkyBlockageRatio": "Sky blockage ratio",
        "SkyBlockageCount": "Sky blockage count. Reference"
      }
    }
  }
}
```
