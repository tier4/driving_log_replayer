# Evaluate NDT estimation

Evaluate whether Autoware's localization by NDT is working stably.

In the evaluation of NDT, the reliability, convergence, and availability of NDT are evaluated.

## Evaluation method

Launching the file executes the following steps:

1. Execute launch of evaluation node (`localization_evaluator_node`), `logging_simulator.launch` file and `ros2 bag play` command
2. Autoware receives sensor data input from previously prepared rosbag and performs localization estimation
3. Evaluation node subscribes to Autoware's output topics, determines whether NDT reliability, convergence, and availability meet the criteria, and dumps the results to a file
4. When the playback of the rosbag is finished, Autoware's launch is automatically terminated, and the evaluation is completed.

### Reliability of NDT

Of the following two topics, the one specified in the scenario will be used for evaluation.

- `/localization/pose_estimator/transform_probability`
- `/localization/pose_estimator/nearest_voxel_transformation_likelihood`

### Convergence of NDT

Convergence evaluation is based on the following topics:

- `/localization/pose_estimator/initial_to_result_relative_pose`

### Availability of NDT

We also evaluate whether the output of `ndt_scan_matcher` is available. This is mainly useful for detecting some cases where NDT is not capable of publishing the appropriate outputs, for example due to:

- failure in `pointcloud_preprocessor` due to runtime error (which would result in unavailability of LiDAR scan inputs for `ndt_scan_matcher`)
- failure in `ndt_scan_matcher` due to runtime error

Here we evaluate whether the following output is being output regularly:

- `/localization/pose_estimator/exe_time_ms`

This is accomplished by indirectly using a package within Autoware called Component State Monitor. The evaluator subscribes the following topic for the information:

- `/diagnostics`

The reason why `/localization/pose_estimator/exe_time_ms` was chosen from the output topics of NDT is that it is possible to detect the aforementioned failure by confirming that messages are being output to the topic regularly.
For example, `/localization/pose_estimator/pose` is not suitable as a monitoring topic this time. This is because the topic may not output if the score of NVTL or TP is low, and it is difficult to isolate the cause to the failure by just monitoring the output.

## Evaluation Result

The results are calculated for each subscription. The format and available states are described below.

### Reliability Normal

If the data in `/localization/pose_estimator/transform_probability` or `/localization/pose_estimator/nearest_voxel_transformation_likelihood` is greater than or equal the `AllowableLikelihood` described in the scenario.

### Reliability Error

If the data in `/localization/pose_estimator/transform_probability` or `/localization/pose_estimator/nearest_voxel_transformation_likelihood` is less than the `AllowableLikelihood` described in the scenario.

### Convergence Normal

If all of the following conditions are met, the convergence is reported as Normal:

1. The lateral distance of `/localization/pose_estimator/initial_to_result_relative_pose` is less than or equal to `AllowableDistance` described in the scenario
2. Execution time published to `/localization/pose_estimator/exe_time_ms` is less than or equal to `AllowableExeTimeMs` described in the scenario
3. Number of iterations published to `/localization/pose_estimator/iteration_num` is less than or equal to `AllowableIterationNum` described in the scenario

The lateral distance obtained in the step 1 is published as `/log_evaluator/localization/lateral_distance`.

### Convergence Error

The convergence evaluation output is marked as `Error` when conditions for `Convergence Normal` are not met.

### NDT Availability Normal

Information related to the monitored topic is extracted from `/diagnostics` which Component State Monitor outputs. If the most recent information NOT Timeout nor NotReceived, it is considered as pass.

### NDT Availability Error

The NDT availability evaluation output is marked as `Error` when conditions for `NDT Availability Normal` are not met.

## Topic name and data type used by evaluation node

Subscribed topics:

| Topic name                                                           | Data type                             |
| -------------------------------------------------------------------- | ------------------------------------- |
| /diagnostics                                                         | diagnostic_msgs::msg::DiagnosticArray |
| /localization/pose_estimator/transform_probability                   | tier4_debug_msgs::msg::Float32Stamped |
| /localization/pose_estimator/nearest_voxel_transformation_likelihood | tier4_debug_msgs::msg::Float32Stamped |
| /localization/pose_estimator/initial_to_result_relative_pose         | geometry_msgs::msg::PoseStamped       |
| /localization/pose_estimator/exe_time_ms                             | tier4_debug_msgs::msg::Float32Stamped |
| /localization/pose_estimator/iteration_num                           | tier4_debug_msgs::msg::Int32Stamped   |
| /tf                                                                  | tf2_msgs/msg/TFMessage                |
| /localization/util/downsample/pointcloud                             | sensor_msgs::msg::PointCloud2         |
| /localization/pose_estimator/points_aligned                          | sensor_msgs::msg::PointCloud2         |

Published topics:

| Topic name                                   | Data type                        |
| -------------------------------------------- | -------------------------------- |
| /log_evaluator/localization/lateral_distance | example_interfaces::msg::Float64 |

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

The vehicle's ECU CAN and sensors data topics are required for the evaluation to be run correctly.
The following example shows the topic list available in evaluation input rosbag when multiple LiDARs are used in a real-world vehicle configuration.

| Topic name                         | Data type                                    |
| ---------------------------------- | -------------------------------------------- |
| /gsm8/from_can_bus                 | can_msgs/msg/Frame                           |
| /sensing/gnss/ublox/fix_velocity   | geometry_msgs/msg/TwistWithCovarianceStamped |
| /sensing/gnss/ublox/nav_sat_fix    | sensor_msgs/msg/NavSatFix                    |
| /sensing/gnss/ublox/navpvt         | ublox_msgs/msg/NavPVT                        |
| /sensing/imu/tamagawa/imu_raw      | sensor_msgs/msg/Imu                          |
| /sensing/lidar/\*/velodyne_packets | velodyne_msgs/VelodyneScan                   |

The vehicle topics can be included instead of CAN.

| Topic name                             | Data type                                      |
| -------------------------------------- | ---------------------------------------------- |
| /sensing/gnss/ublox/fix_velocity       | geometry_msgs/msg/TwistWithCovarianceStamped   |
| /sensing/gnss/ublox/nav_sat_fix        | sensor_msgs/msg/NavSatFix                      |
| /sensing/gnss/ublox/navpvt             | ublox_msgs/msg/NavPVT                          |
| /sensing/imu/tamagawa/imu_raw          | sensor_msgs/msg/Imu                            |
| /sensing/lidar/\*/velodyne_packets     | velodyne_msgs/VelodyneScan                     |
| /vehicle/status/control_mode           | autoware_vehicle_msgs/msg/ControlModeReport    |
| /vehicle/status/gear_status            | autoware_vehicle_msgs/msg/GearReport           |
| /vehicle/status/steering_status        | autoware_vehicle_msgs/SteeringReport           |
| /vehicle/status/turn_indicators_status | autoware_vehicle_msgs/msg/TurnIndicatorsReport |
| /vehicle/status/velocity_status        | autoware_vehicle_msgs/msg/VelocityReport       |

### Topics that must not be included in the input rosbag

| Topic name | Data type               |
| ---------- | ----------------------- |
| /clock     | rosgraph_msgs/msg/Clock |

The clock is output by the --clock option of ros2 bag play, so if it is recorded in the bag itself, it is output twice, so it is not included in the bag.

## About Evaluation

State the information necessary for the evaluation.

### Scenario Format

See [sample](https://github.com/tier4/log_evaluator/blob/main/sample/localization/scenario.yaml).

### Evaluation Result Format

See [sample](https://github.com/tier4/log_evaluator/blob/main/sample/localization/result.json).

Since localization evaluates convergence, reliability, and availability, each line contains the result of convergence, reliability, or availability.
The Result is `true` if convergence, reliability, and availability all pass, and `false` otherwise.

Examples of each evaluation are described below.
**NOTE: common part of the result file format, which has already been explained, is omitted.**

Convergence Result example:

```json
{
  "Convergence": {
    "Result": "Success or Fail",
    "Info": {
      "LateralDistance": "initial_to_result_relative_pose.pose.position.y",
      "HorizontalDistance": "Horizontal distance of initial_to_result_relative_pose.pose.position",
      "ExeTimeMs": "Time taken to calculate ndt",
      "IterationNum": "Number of recalculations of ndt"
    }
  }
}
```

Reliability Result example:

```json
{
  "Reliability": {
    "Result": "Success or Fail",
    "Info": {
      "Value": {
        "stamp": {
          "sec": "sec of stamp",
          "nanosec": "nanosec of stamp"
        },
        "data": "Value of NVTL or TP"
      },
      "Reference": {
        "stamp": {
          "sec": "sec of stamp",
          "nanosec": "nanosec of stamp"
        },
        "data": "Likelihood not used in the evaluation. Reference value; if Value is NVTL, TP is entered."
      }
    }
  }
}
```

Availability Result example:

```json
{
  "Availability": {
    "Result": { "Total": "Success or Fail", "Frame": "Success, Fail, or Warn" },
    "Info": {}
  }
}
```
