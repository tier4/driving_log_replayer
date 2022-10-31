# Evaluate self-localization estimation

Evaluate whether Autoware's localization is working stably.

In the evaluation of localization, the reliability and convergence of NDT are evaluated.

## Evaluation method

The localization's evaluation is executed by launching the `localization.launch.py` file.
Launching the file executes the following steps:

1. Execute launch of evaluation node (`localization_evaluator_node`), `logging_simulator.launch` file and `ros2 bag play` command 
2. Autoware receives sensor data  input from previously prepared rosbag and performs localization estimation
3. Evaluation node subscribes to Autoware's output topics, determines whether NDT reliability and convergence meet the criteria, and dumps the results to a file
4. When the playback of the rosbag is finished, Autoware's launch is automatically terminated, and the evaluation is completed.

## Evaluation Result

The results are calculated for each subscription. The format and available states are described below.

### Reliability of NDT

Of the following two topics, the one specified in the scenario will be used for evaluation.

- `/localization/pose_estimator/transform_probability`
- `/localization/pose_estimator/nearest_voxel_transformation_likelihood`

### Convergence of NDT

Convergence evaluation is based on the following topics:

- `/localization/pose_estimator/pose`
- `/localization/pose_twist_fusion_filter/pose`

However, evaluation of convergence will be started after NDT convergence, and convergence is determined by /localization/pose_estimator/transform_probability > 0 or /localization/pose_estimator/nearest_voxel_transformation_likelihood > 0.

### Reliability Normal

If the data in `/localization/pose_estimator/transform_probability` or `/localization/pose_estimator/nearest_voxel_transformation_likelihood` is greater than or equal the `AllowableLikelihood` described in the scenario.

### Reliability Error

If the data in `/localization/pose_estimator/transform_probability` or `/localization/pose_estimator/nearest_voxel_transformation_likelihood` is less than the `AllowableLikelihood` described in the scenario.

### Convergence Normal

If all of the following conditions are met, the convergence is reported as Normal:

1. The lateral distance (calculated from `/localization/pose_estimator/pose` and `/localization/pose_twist_fusion_filter/pose` topics output)  is less than or equal to `AllowableDistance` described in the scenario
2. Execution time published to `/localization/pose_estimator/exe_time_ms` is less than or equal to `AllowableExeTimeMs` described in the scenario
3. Number of iterations published to `/localization/pose_estimator/iteration_num` is less than or equal to `AllowableIterationNum` described in the scenario

The lateral distance calculated in the step 1 is published as `/driving_log_replayer/localization/lateral_distance`.

### Convergence Error

The convergence evaluation output is marked as `Error` when conditions for ` Convergence Normal` are not met.

## Topic name and data type used by evaluation node

Subscribed topics:

| Topic name                                                           | Data type                             |
| -------------------------------------------------------------------- | ------------------------------------- |
| /localization/pose_estimator/transform_probability                   | tier4_debug_msgs::msg::Float32Stamped |
| /localization/pose_estimator/nearest_voxel_transformation_likelihood | tier4_debug_msgs::msg::Float32Stamped |
| /localization/pose_estimator/pose                                    | geometry_msgs::msg::PoseStamped       |
| /localization/kinematic_state                                        | nav_msgs::msg::Odometry               |
| /tf                                                                  | tf2_msgs/msg/TFMessage                |
| /localization/util/downsample/pointcloud                             | sensor_msgs::msg::PointCloud2         |
| /localization/pose_estimator/points_aligned                          | sensor_msgs::msg::PointCloud2         |
| /localization/pose_estimator/exe_time_ms                             | tier4_debug_msgs::msg::Float32Stamped |
| /localization/pose_estimator/iteration_num                           | tier4_debug_msgs::msg::Int32Stamped   |

Published topics:

| Topic name                                          | Data type                                     |
| --------------------------------------------------- | --------------------------------------------- |
| /driving_log_replayer/localization/lateral_distance | example_interfaces::msg::Float64              |
| /initialpose                                        | geometry_msgs::msg::PoseWithCovarianceStamped |

## Arguments passed to logging_simulator.launch

To make Autoware processing less resource-consuming, modules that are not relevant to evaluation are disabled by passing the `false` parameter as a launch argument.
The following parameters are set to `false` when launching the `localization` evaluation scenario:

- planning: false
- control: false

## About simulation

State the information required to run the simulation.

### Topic to be included in the input rosbag

The vehicle's ECU CAN and sensors data topics are required for the evaluation to be run correctly.
The following example shows the topic list available in evaluation input rosbag when multiple LiDARs are used in a real-world vehicle configuration.

- /sensing/gnss/ublox/fix_velocity
- /sensing/gnss/ublox/nav_sat_fix
- /sensing/gnss/ublox/navpvt
- /sensing/imu/tamagawa/imu_raw
- /sensing/lidar/\*/velodyne_packets
- /gsm8/from_can_bus

### Topics that must not be included in the input rosbag

- /clock

## About Evaluation

State the information necessary for the evaluation.

### Scenario Format

```yaml
Evaluation:
  UseCaseName: localization
  UseCaseFormatVersion: 1.2.0
  Conditions:
    Convergence:
      AllowableDistance: 0.2 # Lateral distance to be considered convergence
      AllowableExeTimeMs: 100.0 # If the NDT computation time is less than or equal to this value, it is considered successful.
      AllowableIterationNum: 30 # If the number of NDT calculations is less than or equal to this value, it is considered a success.
      PassRate: 95.0 # How much (%) of the evaluation attempts are considered successful.
    Reliability:
      Method: NVTL # NVTL or TP which method to use for evaluation
      AllowableLikelihood: 2.3 # If above this value, the localization reliability value is considered normal.
      NGCount: 10 # If the reliability value is lower than the threshold value for more than this number in the sequence. the evaluation is considered to have failed.
  InitialPose:
    position:
      x: 3836.5478515625
      y: 73729.96875
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.9689404241590215
      w: 0.2472942668776119
```

### Evaluation Result Format

Since localization evaluates both convergence and confidence of the localization, each line contains the result of either convergence or confidence.
The result is `true` if both convergence and confidence are evaluated correctly. Otherwise, the output is set to `false`.

Examples of each evaluation are described below.
*NOTE: common part of the result file format, which has already been explained, is omitted.*

Convergence Result example:

```json
{
  "Frame": {
    "Convergence": {
      "Result": "Success or Fail",
      "Info": [
        {
          "LateralDistance": "Lateral distance between ndt and ekf pose",
          "HorizontalDistance": "Horizontal distance between ndt and ekf. Reference value",
          "ExeTimeMs": "Time taken to calculate ndt",
          "IterationNum": "Number of recalculations of ndt"
        }
      ]
    }
  }
}
```

Reliability Result  example:

```json
{
  "Frame": {
    "Reliability": {
      "Result": "Success or Fail",
      "Info": [
        {
          "Value": "Value of NVTL or TP",
          "Reference": "Likelihood not used in the evaluation. Reference value; if Value is NVTL, TP is entered."
        }
      ]
    }
  }
}
```
