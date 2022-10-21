# Evaluate self-localization estimation

Evaluate whether Autoware's self-location estimation (localization) is working stably.

In the evaluation of self-location estimation, the reliability and convergence of NDT are evaluated.

## Evaluation method

The launch file "localization.launch.py" is used for evaluation.
When the launch file is launched, the following is executed and evaluated.

1. launch evaluation node (localization_evaluator_node), logging_simulator.launch and ros2 bag play
2. autoware receives sensor data output from bag and performs self-location estimation
3. evaluation node subscribes topics, determines whether NDT reliability and convergence meet the criteria, and records the results in a file
4. when the playback of the bag is finished, launch is automatically terminated and the evaluation is completed.

### Reliability of NDT

Of the following two topics, the one specified in the scenario will be used for evaluation.

- /localization/pose_estimator/transform_probability
- /localization/pose_estimator/nearest_voxel_transformation_likelihood

### Convergence of NDT

Evaluate using the following

- /localization/pose_estimator/pose
- /localization/pose_twist_fusion_filter/pose

However, the convergence evaluation starts with /localization/pose_estimator/transform_probability > 0 or /localization/pose_estimator/nearest_voxel_transformation_likelihood > 0.

## Evaluation Result

For each subscription, the judgment result described below is output.

### Reliability Normal

If the data in /localization/pose_estimator/transform_probability or /localization/pose_estimator/nearest_voxel_transformation_likelihood is greater than or equal the AllowableLikelihood described in the scenario.

### Reliability Error

If the data in /localization/pose_estimator/transform_probability or /localization/pose_estimator/nearest_voxel_transformation_likelihood is less than the AllowableLikelihood described in the scenario.

### Convergence Normal

If all of the following three conditions are met

1. Calculate the lateral distance from /localization/pose_estimator/pose and /localization/pose_twist_fusion_filter/pose, and if the lateral distance is less than or equal to AllowableDistance described in the scenario
2. /localization/pose_estimator/exe_time_ms is less than or equal to AllowableExeTimeMs described in the scenario
3. /localization/pose_estimator/iteration_num is less than or equal to AllowableIterationNum described in the scenario

The lateral distance calculated in the step 1 is published as /driving_log_replayer/localization/lateral_distance.

### Convergence Error

When conditions for Convergence Normal are not met

## Topic name and data type used by evaluation node

- subscribe

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

- publish

| Topic name                                          | Data type                                     |
| --------------------------------------------------- | --------------------------------------------- |
| /driving_log_replayer/localization/lateral_distance | example_interfaces::msg::Float64              |
| /initialpose                                        | geometry_msgs::msg::PoseWithCovarianceStamped |

## Arguments passed to logging_simulator.launch

To lighten autoware processing, modules that are not relevant to evaluation are disabled by passing false as a launch argument.
The following is set.

- planning: false
- control: false

## About simulation

State the information required to run the simulation.

### Topic to be included in the input rosbag

The CAN of the vehicle's ECU and the topic of the sensor being used are required.
The following is an example, and may be changed if different sensors are used.

If multiple LiDARs are installed, include packets for all LiDARs installed.

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

### Result Format

Since localization evaluates both convergence and confidence, each line contains the result of either convergence or confidence.
The Result is true if both convergence and confidence pass, and false otherwise.

Examples of each evaluation are described below.
However, common parts that have already been explained in the result file format are omitted.

Convergence Result (when there is a Convergence item in the Frame)

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

Reliability Result (when there is a Reliability item in the Frame)

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
