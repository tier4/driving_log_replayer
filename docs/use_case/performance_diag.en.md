# Evaluate diagnostics

Evaluate whether Autoware's diagnostics functionality behaves as expected.

The following subjects of evaluation are currently supported:

- visibility: function to determine if visibility is impaired by fog, rain, etc.
- blockage: function to determine if leaves or other objects are attached to LiDAR and obstructing the measurement.

## Evaluation Method

The diagnostics evaluation is executed by launching the `performance_diag.launch.py` file.
Launching the file executes the following steps:

1. Execute launch of evaluation node (`performance_diag_evaluator_node`), `logging_simulator.launch` file and `ros2 bag play` command.
2. Autoware receives sensor data output from the input rosbag and outputs the`/diagnostics_agg` topic.
3. The evaluation node subscribes to `/diagnostics_agg` topic, and evaluates data. The result is dumped into a file.
4. When the playback of the rosbag is finished, Autoware's launch is automatically terminated, and the evaluation is completed.

### visibility evaluation

The evaluation process confirms that more than a specified number of ERRORs for limited visibility are generated for rosbag input data obtained under rainy conditions (naturally or artificially generated).
Also, using data obtained during sunny weather, it is confirmed that ERRORs are never generated.

The `status.name` in `/diagnostics_agg` corresponding to `/autoware/sensing/lidar/performance_monitoring/visibility/\*` is used for judgment.

### blockage evaluation

For blockage evaluation, acquire data with LiDAR intentionally covered with a material that will not pass the laser beam (for example box). The evaluation confirms that ERROR for blockage is output more than a certain number of times for the considered situation.
The node will also confirm that no ERROR is generated for not covered LiDAR.

The `status.name` in `/diagnostics_agg` corresponding to `/autoware/sensing/lidar/performance_monitoring/blockage/*` is used for judgment.

## Evaluation Result

For each LiDAR diagnostic subscription, the evaluation judgment will be published on the topics described below:

Each output of the evaluation can be considered a success or a failure depending on what you want to evaluate. You can change this by describing the type in the scenario.

- If the scenario type is TP (true positive), it succeeds if the `Diag` state is ERROR.
- If the scenario type is FP (false positive), it succeeds if the `Diag` state is not ERROR.
- If the scenario type is null, the test is omitted.

### TP Normal

If the scenario type is TP and the level of visibility or blockage in the diagnostic information (`/diagnostics_agg`) is ERROR

### TP Error

If the scenario type is TP and the level of visibility or blockage in the diagnostic information (`/diagnostics_agg`) is not ERROR.

### FP Normal

If the scenario type is FP and the level of visibility or blockage in the diagnostic information (`/diagnostics_agg`) is not ERROR.

### FP Error

If the scenario type is FP and the level of visibility or blockage in the diagnostic information (`/diagnostics_agg`) is ERROR.

## Topic name and data type used by evaluation node

Subscribed topics:

| Topic name                                   | Data type                             |
| -------------------------------------------- | ------------------------------------- |
| /perception/obstacle_segmentation/pointcloud | sensor_msgs::msg::PointCloud2         |
| /diagnostics_agg                             | diagnostic_msgs::msg::DiagnosticArray |
| /tf                                          | tf2_msgs/msg/TFMessage                |

Published topics:

| Topic name                                               | Data type                                     |
| -------------------------------------------------------- | --------------------------------------------- |
| /driving_log_replayer/visibility/value                   | example_interfaces::msg::Float64              |
| /driving_log_replayer/visibility/level                   | example_interfaces::msg::Byte                 |
| /driving_log_replayer/blockage/{lidar_name}/ground/ratio | example_interfaces::msg::Float64              |
| /driving_log_replayer/blockage/{lidar_name}/sky/ratio    | example_interfaces::msg::Float64              |
| /driving_log_replayer/blockage/{lidar_name}/level        | example_interfaces::msg::Byte                 |
| /initialpose                                             | geometry_msgs::msg::PoseWithCovarianceStamped |

{lidar_name} contains the name of the mounted lidar.

## Arguments passed to logging_simulator.launch

To make Autoware processing less resource-consuming, modules that are not relevant to evaluation are disabled by passing the `false` parameter as a launch argument.
The following parameters are set to `false` when launching the `performance_diag` evaluation scenario:

- planning: false
- control: false
- localization: false / true (default value is false. Specify in scenario)

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
- /tf

**NOTE:If localization is false (false by default), /tf is required.**

### Topics that must not be included in the input rosbag

- /clock

## About Evaluation

State the information necessary for the evaluation.

### Scenario Format

```yaml
Evaluation:
  UseCaseName: performance_diag
  UseCaseFormatVersion: 1.0.0
  LaunchLocalization: false # If false, /tf in the bag is output; if true, /tf in the bag is remapped and disabled.
  InitialPose: null # Specifies the initial pose; only works when LaunchLocalization is enabled.
  Conditions:
    LiDAR:
      Visibility:
        PassFrameCount: 100 # When ScenarioType is TP, the Visibility test is considered successful if this number of ERRORs are generated; for FP, this condition is ignored because the test must not generate any ERRORs.
        ScenarioType: FP # TP/FP/null
      Blockage:
        front_lower: # Set for each on-board LiDAR
          ScenarioType: TP # TP/FP/null
          BlockageType: both # sky/ground/both Where is the blockage occurring?
          PassFrameCount: 100 # If ScenarioType is TP, Blockage type matches, and ERROR is generated at least this many times, the Blockage test is considered successful.  for FP, this condition is ignored because the test must not generate any ERRORs.
        front_upper:
          ScenarioType: TP
          BlockageType: both
          PassFrameCount: 100
        left_lower:
          ScenarioType: FP
          BlockageType: sky
          PassFrameCount: 30
        left_upper:
          ScenarioType: FP
          BlockageType: both
          PassFrameCount: 40
        rear_lower:
          ScenarioType: FP
          BlockageType: ground
          PassFrameCount: 50
        rear_upper:
          ScenarioType: FP
          BlockageType: sky
          PassFrameCount: 60
        right_lower:
          ScenarioType: FP
          BlockageType: both
          PassFrameCount: 70
        right_upper:
          ScenarioType: FP
          BlockageType: ground
          PassFrameCount: 80
```

### Evaluation Result Format

In `performance_diag` evaluation scenario visibility and blockage are evaluated.
The `Result` is `true` if both visibility and blockage evaluation steps have passed. Otherwise, the `Result` is `false`.

The result format is shown below.
**NOTE: common part of the result file format, which has already been explained, is omitted.**

```json
{
  "Frame": {
    "Visibility": [
      {
        "Result": "Success or Fail or Skipped",
        "Info": [
          {
            "Level": "Level of diag",
            "Visibility": "Value of visibility"
          }
        ]
      }
    ],
    "Blockage": [
      {
        "Result": "Success or Fail or Skipped",
        "Info": [
          {
            "Name": "Name of LiDAR",
            "Level": "Level of diag",
            "GroundBlockageRatio": "Ground blockage ratio",
            "GroundBlockageCount": "Ground blockage count. Reference",
            "SkyBlockageRatio": "Sky blockage ratio",
            "SkyBlockageCount": "Sky blockage count. Reference"
          }
        ]
      }
    ]
  }
}
```
