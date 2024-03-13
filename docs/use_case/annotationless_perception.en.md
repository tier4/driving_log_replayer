# Evaluate Annotationless Perception

Evaluate Autoware's recognition features (perception) without annotations using the perception_online_evaluator.

Requires Autoware with the following PR features.
<https://github.com/autowarefoundation/autoware.universe/pull/6556>

## Evaluation method

The annotationless_perception evaluation is executed by launching the `annotationless_perception.launch.py` file.
Launching the file executes the following steps:

1. Execute launch of evaluation node (`annotationless_perception_evaluator_node`), `logging_simulator.launch` file and `ros2 bag play` command
2. Autoware receives sensor data output from input rosbag and the perception module performs recognition.
3. The perception_online_evaluator publishes diagnostic topic to `/diagnostic/perception_online_evaluator/metrics`
4. The evaluation node subscribes to the topic and evaluates data. The result is dumped into a file.
5. When the playback of the rosbag is finished, Autoware's launch is automatically terminated, and the evaluation is completed.

## Evaluation results

The results are calculated for each subscription. The format and available states are described below.

### Deviation Normal

The following two values specified in the scenario or launch argument are used to judge

- Threshold
  - Threshold for judging the success or failure of each item
- PassRange(Coefficient to correct threshold)
  - The range between `threshold * lower_limit` and `threshold * upper limit` is considered to pass the test.

Add the min, max, and mean values for each status.name in `/diagnostic/perception_online_evaluator/metrics` and calculate the average value.
If the `threshold * lower limit` <= `calculated_average` <= `threshold value * upper_limit`, it is assumed to be normal.

An illustration is shown below.

![metrics](./images/annotationless_metrics.drawio.svg)

### Deviation Error

When the deviation normal condition is not met

## Topic name and data type used by evaluation node

Subscribed topics:

| Topic name                                      | Data type                             |
| ----------------------------------------------- | ------------------------------------- |
| /diagnostic/perception_online_evaluator/metrics | diagnostic_msgs::msg::DiagnosticArray |

Published topics:

| Topic name | Data type |
| ---------- | --------- |
| N/A        | N/A       |

### Method of specifying conditions

The conditions can be given in two ways

#### Describe in scenario

```yaml
Evaluation:
  UseCaseName: annotationless_perception
  UseCaseFormatVersion: 0.1.0
  Conditions:
    # Threshold: {} # If Metrics are specified from result.jsonl of a previous test, the value here will be overwritten. If it is a dictionary type, it can be empty.
    Threshold:
      lateral_deviation: { min: 10.0, max: 10.0, mean: 10.0 }
      yaw_deviation: { min: 10.0, max: 10.0, mean: 10.0 }
      predicted_path_deviation_5.00: { min: 10.0, max: 10.0, mean: 10.0 }
      predicted_path_deviation_3.00: { min: 10.0, max: 10.0, mean: 10.0 }
      predicted_path_deviation_2.00: { min: 10.0, max: 10.0, mean: 10.0 }
      predicted_path_deviation_1.00: { min: 10.0, max: 10.0, mean: 10.0 }
    PassRange: 0.5-1.05 # lower[<=1.0]-upper[>=1.0] # The test will pass under the following `condition threshold * lower <= Σ deviation / len(deviation) <= threshold * upper`
```

#### Specify by launch argument

This method is assumed to be used mainly.
If the file path of result.jsonl output from a past test is specified, the metrics values from past tests can be used as threshold values.
The passing range can also be specified as an argument.

An image of its use is shown below.

![threshold](./images/annotationless_threshold.drawio.svg)

##### driving-log-replayer-cli

```shell
dlr simulation run -p annotationless_perception -l "annotationless_thresold_file:=${previous_test_result.jsonl_path},annotationless_pass_range:=${lower-upper}
```

##### WebAutoCLI

```shell
webauto ci scenario run --project-id ${project-id} --scenario-id ${scenario-id} --scenario-version-id ${scenario-version-id} --simulator-parameter-overrides annotationless_thresold_file=${previous_test_result.jsonl_path},annotaionless_pass_rate=${lower-upper}
```

##### Autoware Evaluator

Add to parameters in the simulator configuration in `.webauto-ci.yml`.

```yaml
simulations:
  - name: annotationless_perception
    type: annotationless_perception
    simulator:
      deployment:
        type: container
        artifact: main
      runtime:
        type: simulator/standard1/amd64/medium
      parameters:
        annotationless_threshold_file: ${previous_test_result.jsonl_path}
        annotationless_pass_range: ${upper-lower}
```

## Arguments passed to logging_simulator.launch

To make Autoware processing less resource-consuming, modules that are not relevant to evaluation are disabled by passing the `false` parameter as a launch argument.
The following parameters are set to `false`:

- perception: true
- planning: false
- control: false
- sensing: false / true (default false, set by launch argument)

### How to specify the sensing argument

#### driving-log-replayer-cli

```shell
dlr simulation run -p annotationless_perception -l "sensing:=true"
```

#### WebAutoCLI

```shell
webauto ci scenario run --project-id ${project-id} --scenario-id ${scenario-id} --scenario-version-id ${scenario-version-id} --simulator-parameter-overrides sensing=true
```

#### Autoware Evaluator

Add to parameters in the simulator configuration in `.webauto-ci.yml`.

```yaml
simulations:
  - name: annotationless_perception
    type: annotationless_perception
    simulator:
      deployment:
        type: container
        artifact: main
      runtime:
        type: simulator/standard1/amd64/medium
      parameters:
        sensing: "true"
```

## simulation

State the information required to run the simulation.

### Topic to be included in the input rosbag

| Topic name                             | Data type                                    |
| -------------------------------------- | -------------------------------------------- |
| /gsm8/from_can_bus                     | can_msgs/msg/Frame                           |
| /localization/kinematic_state          | nav_msgs/msg/Odometry                        |
| /sensing/gnss/ublox/fix_velocity       | geometry_msgs/msg/TwistWithCovarianceStamped |
| /sensing/gnss/ublox/nav_sat_fix        | sensor_msgs/msg/NavSatFix                    |
| /sensing/gnss/ublox/navpvt             | ublox_msgs/msg/NavPVT                        |
| /sensing/imu/tamagawa/imu_raw          | sensor_msgs/msg/Imu                          |
| /sensing/lidar/concatenated/pointcloud | sensor_msgs/msg/PointCloud2                  |
| /sensing/lidar/\*/velodyne_packets     | velodyne_msgs/VelodyneScan                   |
| /tf                                    | tf2_msgs/msg/TFMessage                       |

The vehicle topics can be included instead of CAN.

| Topic name                             | Data type                                           |
| -------------------------------------- | --------------------------------------------------- |
| /localization/kinematic_state          | nav_msgs/msg/Odometry                               |
| /sensing/gnss/ublox/fix_velocity       | geometry_msgs/msg/TwistWithCovarianceStamped        |
| /sensing/gnss/ublox/nav_sat_fix        | sensor_msgs/msg/NavSatFix                           |
| /sensing/gnss/ublox/navpvt             | ublox_msgs/msg/NavPVT                               |
| /sensing/imu/tamagawa/imu_raw          | sensor_msgs/msg/Imu                                 |
| /sensing/lidar/concatenated/pointcloud | sensor_msgs/msg/PointCloud2                         |
| /sensing/lidar/\*/velodyne_packets     | velodyne_msgs/VelodyneScan                          |
| /tf                                    | tf2_msgs/msg/TFMessage                              |
| /vehicle/status/control_mode           | autoware_auto_vehicle_msgs/msg/ControlModeReport    |
| /vehicle/status/gear_status            | autoware_auto_vehicle_msgs/msg/GearReport           |
| /vehicle/status/steering_status        | autoware_auto_vehicle_msgs/SteeringReport           |
| /vehicle/status/turn_indicators_status | autoware_auto_vehicle_msgs/msg/TurnIndicatorsReport |
| /vehicle/status/velocity_status        | autoware_auto_vehicle_msgs/msg/VelocityReport       |

### Topics that must not be included in the input rosbag

| Topic name | Data type               |
| ---------- | ----------------------- |
| /clock     | rosgraph_msgs/msg/Clock |

The clock is output by the --clock option of ros2 bag play, so if it is recorded in the bag itself, it is output twice, so it is not included in the bag.

## evaluation

State the information necessary for the evaluation.

### Scenario Format

See [sample](https://github.com/tier4/driving_log_replayer/blob/main/sample/annotationless_perception/scenario.yaml)

### Evaluation Result Format

See [sample](https://github.com/tier4/driving_log_replayer/blob/main/sample/annotationless_perception/result.json)

The format of each frame and the metrics format are shown below.
**NOTE: common part of the result file format, which has already been explained, is omitted.**

```json
{
  "Deviation": {
    "Result": { "Total": "Success or Fail", "Frame": "Success or Fail" }, // The results for Total and Frame are the same. The same values are output to make the data structure the same as other evaluations.
    "Info": {
      "lateral_deviation": {
        "min": "Minimum distance",
        "max": "Maximum distance",
        "mean": "Mean distance"
      },
      "yaw_deviation": {
        "min": "Minimum Angle Difference",
        "max": "Maximum Angle Difference",
        "mean": "Mean Angle Difference"
      },
      "predicted_path_deviation_5.00": {
        "min": "Minimum distance",
        "max": "Maximum distance",
        "mean": "Mean distance"
      },
      "predicted_path_deviation_3.00": {
        "min": "Minimum distance",
        "max": "Maximum distance",
        "mean": "Mean distance"
      },
      "predicted_path_deviation_2.00": {
        "min": "Minimum distance",
        "max": "Maximum distance",
        "mean": "Mean distance"
      },
      "predicted_path_deviation_1.00": {
        "min": "Minimum distance",
        "max": "Maximum distance",
        "mean": "Mean distance"
      }
    },
    "Metrics": {
      "lateral_deviation": {
        "min": "Average Minimum distance",
        "max": "Average Maximum distance",
        "mean": "Average Mean distance"
      },
      "yaw_deviation": {
        "min": "Average Minimum Angle Difference",
        "max": "Average Maximum Angle Difference",
        "mean": "Average Mean Angle Difference"
      },
      "predicted_path_deviation_5.00": {
        "min": "Average Minimum distance",
        "max": "Average Maximum distance",
        "mean": "Average Mean distance"
      },
      "predicted_path_deviation_3.00": {
        "min": "Average Minimum distance",
        "max": "Average Maximum distance",
        "mean": "Average Mean distance"
      },
      "predicted_path_deviation_2.00": {
        "min": "Average Minimum distance",
        "max": "Average Maximum distance",
        "mean": "Average Mean distance"
      },
      "predicted_path_deviation_1.00": {
        "min": "Average Minimum distance",
        "max": "Average Maximum distance",
        "mean": "Average Mean distance"
      }
    }
  }
}
```

See the figure below for the meaning of items

![lateral_deviation](./images/lateral_deviation.png)

![predicted_path_deviation](./images/predicted_path_deviation.png)
