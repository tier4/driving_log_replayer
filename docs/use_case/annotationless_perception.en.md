# Evaluate Annotationless Perception

Evaluate Autoware's recognition features (perception) without annotations using the perception_online_evaluator.

Requires Autoware with the following PR features.
<https://github.com/autowarefoundation/autoware.universe/pull/6556>

## Evaluation method

Launching the file executes the following steps:

1. Execute launch of evaluation node (`annotationless_perception_evaluator_node`), `logging_simulator.launch` file and `ros2 bag play` command
2. Autoware receives sensor data output from input rosbag and the perception module performs recognition.
3. The perception_online_evaluator publishes diagnostic topic to `/diagnostic/perception_online_evaluator/metrics`
4. The evaluation node subscribes to the topic and evaluates data. The result is dumped into a file.
5. When the playback of the rosbag is finished, Autoware's launch is automatically terminated, and the evaluation is completed.

## Evaluation results

The output topic of perception_online_evaluator is in the form of the following sample.
[topic sample](https://github.com/tier4/log_evaluator/blob/main/sample/annotationless_perception/diag_topic.txt)

For each subscription, the following judgment results are output for each recognition class.

If all classes are normal, the test is successful.

### Normal

The following two values specified in the scenario or launch argument are used to judge

- Threshold
- PassRange(Coefficient to correct threshold)

Success or failure is determined for each status.name in `/diagnostic/perception_online_evaluator/metrics` according to the following rules.
Items for which no threshold is set (min, max, mean) are always judged as normal. Only those items for which a threshold is specified are subject to evaluation.

#### min

If `threshold * lower_limit` <= `minimum value of min` <= `threshold * upper_limit`, it is assumed to be normal.

#### max

If `threshold * lower_limit` <= `maximum value of max` <= `threshold * upper_limit`, it is assumed to be normal.

Lower limit recommended to be 0.0

#### mean

If `threshold * lower_limit` <= `average value of mean` <= `threshold * upper_limit`, it is assumed to be normal.

#### metric_value

If `threshold * lower_limit` <= `value of metric_value` <= `threshold * upper_limit`, it is assumed to be normal.
metric_value is determined by the current topic value only and does not update the values of min, max, and mean metrics.

An illustration is shown below.

![metrics](./images/annotationless_metrics.drawio.svg)

### Error

When the normal condition is not met

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
  UseCaseFormatVersion: 0.3.0
  Conditions:
    ClassConditions:
      # Describe the conditions for each class. If a class with no conditions is output, only the metrics are calculated. It does not affect the evaluation.
      # In the sample data, the class of TRUCK is also output, but the condition is not described, so TRUCK is always Success.
      # When specifying conditions from result.jsonl, only keys described here will be updated.
      # Even though TRUCK metrics appear in result.jsonl, they are not added to the evaluation condition because the TRUCK key is not specified in this example.
      CAR: # classification key
        Threshold:
          # Keys not described will not be evaluated (will always be a success)
          lateral_deviation: { max: 0.4, mean: 0.019 }
          yaw_deviation: { max: 3.1411, mean: 0.05 }
          predicted_path_deviation_5.00: { max: 16.464, mean: 1.8 }
          total_objects_count_r60.00_h10.00: { metric_value: 10 }
        PassRange:
          min: 0.0-2.0 # lower[<=1.0]-upper[>=1.0]
          max: 0.0-2.0 # lower[<=1.0]-upper[>=1.0]
          mean: 0.5-2.0 # lower[<=1.0]-upper[>=1.0]
          metric_value: 0.9-1.1
      BUS: # classification key
        Threshold:
          # Only lateral_deviation is evaluated.
          yaw_rate: { max: 0.05 } # Only max is evaluated.
        PassRange:
          min: 0.0-2.0 # lower[<=1.0]-upper[>=1.0]
          max: 0.0-2.0 # lower[<=1.0]-upper[>=1.0]
          mean: 0.5-2.0 # lower[<=1.0]-upper[>=1.0]
          metric_value: 0.9-1.1
```

#### Specify by launch argument

This method is assumed to be used mainly.

If the file path of result.jsonl output from past tests is specified, the metrics values from past tests are used as threshold values.
The values are updated from result.jsonl only for the thresholds listed in the scenario.

The passing range can also be specified as an argument.

An image of its use is shown below.

![threshold](./images/annotationless_threshold.drawio.svg)

##### driving-log-replayer-cli

```shell
dlr simulation run -p annotationless_perception -l annotationless_threshold_file:=${previous_test_result.jsonl_path} -l 'annotationless_pass_range:={"KEY1":VALUE1"[,"KEY2":"VALUE2"...]}'

# example
dlr simulation run -p annotationless_perception -l annotationless_threshold_file:=$HOME/out/annotationless/2024-0314-155106/sample/result.jsonl -l 'annotationless_pass_range:={"CAR":{"min":"0.0-1.1","max":"0.0-1.2","mean":"0.5-1.3"},"BUS":{"min":"0.0-1.1","max":"0.0-1.2","mean":"0.5-1.3"}}'
```

##### WebAutoCLI

```shell
webauto ci scenario run --project-id ${project-id} --scenario-id ${scenario-id} --scenario-version-id ${scenario-version-id} --simulator-parameter-overrides 'annotationless_threshold_file=${previous_test_result.jsonl_path},annotationless_pass_range:={"KEY1":VALUE1"[,"KEY2":"VALUE2"...]}'
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
        annotationless_pass_range:
          KEY1: VALUE1
          KEY2: VALUE2
```

#### How to update scenario conditions

The driving-log-replayer-cli has the ability to run multiple scenarios in succession that exist under the data_directory of a profile.
On the other hand, when evaluation conditions are given as arguments, the same arguments are applied to multiple scenarios, which is inconvenient.

In the case of local testing using driving-log-replayer-cli, instead of specifying arguments, the following commands are provided so that scenario conditions can be updated as needed.

- update-condition command to manually update scenario conditions
- run's -u option to automatically update scenario conditions after a simulation run

There are two ways to update

- existing Update only those items that appear in the scenario
- all Update all values in the metrics

```shell
# manual update
dlr simulation update-condition -s ${scenario_path} -r ${result.jsonl_path} -u ${existing|all}

# automatically update scenario after simulation run
dlr simulation run -p annotationless_perception -u ${existing|all}
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
dlr simulation run -p annotationless_perception -l sensing:=true
```

#### WebAutoCLI

```shell
webauto ci scenario run --project-id ${project-id} --scenario-id ${scenario-id} --scenario-version-id ${scenario-version-id} --simulator-parameter-overrides 'sensing=true'
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

See [sample](https://github.com/tier4/log_evaluator/blob/main/sample/annotationless_perception/scenario.yaml)

### Evaluation Result Format

See [sample](https://github.com/tier4/log_evaluator/blob/main/sample/annotationless_perception/result.json)

The format of each frame and the metrics format are shown below.
**NOTE: common part of the result file format, which has already been explained, is omitted.**

```json
{
  "Frame": {
    "Ego": {},
    "OBJECT_CLASSIFICATION": {
      // Recognized class
      "Result": { "Total": "Success or Fail", "Frame": "Success or Fail" }, // The results for Total and Frame are the same. The same values are output to make the data structure the same as other evaluations.
      "Info": {
        "name_min_max_mean": { "min": "min value", "max": "max value", "mean": "average value" },
        "name_metric_value": { "metric_value": "value"},
        ...
      },
      "Metrics": {
        "name_min_max_mean": {
          "min": "Minimum value of min",
          "max": "Maximum value of max",
          "mean": "Average value of mean"
        },
        ...
      }
    }
  }
}
```

See the figure below for the meaning of items

![lateral_deviation](./images/lateral_deviation.png)

![predicted_path_deviation](./images/predicted_path_deviation.png)
