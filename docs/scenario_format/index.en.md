# Driving Log Replayer Scenario Format Definition

This section describes the scenario format used in driving_log_replayer.

## Notes on the format

- Keys are defined in CamelCase.
- Unless otherwise specified, the coordinate system is `map` coordinate system.
- Unless otherwise specified, the following unit system is used.

```shell
Distance: m
Velocity: m/s
acceleration: m/s^2
Time: s
```

## Samples

Sample scenarios are stored in the [sample](https://github.com/tier4/driving_log_replayer/tree/develop/sample) folder.

## Format

The basic structure is as follows. Details of each key are described below.

### 2.x.x Format

For `localization`, `performance_diag`, `yabloc`, `eagleye` and `ar_tag_based_localizer` evaluation scenarios

```yaml
ScenarioFormatVersion: 2.x.x
ScenarioName: String
ScenarioDescription: String
SensorModel: String
VehicleModel: String
VehicleId: String
LocalMapPath: String
Evaluation:
  UseCaseName: String
  UseCaseFormatVersion: String
  Conditions: Dictionary # refer use case
```

### 3.x.x Format

For `perception` and `obstacle_segmentation` evaluation scenarios.

**NOTE: VehicleId and LocalMapPath have been changed to be set for each id of t4_dataset.**

```yaml
ScenarioFormatVersion: 3.x.x
ScenarioName: String
ScenarioDescription: String
SensorModel: String
VehicleModel: String
PerceptionMode: String
Evaluation:
  UseCaseName: String
  UseCaseFormatVersion: String
  Datasets:
    - DatasetName:
        VehicleId: String
        LocalMapPath: String
  Conditions: Dictionary # refer use case
```

### ScenarioFormatVersion

Describe the version information of the scenario format. Use the semantic version.

`localization`, `performance_diag`, `yabloc`, `eagleye` and `ar_tag_based_localizer` scenarios use the 2.x.x series. The latest version of 2.x.x is 2.2.0.
`perception` and `obstacle_segmentation` scenarios use 3.x.x series. The latest version of 3.x.x is 3.1 .0

Minor versions are updated each time the format is updated.

### ScenarioName

Describes the name of the scenario, used as the display name of the scenario on the Autoware Evaluator.

### ScenarioDescription

Describes a scenario, used as a scenario description on the Autoware Evaluator.

### SensorModel

Specify `sensor_model` as argument in `autoware_launch/launch/logging_simulator.launch.xml`

### VehicleModel

Specify `vehicle_model` as an argument in `autoware_launch/launch/logging_simulator.launch.xml`

### VehicleId

Specify `vehicle_id` as an argument in `autoware_launch/launch/logging_simulator.launch.xml`

If you don't know `vehicle_id`, set `default`.

### PerceptionMode

Specify the `perception_mode` in `autoware_launch/launch/autoware.launch.xml`.

### LocalMapPath

Describes the path of the map folder to be used in the local environment.

Environment variables such as `$HOME` can be used.

### Evaluation

Define the evaluation conditions for the simulation.

#### UseCaseName

Specify an evaluation program.

The evaluation is executed by calling the launch file with the name specified here.
The `launch.py` file with the same name as specified must exist in the `driving_log_replayer/launch` folder.

#### UseCaseFormatVersion

Describe the version information of the use case format. The semantic version shall be used.
Until the major version becomes 1, the minor version is updated every time the format is updated.
The initial version is 0.1.0.

#### Conditions

Specify conditions that can be set for each use case.

Refer to each [use case](../use_case/index.en.md) for the conditions that can be specified.
