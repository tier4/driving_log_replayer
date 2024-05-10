# Evaluate perception

The performance of Autoware's recognition function (perception) is evaluated by calculating mAP (mean Average Precision) and other indices from the recognition results.

Run the perception module and pass the output perception topic to the evaluation library for evaluation.

Since it is activated in the perception_mode described in the scenario, change the scenario if you want to change the sensor to be evaluated.

## Preparation

In perception evaluation, machine learning pre-trained models are used.
If the model is not prepared in advance, Autoware will not output recognition results.
If no evaluation results are produced, check to see if this has been done correctly.

### Downloading Model Files

Models are downloaded during Autoware setup.
The method of downloading models depends on the version of Autoware you are using, so check which method is used.
The following patterns exist.

#### Download with ansible

When you run the ansible setup script, you will see `Download artifacts? [y/N]`, type `y` and press enter (Autoware foundation's main branch use this method)
<https://github.com/autowarefoundation/autoware/blob/main/ansible/roles/artifacts/tasks/main.yaml>

#### Automatically downloaded when the package is built

If you are using a slightly older Autoware.universe, this is the one to use, until the commit hash of `13b96ad3c636389b32fea3a47dfb7cfb7813cadc`.
[lidar_centerpoint/CMakeList.txt](https://github.com/autowarefoundation/autoware.universe/blob/13b96ad3c636389b32fea3a47dfb7cfb7813cadc/perception/lidar_centerpoint/CMakeLists.txt#L112-L118)

### Conversion of model files

The downloaded onnx file is not to be used as-is, but to be converted to a TensorRT engine file for use.
A conversion command is available, so source the autoware workspace and execute the command.

Let's assume that autoware is installed in `$HOME/autoware`.

```shell
source $HOME/autoware/install/setup.bash
ros2 launch lidar_centerpoint lidar_centerpoint.launch.xml build_only:=true
```

When the conversion command finishes, the engine file is output.
The output destination changes according to the model download method, so check that the output is in the appropriate directory.

#### Download with ansible

An example of the use of autowarefoundation's autoware.universe is shown below.

The following file is output.

```shell
$HOME/autoware_data/lidar_centerpoint/pts_backbone_neck_head_centerpoint_tiny.engine
$HOME/autoware_data/lidar_centerpoint/pts_voxel_encoder_centerpoint_tiny.engine
```

#### Automatic download at package build time

The following file is output.

```shell
$HOME/autoware/install/lidar_centerpoint/share/lidar_centerpoint/data/pts_backbone_neck_head_centerpoint_tiny.engine
$HOME/autoware/install/lidar_centerpoint/share/lidar_centerpoint/data/pts_voxel_encoder_centerpoint_tiny.engine
```

## Evaluation method

The perception evaluation is executed by launching the `perception.launch.py` file.
Launching the file executes the following steps:

1. Execute launch of evaluation node (`perception_evaluator_node`), `logging_simulator.launch` file and `ros2 bag play` command
2. Autoware receives sensor data output from input rosbag and outputs point cloud data, and the perception module performs recognition.
3. The evaluation node subscribes to `/perception/object_recognition/{detection, tracking}/objects` and evaluates data. The result is dumped into a file.
4. When the playback of the rosbag is finished, Autoware's launch is automatically terminated, and the evaluation is completed.

## Evaluation results

The results are calculated for each subscription. The format and available states are described below.

### Perception Normal

Satisfy Criteria in the Criterion tag of the scenario.

The scenario.yaml of the sample is as follows,

```yaml
Criterion:
  - PassRate: 95.0 # How much (%) of the evaluation attempts are considered successful.
    CriteriaMethod: num_tp # Method name of criteria (num_tp/metrics_score)
    CriteriaLevel: hard # Level of criteria (perfect/hard/normal/easy, or custom value 0.0-100.0)
    Filter:
      Distance: 0.0-50.0 # [m] null [Do not filter by distance] or lower_limit-(upper_limit) [Upper limit can be omitted. If omitted value is 1.7976931348623157e+308]
  - PassRate: 95.0 # How much (%) of the evaluation attempts are considered successful.
    CriteriaMethod: num_tp # Method name of criteria (num_tp/metrics_score)
    CriteriaLevel: easy # Level of criteria (perfect/hard/normal/easy, or custom value 0.0-100.0)
    Filter:
      Distance: 50.0- # [m] null [Do not filter by distance] or lower_limit-(upper_limit) [Upper limit can be omitted. If omitted value is 1.7976931348623157e+308]
```

- For each subscription of `/perception/object_recognition/{detection, tracking}/objects`, the number of objects in tp is hard (75.0%) or more for objects at a distance of 0.0-50.0[m]. Frame of Result becomes Success.
- For one subscription of `/perception/object_recognition/{detection, tracking}/objects`, the number of objects in tp is easy (25.0%) or more for objects at a distance of 50.0-1.7976931348623157e+308[m]. Frame of Result becomes Success.
- If the condition `PassRate >= Normal / Total Received * 100` is satisfied, the Total of Result becomes Success.

### Perception Error

The perception evaluation output is marked as `Error` when condition for `Normal` is not met.

### Skipping evaluation

Only add 1 to FrameSkip in the following cases.
FrameSkip is a counter for the number of times evaluation is skipped.

- No Ground Truth exists within 75msec before or after the received object's header time.
- If the number of footprint.points of the received object is 1 or 2 (this condition will be removed when "perception_eval" is updated)

### Skipping evaluation(NoGTNoObject)

- When the Ground Truth and the recognition objects are filtered by the filter condition and not evaluated (when the content of the evaluation result PassFail object is empty).

## Topic name and data type used by evaluation node

Subscribed topics:

| Topic name                                       | Data type                                         |
| ------------------------------------------------ | ------------------------------------------------- |
| /perception/object_recognition/detection/objects | autoware_auto_perception_msgs/msg/DetectedObjects |
| /perception/object_recognition/tracking/objects  | autoware_auto_perception_msgs/msg/TrackedObjects  |

Published topics:

| Topic name                                | Data type                            |
| ----------------------------------------- | ------------------------------------ |
| /driving_log_replayer/marker/ground_truth | visualization_msgs::msg::MarkerArray |
| /driving_log_replayer/marker/results      | visualization_msgs::msg::MarkerArray |

## Arguments passed to logging_simulator.launch

To make Autoware processing less resource-consuming, modules that are not relevant to evaluation are disabled by passing the `false` parameter as a launch argument.
The following parameters are set to `false`:

- localization: false
- planning: false
- control: false
- sensing: false / true (default value is false. Specify by `LaunchSensing` key for each t4_dataset in the scenario)

**NOTE: The `tf` in the bag is used to align the localization during annotation and simulation. Therefore, localization is invalid.**

## Dependent libraries

The perception evaluation step bases on the [perception_eval](https://github.com/tier4/autoware_perception_evaluation) library.

### Division of roles of driving_log_replayer with dependent libraries

`driving_log_replayer` package is in charge of the connection with ROS. The actual perception evaluation is conducted in [perception_eval](https://github.com/tier4/autoware_perception_evaluation) library.
The [perception_eval](https://github.com/tier4/autoware_perception_evaluation) is a ROS-independent library, it cannot receive ROS objects. Also, ROS timestamps use nanoseconds while the `t4_dataset` format is based on milliseconds (because it uses `nuScenes`), so the values must be properly converted before using the library's functions.

`driving_log_replayer` subscribes the topic output from the perception module of Autoware, converts it to the data format defined in [perception_eval](https://github.com/tier4/autoware_perception_evaluation), and passes it on.
It is also responsible for publishing and visualizing the evaluation results from [perception_eval](https://github.com/tier4/autoware_perception_evaluation) on proper ROS topic.

[perception_eval](https://github.com/tier4/autoware_perception_evaluation) is in charge of the part that compares the detection results passed from `driving_log_replayer` with ground truth data, calculates the index, and outputs the results.

## About simulation

State the information required to run the simulation.

### Topic to be included in the input rosbag

Must contain the required topics in `t4_dataset` format.

The vehicle's ECU CAN and sensors data topics are required for the evaluation to be run correctly.
The following example shows the topic list available in evaluation input rosbag when multiple LiDARs and Cameras are used in a real-world vehicle configuration.

/sensing/lidar/concatenated/pointcloud is used if the scenario LaunchSensing: false.

| Topic name                                           | Data type                                    |
| ---------------------------------------------------- | -------------------------------------------- |
| /gsm8/from_can_bus                                   | can_msgs/msg/Frame                           |
| /localization/kinematic_state                        | nav_msgs/msg/Odometry                        |
| /sensing/camera/camera\*/camera_info                 | sensor_msgs/msg/CameraInfo                   |
| /sensing/camera/camera\*/image_rect_color/compressed | sensor_msgs/msg/CompressedImage              |
| /sensing/gnss/ublox/fix_velocity                     | geometry_msgs/msg/TwistWithCovarianceStamped |
| /sensing/gnss/ublox/nav_sat_fix                      | sensor_msgs/msg/NavSatFix                    |
| /sensing/gnss/ublox/navpvt                           | ublox_msgs/msg/NavPVT                        |
| /sensing/imu/tamagawa/imu_raw                        | sensor_msgs/msg/Imu                          |
| /sensing/lidar/concatenated/pointcloud               | sensor_msgs/msg/PointCloud2                  |
| /sensing/lidar/\*/velodyne_packets                   | velodyne_msgs/VelodyneScan                   |
| /tf                                                  | tf2_msgs/msg/TFMessage                       |

The vehicle topics can be included instead of CAN.

| Topic name                                           | Data type                                           |
| ---------------------------------------------------- | --------------------------------------------------- |
| /localization/kinematic_state                        | nav_msgs/msg/Odometry                               |
| /sensing/camera/camera\*/camera_info                 | sensor_msgs/msg/CameraInfo                          |
| /sensing/camera/camera\*/image_rect_color/compressed | sensor_msgs/msg/CompressedImage                     |
| /sensing/gnss/ublox/fix_velocity                     | geometry_msgs/msg/TwistWithCovarianceStamped        |
| /sensing/gnss/ublox/nav_sat_fix                      | sensor_msgs/msg/NavSatFix                           |
| /sensing/gnss/ublox/navpvt                           | ublox_msgs/msg/NavPVT                               |
| /sensing/imu/tamagawa/imu_raw                        | sensor_msgs/msg/Imu                                 |
| /sensing/lidar/concatenated/pointcloud               | sensor_msgs/msg/PointCloud2                         |
| /sensing/lidar/\*/velodyne_packets                   | velodyne_msgs/VelodyneScan                          |
| /tf                                                  | tf2_msgs/msg/TFMessage                              |
| /vehicle/status/control_mode                         | autoware_auto_vehicle_msgs/msg/ControlModeReport    |
| /vehicle/status/gear_status                          | autoware_auto_vehicle_msgs/msg/GearReport           |
| /vehicle/status/steering_status                      | autoware_auto_vehicle_msgs/SteeringReport           |
| /vehicle/status/turn_indicators_status               | autoware_auto_vehicle_msgs/msg/TurnIndicatorsReport |
| /vehicle/status/velocity_status                      | autoware_auto_vehicle_msgs/msg/VelocityReport       |

### Topics that must not be included in the input rosbag

| Topic name | Data type               |
| ---------- | ----------------------- |
| /clock     | rosgraph_msgs/msg/Clock |

The clock is output by the --clock option of ros2 bag play, so if it is recorded in the bag itself, it is output twice, so it is not included in the bag.

## About Evaluation

State the information necessary for the evaluation.

### Scenario Format

There are two types of evaluation: use case evaluation and database evaluation.
Use case evaluation is performed on a single dataset, while database evaluation uses multiple datasets and takes the average of the results for each dataset.

In the database evaluation, the `vehicle_id` should be able to be set for each data set, since the calibration values may change.
Also, it is necessary to set whether or not to activate the sensing module.

See [sample](https://github.com/tier4/driving_log_replayer/blob/main/sample/perception/scenario.yaml).

### Evaluation Result Format

See [sample](https://github.com/tier4/driving_log_replayer/blob/main/sample/perception/result.json).

The evaluation results by [perception_eval](https://github.com/tier4/autoware_perception_evaluation) under the conditions specified in the scenario are output for each frame.
Only the final line has a different format from the other lines since the final metrics are calculated after all data has been flushed.

The format of each frame and the metrics format are shown below.
**NOTE: common part of the result file format, which has already been explained, is omitted.**

Format of each frame:

```json
{
  "Frame": {
    "FrameName": "Frame number of t4_dataset used for evaluation",
    "FrameSkip": "The total number of times the evaluation was skipped, which occurs when the evaluation of an object is requested but there is no Ground Truth in the dataset within 75msec, or when the number of footprint.points is 1 or 2.",
    "criteria0": {
      // result of criteria 0, If the Ground Truth and recognition objects exist
      "PassFail": {
        "Result": { "Total": "Success or Fail", "Frame": "Success or Fail" },
        "Info": {
          "TP": "Number of filtered objects determined to be TP",
          "FP": "Number of filtered objects determined to be FP",
          "FN": "Number of filtered objects determined to be FN"
        }
      }
    },
    "criteria1": {
      // result of criteria 1. If the Ground Truth and the recognition objects do not exist
      "NoGTNoObj": "Number of times that the Ground Truth and the recognition objects were filtered and could not be evaluated."
    }
  }
}
```

Warning Data Format:

```json
{
  "Frame": {
    "Warning": "Warning Message",
    "FrameSkip": "The total number of times the evaluation was skipped, which occurs when the evaluation of an object is requested but there is no Ground Truth in the dataset within 75msec, or when the number of footprint.points is 1 or 2."
  }
}
```

Metrics Data Format:

When the `evaluation_task` is detection or tracking

```json
{
  "Frame": {
    "FinalScore": {
      "Score": {
        "TP": {
          "ALL": "TP rate for all labels",
          "label0": "TP rate of label0",
          "label1": "TP rate of label1"
        },
        "FP": {
          "ALL": "FP rate for all labels",
          "label0": "FP rate of label0",
          "label1": "FP rate of label1"
        },
        "FN": {
          "ALL": "FN rate for all labels",
          "label0": "FN rate of label0",
          "label1": "FN rate of label1"
        },
        "TN": {
          "ALL": "TN rate for all labels",
          "label0": "TN rate of label0",
          "label1": "TN rate of label1"
        },
        "AP(Center Distance)": {
          "ALL": "AP(Center Distance) rate for all labels",
          "label0": "AP(Center Distance) rate of label0",
          "label1": "AP(Center Distance) rate of label1"
        },
        "APH(Center Distance)": {
          "ALL": "APH(Center Distance) rate for all labels",
          "label0": "APH(Center Distance) rate of label0",
          "label1": "APH(Center Distance) rate of label1"
        },
        "AP(IoU 2D)": {
          "ALL": "AP(IoU 2D) rate for all labels",
          "label0": "AP(IoU 2D) rate of label0",
          "label1": "AP(IoU 2D) rate of label1"
        },
        "APH(IoU 2D)": {
          "ALL": "APH(IoU 2D) rate for all labels",
          "label0": "APH(IoU 2D) rate of label0",
          "label1": "APH(IoU 2D) rate of label1"
        },
        "AP(IoU 3D)": {
          "ALL": "AP(IoU 3D) rate for all labels",
          "label0": "AP(IoU 3D) rate of label0",
          "label1": "AP(IoU 3D) rate of label1"
        },
        "APH(IoU 3D)": {
          "ALL": "APH(IoU 3D) rate for all labels",
          "label0": "APH(IoU 3D) rate of label0",
          "label1": "APH(IoU 3D) rate of label1"
        },
        "AP(Plane Distance)": {
          "ALL": "AP(Plane Distance) rate for all labels",
          "label0": "AP(Plane Distance) rate of label0",
          "label1": "AP(Plane Distance) rate of label1"
        },
        "APH(Plane Distance)": {
          "ALL": "APH(Plane Distance) rate for all labels",
          "label0": "APH(Plane Distance) rate of label0",
          "label1": "APH(Plane Distance) rate of label1"
        }
      },
      "MOTA": {"https://github.com/tier4/autoware_perception_evaluation/blob/develop/docs/en/perception/metrics.md#tracking"},
      "MOTA": {"https://github.com/tier4/autoware_perception_evaluation/blob/develop/docs/en/perception/metrics.md#tracking"},
      "IDswitch": {"https://github.com/tier4/autoware_perception_evaluation/blob/develop/docs/en/perception/metrics.md#id-switch"},
      "Error": {
        "ALL": {
          "average": {
            "x": "x position",
            "y": "y position",
            "yaw": "yaw",
            "length": "length",
            "width": "width",
            "vx": "x velocity",
            "vy": "y velocity",
            "nn_plane": "Nearest neighbor plane distance"
          },
          "rms": {
            "x": "x position",
            "y": "y position",
            "yaw": "yaw",
            "length": "length",
            "width": "width",
            "vx": "x velocity",
            "vy": "y velocity",
            "nn_plane": "Nearest neighbor plane distance"
          },
          "std": {
            "x": "x position",
            "y": "y position",
            "yaw": "yaw",
            "length": "length",
            "width": "width",
            "vx": "x velocity",
            "vy": "y velocity",
            "nn_plane": "Nearest neighbor plane distance"
          },
          "max": {
            "x": "x position",
            "y": "y position",
            "yaw": "yaw",
            "length": "length",
            "width": "width",
            "vx": "x velocity",
            "vy": "y velocity",
            "nn_plane": "Nearest neighbor plane distance"
          },
          "min": {
            "x": "x position",
            "y": "y position",
            "yaw": "yaw",
            "length": "length",
            "width": "width",
            "vx": "x velocity",
            "vy": "y velocity",
            "nn_plane": "Nearest neighbor plane distance"
          }
        },
        "label0": "Error metrics for the label0"
      }
    }
  }
}
```

When the `evaluation_task` is fp_validation

```json
{
  "Frame": {
    "FinalScore": {
      "GroundTruthStatus": {
        "UUID": {
          "rate": {
            "TP": "TP rate of the displyed UUID",
            "FP": "FP rate of the displyed UUID",
            "TN": "TN rate of the displyed UUID",
            "FN": "FN rate of the displyed UUID"
          },
          "frame_nums": {
            "total": "List of frame numbers, which GT is evaluated",
            "TP": "List of frame numbers, which GT is evaluated as TP",
            "FP": "List of frame numbers, which GT is evaluated as FP",
            "TN": "List of frame numbers, which GT is evaluated as TN",
            "FN": "List of frame numbers, which GT is evaluated as FN"
          }
        }
      },
      "Scene": {
        "TP": "TP rate of the scene",
        "FP": "FP rate of the scene",
        "TN": "TN rate of the scene",
        "FN": "FN rate of the scene"
      }
    }
  }
}
```

### pickle file

In database evaluation, it is necessary to replay multiple rosbags, but due to the ROS specification, it is impossible to use multiple bags in a single launch.
Since one rosbag, i.e., one `t4_dataset`, requires one launch, it is necessary to execute as many launches as the number of datasets contained in the database evaluation.

Since database evaluation cannot be done in a single launch, perception outputs a file `scene_result.pkl` in addition to `result.jsonl` file.
A pickle file is a python object saved as a file, PerceptionEvaluationManager.frame_results of [perception_eval](https://github.com/tier4/autoware_perception_evaluation).
The dataset evaluation can be performed by reading all the objects recorded in the pickle file and outputting the index of the dataset's average.

### Result file of database evaluation

In the case of a database evaluation with multiple datasets in the scenario, a file named `database_result.json` is output to the results directory.

The format is the same as the [Metrics Data Format](#evaluation-result-format).
