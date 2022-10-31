# Evaluate perception

The performance of Autoware's recognition function (perception) is evaluated by calculating mAP (mean Average Precision) and other indices from the recognition results.

Run the perception module and pass the output perception topic to the evaluation library for evaluation.

## Evaluation method

The perception evaluation is executed by launching the `perception.launch.py` file.
Launching the file executes the following steps:

1. Execute launch of evaluation node (`perception_evaluator_node`), `logging_simulator.launch` file and `ros2 bag play` command
2. Autoware receives sensor data output from input rosbag and outputs point cloud data, and the perception module performs recognition.
3. The evaluation node subscribes to `/perception/object_recognition/{detection, tracking}/objects` and evaluates data. The result is dumped into a file.
4. When the playback of the rosbag is finished, Autoware's launch is automatically terminated, and the evaluation is completed.

### Notes

Autoware's perception module is launched for the first time, it converts `onnx` files in `lidar_centerpoint` package.
In step 1 of the evaluation method, there is a long, seemingly halting process as it waits for the conversion to complete.
This is normal operation, so please do not stop with keyboard input, but wait as is.

## Evaluation results

The results are calculated for each subscription. The format and available states are described below.

### Perception Normal

If no object fail the evaluation function `perception_eval` (`frame_result.pass_fail_result.get_fail_object_num() == 0`).

### Perception Error

The perception evaluation output is marked as `Error` when condition for ` Normal` is not met.

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
The following parameters are set to `false` when launching the `perception` evaluation scenario:

- localization: false
- planning: false
- control: false
- sensing: false / true (default value is false. Specify by `LaunchSensing` key for each t4_dataset in the scenario)

*NOTE: The `tf` in the bag is used to align the localization during annotation and simulation. Therefore, localization is invalid.*

## Dependent libraries

The perception evaluation step bases on the [perception_eval](https://github.com/tier4/autoware_perception_evaluation) library.

### Division of roles of driving_log_replayer with dependent libraries

`driving_log_replayer` package is in charge of the connection with ROS. The actual perception evaluation is conducted in  [perception_eval](https://github.com/tier4/autoware_perception_evaluation) library.
The [perception_eval](https://github.com/tier4/autoware_perception_evaluation) is a ROS-independent library, it cannot receive ROS objects. Also, ROS timestamps use nanoseconds while the `t4_dataset` format is based on milliseconds (because it uses `nuScenes`), so the values must be properly converted before using the library's functions.

`driving_log_replayer` subscribes the topic output from the perception module of Autoware, converts it to the data format expected by [perception_eval](https://github.com/tier4/autoware_perception_evaluation), and passes it on.
It is also responsible for publishing and visualizing the evaluation results from [perception_eval](https://github.com/tier4/autoware_perception_evaluation) on proper ROS topic.

[perception_eval](https://github.com/tier4/autoware_perception_evaluation) is in charge of the part that compares the detection results passed from `driving_log_replayer` with ground truth data, calculates the index, and outputs the results.

## About simulation

State the information required to run the simulation.

### Topic to be included in the input rosbag

Must contain the required topics in `t4_dataset` format.

## About Evaluation

State the information necessary for the evaluation.

### Scenario Format

There are two types of evaluation: use case evaluation and database evaluation.
Use case evaluation is performed on a single dataset, while database evaluation uses multiple datasets and takes the average of the results for each dataset.

In the database evaluation, the `vehicle_id` should be able to be set for each data set, since the calibration values may change.
Also, it is necessary to set whether or not to activate the sensing module.

```yaml
Evaluation:
  UseCaseName: perception
  UseCaseFormatVersion: 0.3.0
  Datasets:
    - sample_dataset:
        VehicleId: default
        LaunchSensing: false
        LocalMapPath: $HOME/autoware_map/sample-map-planning
  Conditions:
    PassRate: 99.0 # How much (%) of the evaluation attempts are considered successful.
  PerceptionEvaluationConfig:
    evaluation_config_dict:
      evaluation_task: detection # detection or tracking. Evaluate the objects specified here
      target_labels: [car, bicycle, pedestrian, motorbike] # evaluation label
      max_x_position: 102.4 # Maximum x position of object to be evaluated
      max_y_position: 102.4 # Maximum y position of object to be evaluated
      max_distance: null # Maximum distance from the base_link of the object to be evaluated. Exclusive use with max_x_potion, max_y_position.
      min_distance: null # Minimum distance from the base_link of the object to be evaluated. Exclusive use with max_x_potion, max_y_position.
      min_point_numbers: [0, 0, 0, 0] # Minimum number of point clouds in bounding box for ground truth object. If min_point_numbers=0, all ground truth objects are evaluated
      confidence_threshold: null # Threshold of confidence for the estimated object to be evaluated
      target_uuids: null # If you want to evaluate only specific ground truths, specify the UUIDs of the ground truths to be evaluated. If null, use all
      center_distance_thresholds: [[1.0, 1.0, 1.0, 1.0], [2.0, 2.0, 2.0, 2.0]] # Threshold for center-to-center distance matching
      plane_distance_thresholds: [2.0, 30.0] # Threshold for planar distance matching
      iou_bev_thresholds: [0.5] # Threshold for BEV IoU
      iou_3d_thresholds: [0.5] # Threshold for 3D IoU
  CriticalObjectFilterConfig:
    target_labels: [car, bicycle, pedestrian, motorbike]
    max_x_position_list: [30.0, 30.0, 30.0, 30.0]
    max_y_position_list: [30.0, 30.0, 30.0, 30.0]
    max_distance_list: null
    min_distance_list: null
    min_point_numbers: [0, 0, 0, 0]
    confidence_threshold_list: null
    target_uuids: null
  PerceptionPassFailConfig:
    target_labels: [car, bicycle, pedestrian, motorbike]
    plane_distance_threshold_list: [2.0, 2.0, 2.0, 2.0]
```

### Evaluation Result Format

The evaluation results by [perception_eval](https://github.com/tier4/autoware_perception_evaluation) under the conditions specified in the scenario are output for each frame.
Only the final line has a different format from the other lines since the final metrics are calculated after all data has been flushed.

The format of each frame and the metrics format are shown below.
*NOTE: common part of the result file format, which has already been explained, is omitted.*

Format of each frame:

```json
{
  "Frame": {
    "FrameName": "Frame number of t4_dataset used for evaluation",
    "FrameSkip": "Number of times that an object was requested to be evaluated but the evaluation was skipped because there was no ground truth in the dataset within 75msec",
    "PassFail": {
      "Result": "Success or Fail",
      "Info": [
        {
          "TP": "Number of TPs",
          "FP": "Number of FPs",
          "FN": "Number of FNs"
        }
      ]
    }
  }
}
```

Metrics Data Format:

```json
{
  "Frame": {
    "FinalScore": {
      "Score": {
        "TP": "TP rate of the label",
        "FP": "FP rate of the label",
        "FN": "FN rate of the label",
        "AP": "AP value of the label",
        "APH": "APH value of the label"
      },
      "Error": {
        "Label": "Error metrics for the label"
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
