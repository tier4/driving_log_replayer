# Evaluate perception

The performance of Autoware's recognition function (perception) is evaluated by calculating mAP (mean Average Precision) and other indices from the recognition results.

Run the perception module and pass the output perception topic to the evaluation library for evaluation.

## Evaluation method

Use perception.launch.py to evaluate.
When launch is launched, the following is executed and evaluated.

1. launch evaluation node (perception_evaluator_node), logging_simulator.launch and ros2 bag play
2. autoware receives sensor data output from bag, outputs point cloud data, and the perception module performs recognition
3. the evaluation node subscribe to /perception/object_recognition/{detection, tracking}/objects, evaluates it using the function perception_eval in the callback and records the result in a file
4. when the playback of the bag is finished, launch is automatically terminated and the evaluation is finished.

### Notes

It may appear that bag play starts at the time of launch in step 1 of the evaluation method.
This is because when autoware workspace is set up and perception module is launched for the first time, it waits for the conversion process of lidar_centerpoint's onnx files.
Wait until the string "lidar_centerpoint engine files" are generated.

## Evaluation results

For each subscription, the judgment result described below is output.

### Normal

If there are 0 objects with evaluation failure using the function perception_eval, frame_result.pass_fail_result.get_fail_object_num() == 0

### Error

If there are objects with evaluation failure using the function of perception_eval, frame_result.pass_fail_result.get_fail_object_num() > 0

## Topic name and data type used by evaluation node

- subscribe

| Topic name                                       | Data type                                         |
| ------------------------------------------------ | ------------------------------------------------- |
| /perception/object_recognition/detection/objects | autoware_auto_perception_msgs/msg/DetectedObjects |
| /perception/object_recognition/tracking/objects  | autoware_auto_perception_msgs/msg/TrackedObjects  |

- publish

| Topic name                                | Data type                            |
| ----------------------------------------- | ------------------------------------ |
| /driving_log_replayer/marker/ground_truth | visualization_msgs::msg::MarkerArray |
| /driving_log_replayer/marker/results      | visualization_msgs::msg::MarkerArray |

## Arguments passed to logging_simulator.launch

To lighten autoware processing, modules that are not relevant to evaluation are disabled by passing false as a launch argument.
The following is set.

The tf in the bag is used for the purpose of aligning the self-location during annotation and simulation. Therefore, localization is invalid.

- localization: false
- planning: false
- control: false
- sensing: false / true (default value is false. Specify in scenario)

## Dependent libraries

[perception_eval](https://github.com/tier4/autoware_perception_evaluation)

### division of roles of driving_log_replayer with dependent libraries

Driving_log_replayer is in charge of the connection with ROS, and perception_eval is in charge of handling the data set.
Since perception_eval is a ROS-independent library, it cannot receive ROS objects.
Also, timestamp is in nanoseconds in ROS, while milliseconds are used for datasets.
Since t4_dataset is based on nuScenes and nuScenes uses milliseconds, t4_dataset also uses milliseconds.

Driving_log_replayer subscribe the topic output from the perception module of autoware, converts it to the data format expected by perception_eval, and passes it on.
It is also responsible for publishing and visualizing the evaluation results returned from perception_eval in the ROS topic.

Perception_eval is in charge of the part that compares the detection results passed from driving_log_replayer with GroundTruth, calculates the index, and outputs the results.

## About simulation

State the information required to run the simulation.

### Topic to be included in the input rosbag

Must contain the required topics in t4_dataset

## About Evaluation

State the information necessary for the evaluation.

### Scenario Format

There are two types of evaluation: use case evaluation and database evaluation.
Use case evaluation is performed on a single dataset, while database evaluation uses multiple datasets and takes the average of the results for each dataset.

In the database evaluation, the vehicle_id should be able to be set for each data set, since the calibration values may change.
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

In perception, the results of evaluation by perception_eval under the conditions specified in the scenario are output for each frame.
Only the final line has a different format from the other lines, since the final metrics are calculated after all data has been flushed.

The format of each frame and the format of the metrics are shown below.
However, common parts that have already been explained in the result file format are omitted.

Format of each frame

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
          "FN": "Number of FNs",
        }
      ]
    }
  }
}
```

Metrics Data Format

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

In database evaluation, it is necessary to replay multiple bags, but due to the ROS specification, it is not possible to use multiple bags in a single launch.
Since one bag, i.e., one t4_dataset, requires one launch, it is necessary to execute as many launches as the number of datasets contained in the database evaluation.

Since database evaluation cannot be done in a single launch, perception outputs a file scene_result.pkl in addition to result.jsonl.
The pickle file is a python object saved as a file, PerceptionEvaluationManager.frame_results of perception_eval.
The dataset evaluation can be performed by reading all the objects recorded in the pickle file and outputting the index of the average of the dataset.

### Result file of database evaluation

In the case of a database evaluation with multiple datasets in the scenario, a file named database_result.json is output to the results output directory.

The format is the same as the metrics format.
