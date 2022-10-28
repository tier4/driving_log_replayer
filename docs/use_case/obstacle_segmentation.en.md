# Evaluate point cloud generation

Evaluate if the Autoware point cloud generation process (if there is a connection between sensing and perception nodes) runs, and if data is being published to the `/perception/obstacle_segmentation/pointcloud` topic as intended.

The following evaluations are performed simultaneously to check if point cloud published by perception nodes is valid.

1. Check whether vehicles, pedestrians and other traffic participatns, annotated in advance, are detected (detection step).
2. Check whether extra point clouds appear in the overlapping area between the lane and the polygons around the vehicle defined in the scenario (non_detection step).

The recommended annotation tool is [Deepen](https://www.deepen.ai/), but any tool that supports conversion to `t4_dataset` format can be used.
Multiple annotation tools can be used as long as a conversion tool can be created.

## Evaluation method

The obstacle segmentation evaluation is executed by launching the `obstacle_segmentation.launch.py` file.
Launching the file executes the following steps:

1. Launch C++ evaluation node, Python evaluation node, launch `logging_simulator.launch` file, and execute `ros2 bag play` command.
2. Autoware receives sensor data output from input rosbag and outputs `/perception/obstacle_segmentation/pointcloud` topic.
3. The C++ evaluation node subscribes `/perception/obstacle_segmentation/pointcloud` topic and calculates the polygon of the non-detection area at the time specified in the header.
4. The non-detection area polygon along with a pointcloud is published to `/driving_log_replayer/obstacle_segmentation/input` topic.
5. Python evaluation node subscribes `/driving_log_replayer/obstacle_segmentation/input` and evaluates data. The result is dumped into a file.
6. When the playback of the rosbag is finished, Autoware's launch is automatically terminated, and the evaluation is completed.

## Evaluation Result

The results are calculated for each subscription. The format and available states are described below.

### Detection Normal

If all of the following conditions are met, the evaluation is reported as normal:

1. A bounding box with the UUID specified in the scenario must contain a point cloud (topic `/perception/obstacle_segmentation/pointcloud`) with at least a number of points equal to the specified number.
   - If multiple UUIDs are specified, the condition must be satisfied for all the specified bounding boxes.
2. The output rate of the point cloud cannot be in error state (this data is provided by Autoware's diagnostic function). The default frequency value is 1.0Hz or less.

### Detection Warning

The state is achieved when the visibility of the bounding box with the UUID specified in the scenario is none (bounding box is occluded) and cannot be evaluated.

### Detection Error

The detection state is `Error` when neither conditions for `Normal` nor `Warning` state cannot be met.

### Non-Detection Normal

The state is `Normal` when no point is contained in the non-detection area, which is calculated by the C++ node in step 3 of the evaluation method.

### Non-Detection Error

The state is `Error` when any point was found in the non-detection area.

## Topic name and data type used by evaluation node

Subscribed topics:

| Topic name                                      | Data type                                    |
| ----------------------------------------------- | -------------------------------------------- |
| /perception/obstacle_segmentation/pointcloud    | sensor_msgs::msg::PointCloud2                |
| /diagnostics_agg                                | diagnostic_msgs::msg::DiagnosticArray        |
| /map/vector_map                                 | autoware_auto_mapping_msgs::msg::HADMapBin   |
| /tf                                             | tf2_msgs/msg/TFMessage                       |
| /planning/scenario_planning/status/stop_reasons | tier4_planning_msgs::msg::StopReasonArray    |
| /planning/scenario_planning/trajectory          | autoware_auto_planning_msgs::msg::Trajectory |

Published topics:

| Topic name                                        | Data type                                                |
| ------------------------------------------------- | -------------------------------------------------------- |
| /driving_log_replayer/obstacle_segmentation/input | driving_log_replayer_msgs::msg:ObstacleSegmentationInput |
| /driving_log_replayer/marker/detection            | visualization_msgs::msg::MarkerArray                     |
| /driving_log_replayer/marker/non_detection        | visualization_msgs::msg::MarkerArray                     |
| /driving_log_replayer/pcd/detection               | sensor_msgs::msg::PointCloud2                            |
| /driving_log_replayer/pcd/non_detection           | sensor_msgs::msg::PointCloud2                            |
| /planning/mission_planning/goal                   | geometry_msgs::msg::PoseStamped                          |

## Arguments passed to logging_simulator.launch

To make Autoware processing less resource-consuming, modules that are not relevant to evaluation are disabled by passing the `false` parameter as a launch argument.
The following parameters are set to `false` when launching the `obstacle_segmentation` evaluation scenario:

- localization: false
- control: false

## About simulation

State the information required to run the simulation.

### Topic to be included in the input rosbag

Must contain the required topics in `t4_dataset` format.

## About Evaluation

State the information necessary for the evaluation.

### Scenario Format

```yaml
Evaluation:
  UseCaseName: obstacle_segmentation
  UseCaseFormatVersion: 0.1.0
  Datasets:
    - sample_dataset:
        VehicleId: default
        LocalMapPath: $HOME/autoware_map/sample-map-planning
  Conditions:
    ObstacleDetection:
      PassRate: 99.0 # How much (%) of the evaluation attempts are considered successful.
    NonDetection:
      PassRate: 99.0 # How much (%) of the evaluation attempts are considered successful.
      ProposedArea: # Non-detection area centered on the base_link with a single stroke polygon.
        polygon_2d: # Describe polygon in xy-plane in clockwise direction
          - [10.0, 1.5]
          - [10.0, -1.5]
          - [0.0, -1.5]
          - [0.0, 1.5]
        z_min: 0.0 # Lower z for 3D polygon
        z_max: 1.5 # Upper z for 3D polygon
  SensingEvaluationConfig:
    evaluation_config_dict:
      evaluation_task: sensing # fixed value
      target_uuids: # UUIDs of bounding box to be detected
        - dcb2b352232fff50c4fad23718f31611
      box_scale_0m: 1.0 # Scaling factor to scale the bounding box according to distance. Value at 0m
      box_scale_100m: 1.0 # Scaling factor at 100m. Magnification is determined by linear completion according to distance from 0 to 100m
      min_points_threshold: 1 # Threshold of how many points must be in the bounding box to be successful.
```

### Evaluation Result Format

In `obstacle_segmentation` evaluation scenario, two types of checks, detection (Detection) and non-detection (NonDetection), are evaluated.
Although they are evaluated simultaneously, in one callback function, they are counted separately.
The `Result` is `true` if both detection and non-detection evaluation steps have passed. Otherwise the `Result` is `false`.

An example of evaluation is described below.
*NOTE: common part of the result file format, which has already been explained, is omitted.*

```json
{
  "Frame": {
    "FrameName": "Frame number of t4_dataset used for evaluation",
    "FrameSkip": "Number of times that an object was requested to be evaluated but the evaluation was skipped because there was no ground truth in the dataset within 75msec",
    "Detection": {
      "Result": "Success or Warn or Fail",
      "Info": [
        {
          "Annotation": "Annotated banding box information, position pose, and ID",
          "PointCloud": "Information on the evaluated point cloud, number of points in the bounding box and position of the nearest point from the base_link."
        }
      ]
    },
    "NonDetection": {
      "Result": "Success or Fail",
      "Info": [
        {
          "PointCloud": "Number of points out in the non-detection area and distribution by distance from the base_link."
        }
      ]
    },
    "StopReasons": "Reasons for stopping output by the Planning module. Reference value",
    "TopicRate": "Result of diag indicating whether the output rate of the point cloud is normal or not."
  }
}
```
