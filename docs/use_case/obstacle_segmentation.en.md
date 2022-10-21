# Evaluate point cloud generation

Evaluate if the Autoware point cloud process (sensing -> perception) runs and /perception/obstacle_segmentation/pointcloud is output as intended.

The judgment whether the point cloud is output as intended is made using t4_dataset and the point cloud.
The following evaluations are performed simultaneously

1. evaluation of whether vehicles, pedestrians, etc. annotated in advance are detected (detection)
2. evaluation of whether extra point clouds appear in the overlapping area between the lane and the polygons around the vehicle defined in the scenario (non_detection).

The recommended annotation tool is [Deepen](https://www.deepen.ai/), but any tool that supports conversion to t4_dataset is available.
Multiple annotation tools can be used as long as a conversion tool can be created.

## Evaluation method

Use obstacle_segmentation.launch.py to evaluate.
When launch is launched, the following is executed and evaluated.

1. launch C++ evaluation node, Python evaluation node, logging_simulator.launch, and ros2 bag play
2. autoware receives sensor data output from bag and outputs /perception/obstacle_segmentation/pointcloud
3. evaluation node of C++ subscribe /perception/obstacle_segmentation/pointcloud and calculate polygon of non-detection area at the time of header
4. publish polygon of non-detected area and pointcloud to /driving_log_replayer/obstacle_segmentation/input
5. Python evaluation node subscribe /driving_log_replayer/obstacle_segmentation/input and evaluate it using perception_eval in callback. Record the results in a file.
6. when the playback of the bag is finished, launch is automatically terminated and the evaluation is completed.

## Evaluation Result

For each subscription, the judgment result described below is output.

### Detection Normal

The bounding box with the UUID specified in the scenario must contain a point cloud (/perception/obstacle_segmentation/pointcloud) with the specified number of points or more.
If multiple UUIDs are specified, the condition must be satisfied for all the specified bounding boxes.
Also, the output rate of the point cloud must not be in error by the diagnostic function provided by autoware.
The default value is 1.0Hz or less.

### Detection Warning

If the visibility of the bounding box with the UUID specified in the scenario is none (occlusion state) and cannot be evaluated.

### Detection Error

If neither detection warning nor detection normal

### Non-Detection Normal

There must be no single point cloud in the non-detection area.

The non-detection area is the area calculated by the C++ node in step 3 of the evaluation method.

### Non-Detection Error

There are some point cloud in the non-detection area.

## Topic name and data type used by evaluation node

- subscribe

| Topic name                                      | Data type                                    |
| ----------------------------------------------- | -------------------------------------------- |
| /perception/obstacle_segmentation/pointcloud    | sensor_msgs::msg::PointCloud2                |
| /diagnostics_agg                                | diagnostic_msgs::msg::DiagnosticArray        |
| /map/vector_map                                 | autoware_auto_mapping_msgs::msg::HADMapBin   |
| /tf                                             | tf2_msgs/msg/TFMessage                       |
| /planning/scenario_planning/status/stop_reasons | tier4_planning_msgs::msg::StopReasonArray    |
| /planning/scenario_planning/trajectory          | autoware_auto_planning_msgs::msg::Trajectory |

- publish

| Topic name                                        | Data type                                                |
| ------------------------------------------------- | -------------------------------------------------------- |
| /driving_log_replayer/obstacle_segmentation/input | driving_log_replayer_msgs::msg:ObstacleSegmentationInput |
| /driving_log_replayer/marker/detection            | visualization_msgs::msg::MarkerArray                     |
| /driving_log_replayer/marker/non_detection        | visualization_msgs::msg::MarkerArray                     |
| /driving_log_replayer/pcd/detection               | sensor_msgs::msg::PointCloud2                            |
| /driving_log_replayer/pcd/non_detection           | sensor_msgs::msg::PointCloud2                            |
| /planning/mission_planning/goal                   | geometry_msgs::msg::PoseStamped                          |

## Arguments passed to logging_simulator.launch

To lighten autoware processing, modules that are not relevant to evaluation are disabled by passing false as a launch argument.
The following is set.

- localization: false
- control: false

## About simulation

State the information required to run the simulation.

### Topic to be included in the input rosbag

Must contain the required topics in t4_dataset

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

### Result Format

In obstacle_segmentation, two types of detection (Detection) and non-detection (NonDetection) are evaluated.
Although they are evaluated simultaneously in one point cloud callback, they are counted separately.
The Result is true if both detection and non-detection have passed, and false otherwise.

The format is shown below.
However, common parts that have already been explained in the result file format are omitted.

```json
{
  "Frame": {
    "FrameName": "評価に使用したt4_datasetのフレーム番号",
    "FrameSkip": "objectの評価を依頼したがdatasetに75msec以内の真値がなく評価を飛ばされた回数",
    "Detection": {
      "Result": "Success or Warn or Fail",
      "Info": [
        {
          "Annotation": "アノテーションされたバンディングボックスの情報、位置姿勢、ID",
          "PointCloud": "評価した点群の情報、バウンディングボックス内の点の数と、base_linkからの最近傍の点の位置"
        }
      ]
    },
    "NonDetection": {
      "Result": "Success or Fail",
      "Info": [
        {
          "PointCloud": "非検知エリアに出ている点の数と、base_linkからの距離毎の分布"
        }
      ]
    },
    "StopReasons": "Planning moduleが出力する停止理由。参考値",
    "TopicRate": "点群の出力レートが正常かどうかを示すdiagの結果"
  }
}
```
