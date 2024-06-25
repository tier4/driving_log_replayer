# Troubleshooting

Check if simulation does not work as expected

## Autoware does not start

### Cause 1

The sensor_model, vehicle_model, and vehicle_id specified in the scenario are not included in the Autoware workspace used.

### Example 1

```shell
❯ dlr simulation run -p localization
[INFO] [launch]: All log files can be found below /home/hyt/.ros/log/2024-06-07-12-37-19-365597-dpc2405001-1360746
[INFO] [launch]: Default logging verbosity is set to INFO
1717731451.040883 [77]       ros2: determined eno1 (udp/10.0.55.137) as highest quality interface, selected for automatic interface.
[ERROR] [launch]: Caught exception in launch (see debug for traceback): executed command failed. Command: xacro /home/hyt/ros_ws/pilot-auto/install/tier4_vehicle_launch/share/tier4_vehicle_launch/urdf/vehicle.xacro vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit config_dir:=/home/hyt/ros_ws/pilot-auto/install/individual_params/share/individual_params/config/default/sample_sensor_kit
Captured stderr output: error: package not found: "package 'sample_sensor_kit_description' not found, searching: ...
...
```

### Correction method, Check area 1

Check whether sensor_model, vehicle_model, and vehicle_id specified in the scenario exist in the autoware_path specified in the profile.

### Cause 2

The version of cli does not match the version of driving_log_replayer.

### Example 2

```shell
❯ dlr simulation run -p yabloc -l play_rate:=0.5
Usage: dlr simulation run [OPTIONS]
Try 'dlr simulation run -h' for help.

Error: No such option: -l
```

### Correction method, Check area 2

Check that the value of the version tag in the package.xml of the installed driving_log_replayer matches the version output by the CLI.

```shell
❯ dlr --version
1.18.0
```

## Autoware exits immediately after startup

### Cause

Incorrect scenario format

### Example

```shell
[localization_evaluator_node.py-55] [ERROR] [1717734608.157798307] [driving_log_replayer.localization_evaluator]: An error occurred while loading the scenario. 1 validation error for LocalizationScenario
[localization_evaluator_node.py-55] Evaluation.UseCaseFormatVersion
[localization_evaluator_node.py-55]   Input should be '1.2.0' or '1.3.0' [type=literal_error, input_value='1.0.0', input_type=str]
[localization_evaluator_node.py-55]     For further information visit https://errors.pydantic.dev/2.7/v/literal_error

scenario: direct
--------------------------------------------------
TestResult: Failed
ScenarioFormatError
--------------------------------------------------
```

```jsonl
{"Condition":{}}
{"Result":{"Success":false,"Summary":"NoData"},"Stamp":{"System":1717734608.157981},"Frame":{}}
{"Result":{"Success":false,"Summary":"ScenarioFormatError"},"Stamp":{"System":0},"Frame":{"ErrorMsg":"1 validation error for LocalizationScenario\nEvaluation.UseCaseFormatVersion\n  Input should be '1.2.0' or '1.3.0' [type=literal_error, input_value='1.0.0', input_type=str]\n    For further information visit https://errors.pydantic.dev/2.7/v/literal_error"}}
```

### Correction method, Check area

The result.jsonl file shows what the problem is, so fix it as instructed.
In the example, UseCaseFormatVersion should be 1.2.0 or 1.3.0, but it is 1.0.0, so it cannot be used.
Since the old format is used, you should fix it by referring to the scenario in the sample directory of the repository.

## The evaluation result is NoData

### Cause 1

Autoware is not outputting the topic to be evaluated.

### Example 1-1

The node that outputs the topic to be evaluated is not invoked from launch.
The true/false value in the launch file is incorrectly set.

### Correction method, Check area 1-1

Find the topic to be evaluated in the document and check if the publisher exists by doing topic info.
If the Publisher count: 0, it is highly likely that the system has not been started in the first place.

```shell
❯ ros2 topic info /perception/traffic_light_recognition/traffic_signals -v
Type: autoware_auto_perception_msgs/msg/TrafficSignalArray

Publisher count: 1 <- Make sure it's not 0.

Node name: crosswalk_traffic_light_estimator
Node namespace: /perception/traffic_light_recognition
Topic type: autoware_auto_perception_msgs/msg/TrafficSignalArray
Endpoint type: PUBLISHER
GID: 01.10.d8.43.57.21.7c.2d.98.25.db.df.00.00.46.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RELIABLE
  History (Depth): KEEP_LAST (1)
  Durability: VOLATILE
  Lifespan: Infinite
  Deadline: Infinite
  Liveliness: AUTOMATIC
  Liveliness lease duration: Infinite
```

### Example 1-2

The node is running at launch, but dies immediately after startup.

### Correction method, Check area 1-2

Search the terminal you started or console.log with ERROR.

The following is a log of a case in which no point cloud was produced at all.
Searching by ERROR shows that pointcloud_preprocessor is dead.
Check if component_container, which outputs topics, is not throwing an error.

```shell
[ERROR] [component_container_mt-18]: process has died [pid 95, exit code -6, cmd '/opt/ros/galactic/lib/rclcpp_components/component_container_mt --ros-args -r __node:=pointcloud_preprocessor_container -r __ns:=/sensing/lidar/pointcloud_preprocessor --params-file /tmp/launch_params_rh_9gxcs'].
```

### Example 1-3

Inconsistency between cuda, cuDNN, and TensorRT resulting in no perception recognition results.
This may occur when nvidia driver is updated by apt upgrade.

```shell
hyt@dpc1909014-2204:~/ros_ws/awf$ ros2 launch lidar_centerpoint lidar_centerpoint.launch.xml model_name:=centerpoint_tiny model_path:=/home/hyt/autoware_data/lidar_centerpoint model_param_path:=$(ros2 pkg prefix lidar_centerpoint --share)/config/centerpoint_tiny.param.yaml build_only:=true
[INFO] [launch]: All log files can be found below /home/hyt/.ros/log/2024-01-22-14-36-04-069409-dpc1909014-2204-3835027
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [lidar_centerpoint_node-1]: process started with pid [3835028]
[lidar_centerpoint_node-1] 1705901764.307868 [77] lidar_cent: determined enp4s0 (udp/10.0.53.59) as highest quality interface, selected for automatic interface.
[lidar_centerpoint_node-1] terminate called after throwing an instance of 'thrust::system::system_error'
[lidar_centerpoint_node-1]   what():  This program was not compiled for SM 75
[lidar_centerpoint_node-1] : cudaErrorInvalidDevice: invalid device ordinal
[ERROR] [lidar_centerpoint_node-1]: process has died [pid 3835028, exit code -6, cmd '/home/hyt/ros_ws/awf/install/lidar_centerpoint/lib/lidar_centerpoint/lidar_centerpoint_node --ros-args -r __node:=lidar_centerpoint --params-file /tmp/launch_params_60_o26mq --params-file /tmp/launch_params_79jodq9o --params-file /tmp/launch_params_spwl7uq2 --params-file /tmp/launch_params_ur_yt_y2 --params-file /tmp/launch_params_iqs0hf9o --params-file /tmp/launch_params_t6bo4aow --params-file /tmp/launch_params_ufdn98_7 --params-file /tmp/launch_params_7m7aj130 --params-file /tmp/launch_params_yr4emr64 --params-file /tmp/launch_params_u4_e0ngh --params-file /home/hyt/ros_ws/awf/install/lidar_centerpoint/share/lidar_centerpoint/config/centerpoint_tiny.param.yaml --params-file /home/hyt/ros_ws/awf/install/lidar_centerpoint/share/lidar_centerpoint/config/detection_class_remapper.param.yaml -r ~/input/pointcloud:=/sensing/lidar/pointcloud -r ~/output/objects:=objects'].
```

### Correction method, Check area 1-3

Check to see if `cudaErrorInvalidDevice: invalid device ordinal` is not showing.
If so, reinstall nvidia-driver, cuda, cuDNN, and TensorRT.

```shell
sudo apt-mark unhold cuda-*
sudo apt-mark unhold nvidia-*
sudo apt-mark unhold libcudnn*
sudo apt-mark unhold libnv*

sudo apt purge cuda-*
sudo apt purge nvidia-*
sudo apt purge libcudnn*
sudo apt purge libnv*

# install nvidia driver and run Autoware's setup-dev-env.sh
```

### Cause 2

Autoware is outputting the topic to be evaluated, but the node cannot subscribe.

### Example 2-1

Not obtained due to QoS mismatch

```shell
[component_container_mt-13] [WARN 1633081042.510824100] [localization.util.random_downsample_filter]: New subscription discovered on topic '/localization/util/downsample/pointcloud', requesting incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY
[component_container_mt-19] [WARN 1633081042.593132498] [sensing.lidar.occupancy_grid_map_outlier_filter]: New subscription discovered on topic '/sensing/lidar/no_ground/pointcloud', requesting incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY
[component_container_mt-19] [WARN 1633081042.597116410] [sensing.lidar.concatenate_data]: New subscription discovered on topic '/sensing/lidar/concatenated/pointcloud', requesting incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY
```

### Correction method, Check area 2-1

Search for QoS in the terminal or console.log.

Check that the version of Autoware and the version of driving_log_replayer are compatible.
If you are experiencing this problem using Autoware Foundation main and driving_log_replayer main, please report it in a github issue.

### Example 2-2

Not retrieved due to message type mismatch.
This occurs because the type output by Autoware is different from the type expected by driving_log_replayer.

In June 2024, autoware_auto_msg was changed to autoware_msg. As a result, if the version of autoware and the version of driving_log_replayer do not correspond, this message will appear.

```shell
[ros2-67] [ERROR] [1717610261.542314281] [ROSBAG2_TRANSPORT]: Topic '/perception/object_recognition/tracking/objects' has more than one type associated. Only topics with one type are supported
[ros2-67] [ERROR] [1717610261.721551659] [ROSBAG2_TRANSPORT]: Topic '/perception/object_recognition/tracking/objects' has more than one type associated. Only topics with one type are supported
[ros2-67] [ERROR] [1717610261.903905941] [ROSBAG2_TRANSPORT]: Topic '/perception/object_recognition/tracking/objects' has more than one type associated. Only topics with one type are supported
[ros2-67] [ERROR] [1717610262.084860123] [ROSBAG2_TRANSPORT]: Topic '/perception/object_recognition/tracking/objects' has more than one type associated. Only topics with one type are supported
[ros2-67] [ERROR] [1717610262.263855979] [ROSBAG2_TRANSPORT]: Topic '/perception/object_recognition/tracking/objects' has more than one type associated. Only topics with one type are supported
[ros2-67] [ERROR] [1717610262.442275790] [ROSBAG2_TRANSPORT]: Topic '/perception/object_recognition/tracking/objects' has more than one type associated. Only topics with one type are supported
```

### Correction method, Check area 2-2

If there are major functionality changes, the required Autoware features (PR numbers, etc.) are listed in driving_log_replayer's ReleaseNotes.md.
Check if the required functions are included in the Autoware you are using.

If you are using Autoware Foundation's main and driving_log_replayer's main and are experiencing this issue, please report it in an issue on github.

## Unusually low number of evaluations

### Cause 1

Due to insufficient PC performance, Autoware is not able to publish the topic at the required period (10 Hz for point clouds).

### Example 1

```shell
❯ ros2 topic hz /perception/obstacle_segmentation/pointcloud
1718083964.779455 [77]       ros2: determined eno1 (udp/10.0.55.137) as highest quality interface, selected for automatic interface.
average rate: 5.619
 min: 0.109s max: 0.207s std dev: 0.03246s window: 7
average rate: 5.333
 min: 0.109s max: 0.214s std dev: 0.02783s window: 12
```

### Correction method, Check area1

Check with ros2 topic hz to see if the target topic is being output at the expected period.
Note that if play_rate is 0.5, 10\*0.5=5, which is normal.

If not, lower the play_rate argument in dlr simulation run

```shell
dlr simulation run -p perception -l play_rate:=0.2
```

### Cause 2

The topic does not appear at the beginning of the simulation, but appears at the end of the simulation.
If the ml model has not been converted to an engine in advance, the engine conversion starts when the simulation is executed, and the topic appears after the engine conversion is finished.

### Example2

```shell
[component_container_mt-52] [I] [TRT] [MemUsageChange] Init builder kernel library: CPU +894, GPU +174, now: CPU 1009, GPU 852 (MiB)
[component_container_mt-52] [I] [TRT] ----------------------------------------------------------------
[component_container_mt-52] [I] [TRT] Input filename:   /home/autoware/autoware_data/traffic_light_classifier/traffic_light_classifier_mobilenetv2_batch_6.onnx
[component_container_mt-52] [I] [TRT] ONNX IR version:  0.0.8
[component_container_mt-52] [I] [TRT] Opset version:    11
[component_container_mt-52] [I] [TRT] Producer name:    pytorch
[component_container_mt-52] [I] [TRT] Producer version: 1.13.1
[component_container_mt-52] [I] [TRT] Domain:
[component_container_mt-52] [I] [TRT] Model version:    0
[component_container_mt-52] [I] [TRT] Doc string:
[component_container_mt-52] [I] [TRT] ----------------------------------------------------------------
[component_container_mt-52] [I] [TRT] [MemUsageChange] Init CUDA: CPU +0, GPU +0, now: CPU 116, GPU 678 (MiB)

[component_container_mt-52] [I] [TRT] Applying optimizations and building TRT CUDA engine. Please wait for a few minutes...
```

### Correction method, Check area 2

Check if the terminal or console.log outputs a log like the one shown in the example.
If so, convert the engine file from onnx in advance before evaluation by driving_Log_replayer.

Start logging_simulator.launch.xml with “permission:=true” and leave it for a while.
Or, launch a launch that builds only models.

```shell
# Start logging_simulator.launch.xml and leave it for a while.
ros2 launch autoware_launch logging_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit

# launch lidar_centerpoint with build_only option
ros2 launch lidar_centerpoint lidar_centerpoint.launch.xml model_name:=centerpoint_tiny model_path:=$HOME/autoware_data/lidar_centerpoint model_param_path:=$(ros2 pkg prefix lidar_centerpoint --share)/config/centerpoint_tiny.param.yaml build_only:=true
```

## Not terminating or terminating in the middle of the process

### Cause

An exception occurs due to unintended input data, etc., and the node stops. or terminate.

### Example

The contents of the object in PERCEPTION were not as expected and an exception was output.

```shell
[perception_evaluator_node.py-115] [ERROR] [1711460672.978143229] [driving_log_replayer.perception_evaluator]: Unexpected footprint length: len(perception_object.shape.footprint.points)=2
[perception_evaluator_node.py-115] Exception in thread Thread-2 (run_func):
[perception_evaluator_node.py-115] Traceback (most recent call last):
[perception_evaluator_node.py-115]   File "/usr/lib/python3.10/threading.py", line 1016, in _bootstrap_inner
[perception_evaluator_node.py-115]     self.run()
[perception_evaluator_node.py-115]   File "/usr/lib/python3.10/threading.py", line 953, in run
[perception_evaluator_node.py-115]     self._target(*self._args, **self._kwargs)
[perception_evaluator_node.py-115]   File "/opt/ros/humble/lib/python3.10/site-packages/tf2_ros/transform_listener.py", line 95, in run_func
[perception_evaluator_node.py-115]     self.executor.spin()
[perception_evaluator_node.py-115]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 294, in spin
[perception_evaluator_node.py-115]     self.spin_once()
[perception_evaluator_node.py-115]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 739, in spin_once
[perception_evaluator_node.py-115]     self._spin_once_impl(timeout_sec)
[perception_evaluator_node.py-115]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 728, in _spin_once_impl
[perception_evaluator_node.py-115]     handler, entity, node = self.wait_for_ready_callbacks(timeout_sec=timeout_sec)
[perception_evaluator_node.py-115]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 711, in wait_for_ready_callbacks
[perception_evaluator_node.py-115]     return next(self._cb_iter)
[perception_evaluator_node.py-115]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 612, in _wait_for_ready_callbacks
[perception_evaluator_node.py-115]     raise ExternalShutdownException()
[perception_evaluator_node.py-115] rclpy.executors.ExternalShutdownException
[ros2-117] [INFO] [1711460673.168213400] [rosbag2_recorder]: Subscribed to topic '/driving_log_replayer/marker/results'
[ros2-117] [INFO] [1711460673.174638594] [rosbag2_recorder]: Subscribed to topic '/driving_log_replayer/marker/ground_truth'
[simple_object_merger_node-69] [INFO] [1711460673.191825620] [sensing.radar.simple_object_merger]: waiting for object msg...
[perception_evaluator_node.py-115] Traceback (most recent call last):
[perception_evaluator_node.py-115]   File "/home/autoware/autoware.proj/install/driving_log_replayer/lib/driving_log_replayer/perception_evaluator_node.py", line 336, in <module>
[perception_evaluator_node.py-115]     main()
[perception_evaluator_node.py-115]   File "/home/autoware/autoware.proj/install/driving_log_replayer/local/lib/python3.10/dist-packages/driving_log_replayer/evaluator.py", line 448, in wrapper
[perception_evaluator_node.py-115]     rclpy.shutdown()
[perception_evaluator_node.py-115]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/__init__.py", line 126, in shutdown
[perception_evaluator_node.py-115]     _shutdown(context=context)
[perception_evaluator_node.py-115]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/utilities.py", line 58, in shutdown
[perception_evaluator_node.py-115]     return context.shutdown()
[perception_evaluator_node.py-115]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/context.py", line 100, in shutdown
[perception_evaluator_node.py-115]     raise RuntimeError('Context must be initialized before it can be shutdown')
[perception_evaluator_node.py-115] RuntimeError: Context must be initialized before it can be shutdown
[perception_evaluator_node.py-115] The following exception was never retrieved: Expected BOUNDING_BOX, but got polygon, which should have footprint.
```

### Correction method, Check area

Search the terminal you started or console.log for the string of `evaluator` to see if an exception is output as shown in the example.
