# トラブルシューティング

うまく行かないときに確認する。

## Autowareが起動しない

### 原因1

シナリオで指定したsensor_model、vehicle_model、vehicle_idが利用するAutowareのワークスペースに含まれていない。

### 例1

```shell
❯ dlr simulation run -p localization
[INFO] [launch]: All log files can be found below /home/hyt/.ros/log/2024-06-07-12-37-19-365597-dpc2405001-1360746
[INFO] [launch]: Default logging verbosity is set to INFO
1717731451.040883 [77]       ros2: determined eno1 (udp/10.0.55.137) as highest quality interface, selected for automatic interface.
[ERROR] [launch]: Caught exception in launch (see debug for traceback): executed command failed. Command: xacro /home/hyt/ros_ws/pilot-auto/install/tier4_vehicle_launch/share/tier4_vehicle_launch/urdf/vehicle.xacro vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit config_dir:=/home/hyt/ros_ws/pilot-auto/install/individual_params/share/individual_params/config/default/sample_sensor_kit
Captured stderr output: error: package not found: "package 'sample_sensor_kit_description' not found, searching: ...
...
```

### 修正方法、確認箇所1

プロファイルで指定しているautoware_pathにシナリオで指定した、sensor_model、vehicle_model、vehicle_idが存在するか確認する。

### 原因2

cliのバージョンがdriving_log_replayerのバージョンと一致していない。

### 例2

```shell
❯ dlr simulation run -p yabloc -l play_rate:=0.5
Usage: dlr simulation run [OPTIONS]
Try 'dlr simulation run -h' for help.

Error: No such option: -l
```

### 修正方法、確認箇所2

installされているdriving_log_replayerのpackage.xmlのversionタグの値と、cliが出力するversionが一致しているか確認する。

```shell
❯ dlr --version
1.18.0
```

## Autoware起動後すぐに終了してしまう

### 原因1

シナリオフォーマットが不正

### 例1

コンソールに以下のようなメッセージが出力される。
また、同様の内容がresult.jsonlに出力される

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

### 修正方法、確認箇所1

result.jsonlに何が問題か出力されているので指示通り治す。
例だと、UseCaseFormatVersionは1.2.0か1.3.0である必要があるのに、1.0.0なので利用できない。
古いフォーマットを利用しているので、リポジトリのsampleディレクトリにあるシナリオを参考に修正する。

## 評価結果がNoDataとなる

### 原因1

Autowareから評価対象のtopicが出力されていない

### 例1-1

評価対象のtopicを出力するノードがlaunchから起動されていない
launchファイルのtrue/falseの値の設定間違い

### 修正方法、確認箇所1-1

評価対象のtopicをドキュメントから探して、topic infoしてpublisherが存在するか確認する
Publisher count: 0の場合は、そもそも起動できてない可能性が高い。

```shell
❯ ros2 topic info /perception/traffic_light_recognition/traffic_signals -v
Type: autoware_auto_perception_msgs/msg/TrafficSignalArray

Publisher count: 1 <- 0じゃないことを確認する

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

### 例1-2

起動してから死んでいる

### 修正方法、確認箇所1-2

起動したターミナル、もしくは、出力先ディレクトリにあるconsole.logをERRORで検索する。

以下は、点群が一切出なかったケースのログの一部であるが、ERRORで検索するとpointcloud_preprocessorが死んでいる。
topicを出力するcomponent_containerがエラーを吐いてないか確認する。

```shell
[ERROR] [component_container_mt-18]: process has died [pid 95, exit code -6, cmd '/opt/ros/galactic/lib/rclcpp_components/component_container_mt --ros-args -r __node:=pointcloud_preprocessor_container -r __ns:=/sensing/lidar/pointcloud_preprocessor --params-file /tmp/launch_params_rh_9gxcs'].
```

### 例1-3

cuda, cuDNN, TensorRTの不整合が発生していて、perceptionの認識結果が出てこない。
apt upgradeでnvidia driverが更新されたときに発生することがある。

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

### 修正方法、確認箇所1-3

cudaErrorInvalidDevice: invalid device ordinalが出てないか確認する。
出ていたら、nvidia-driver, cuda, cuDNN, TensorRTを再インストールする。

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

### 原因2

Autowareから評価対象のtopicが出力されているがsubscribeできていない。

### 例2-1

QoSの不一致で取得できていない

```shell
[component_container_mt-13] [WARN 1633081042.510824100] [localization.util.random_downsample_filter]: New subscription discovered on topic '/localization/util/downsample/pointcloud', requesting incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY
[component_container_mt-19] [WARN 1633081042.593132498] [sensing.lidar.occupancy_grid_map_outlier_filter]: New subscription discovered on topic '/sensing/lidar/no_ground/pointcloud', requesting incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY
[component_container_mt-19] [WARN 1633081042.597116410] [sensing.lidar.concatenate_data]: New subscription discovered on topic '/sensing/lidar/concatenated/pointcloud', requesting incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY
```

### 修正方法、確認箇所2-1

起動したターミナルもしくは、console.logをQoSで検索する。
厄介なことにこれはシステム的にはERRORではなくWARNINGであるため、ERRORで検索しても引っかからないので見逃しがちである。

Autowareのバージョンとdriving_log_replayerのバージョンが対応しているか確認する。
Autoware Foundationのmainとdriving_log_replayerのmainを使用して、この問題が発生している場合、Autowareの更新にdriving_log_replayerが追従できていないので、issueで報告してください。

### 例2-2

メッセージ型の不一致で取得できていない。
Autowareが出力する型がdriving_log_replayerが期待している型と異なっているために発生する。

2024年6月にautoware_auto_msgからautoware_msgに変更された。これによって、autowareの本体のバージョンとdriving_log_replayerのバージョンが対応していないとこのメッセージがでる。

```shell
[ros2-67] [ERROR] [1717610261.542314281] [ROSBAG2_TRANSPORT]: Topic '/perception/object_recognition/tracking/objects' has more than one type associated. Only topics with one type are supported
[ros2-67] [ERROR] [1717610261.721551659] [ROSBAG2_TRANSPORT]: Topic '/perception/object_recognition/tracking/objects' has more than one type associated. Only topics with one type are supported
[ros2-67] [ERROR] [1717610261.903905941] [ROSBAG2_TRANSPORT]: Topic '/perception/object_recognition/tracking/objects' has more than one type associated. Only topics with one type are supported
[ros2-67] [ERROR] [1717610262.084860123] [ROSBAG2_TRANSPORT]: Topic '/perception/object_recognition/tracking/objects' has more than one type associated. Only topics with one type are supported
[ros2-67] [ERROR] [1717610262.263855979] [ROSBAG2_TRANSPORT]: Topic '/perception/object_recognition/tracking/objects' has more than one type associated. Only topics with one type are supported
[ros2-67] [ERROR] [1717610262.442275790] [ROSBAG2_TRANSPORT]: Topic '/perception/object_recognition/tracking/objects' has more than one type associated. Only topics with one type are supported
```

### 修正方法、確認箇所2-2

大きな機能変更がある場合、ReleaseNotes.mdにAutoware側に必要な機能(PR番号等)が記載してある。
利用するAutowareに必要な機能が入っているか確認する。

Autoware Foundationのmainとdriving_log_replayerのmainを使用して、この問題が発生している場合、Autowareの更新にdriving_log_replayerが追従できていないので、issueで報告してください。

## 評価数が異常に少ない

### 原因1

PCの性能不足でAutowareが所定の周期(点群なら10Hz)でtopicをpublish出来ていない。

### 例1

```shell
❯ ros2 topic hz /perception/obstacle_segmentation/pointcloud
1718083964.779455 [77]       ros2: determined eno1 (udp/10.0.55.137) as highest quality interface, selected for automatic interface.
average rate: 5.619
 min: 0.109s max: 0.207s std dev: 0.03246s window: 7
average rate: 5.333
 min: 0.109s max: 0.214s std dev: 0.02783s window: 12
```

### 修正方法、確認箇所1

対象のtopicが期待通りの周期で出力されているかros2 topic hzで確認する。
例では5Hz程度になっているが、play_rateが0.5であれば10\*0.5=5で正常であることに注意。

rate 1.0時の想定レート \* play_rateが期待通りかどうかを調べる。

出ていない場合は、実行時のplay_rate引数でbag playのrateを変更できるので、rateを低くしてみる。

```shell
dlr simulation run -p perception -l play_rate:=0.2
```

### 原因2

topicがsimulation開始時点では出てこずに、simulationの終わり頃にようやく出てくる。
事前にml modelをengineに変換していない場合、simulation実行時にengine変換が始まり、engine変換が終わったあとにtopicが出てくる。

### 例2

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

### 修正方法、確認箇所2

実行したターミナルまたはconsole.logに例に示したようなログが出力されていて、simulation実行時にonnxからengineファイルの生成が行わているか確認する。
出ている場合は、driving_Log_replayerで評価を行う前に事前にonnxからengineの変換を行う。

logging_simulator.launch.xmlをpercepiton:=trueで起動してしばらく放置する。
または、モデルだけビルドするlaunchを起動する。

```shell
# 起動してしばらく放置する
ros2 launch autoware_launch logging_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit

# lidar_centerpoint を build_onlyでlaunchを起動
ros2 launch lidar_centerpoint lidar_centerpoint.launch.xml model_name:=centerpoint_tiny model_path:=$HOME/autoware_data/lidar_centerpoint model_param_path:=$(ros2 pkg prefix lidar_centerpoint --share)/config/centerpoint_tiny.param.yaml build_only:=true
```

## 終了しない、途中で終了する

### 原因

意図しない入力データなどにより例外が発生して、ノードが止まる。または終了する。

### 例

perceptionのobjectの中身が想定した通りになっておらずに例外が出力された。

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

### 修正方法、確認箇所

起動したターミナルか、console.logをevalutorの文字列で検索して、例のように例外が出力されていないか確認する。
