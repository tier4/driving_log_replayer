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

評価対象のtopicを出力するノードがlaunchから起動されていない(launchファイルのtrue/falseの値)

### 例1-2

起動してから死んでいる

### 例1-3

cuda, cuDNN, TensorRTの不整合が発生していて、perceptionの認識結果が出てこない

### 修正方法、確認箇所1-1

aaa

### 修正方法、確認箇所1-2

aaa

### 修正方法、確認箇所1-3

aaa

### 原因2

Autowareから評価対象のtopicが出力されているがsubscribeできていない。

### 例2-1

QoSの不一致で取得できていない

```shell
[component_container_mt-13] [WARN 1633081042.510824100] [localization.util.random_downsample_filter]: New subscription discovered on topic '/localization/util/downsample/pointcloud', requesting incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY
[component_container_mt-19] [WARN 1633081042.593132498] [sensing.lidar.occupancy_grid_map_outlier_filter]: New subscription discovered on topic '/sensing/lidar/no_ground/pointcloud', requesting incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY
[component_container_mt-19] [WARN 1633081042.597116410] [sensing.lidar.concatenate_data]: New subscription discovered on topic '/sensing/lidar/concatenated/pointcloud', requesting incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY
```

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

### 修正方法、確認箇所2-1

aaa

### 修正方法、確認箇所2-2

Autowareとdriving_log_replayerno


## 評価数が異常に少ない

PCの性能不足でAutowareが出力するtopic数が少ない
PCの性能不足でsubscriberがcallbackを処理仕切る前に次のtopicが来て、topicが捨てられる
事前にml modelをengineに変換していない場合

## 終了しない

rosbag playは終了しているのに、ずっとrvizはでっぱなし。コンソールのログからも動いてるように見えない。
途中で実行時エラーでノードが死んでいる可能性がある。ノードが正常に動作していないと
input_bagのパスの指定ミスでbag playが実行できずにずっと止まる

## 途中で終了する

終了しない場合と同様で途中で実行時エラーでノードが死んでいる。

## 原因解析

driving_log_replayerのcliを使った

### run.bash


### console.log

コンソールに表示されているログを保存したもの
エラーが発生していないかを確認する


rosのlogging機能を使って出力されているので、以下のような形式で出力されている。

```shell
[重大度] [時刻] [ロガー名]: メッセージ
```

ローカル環境で.bashrcにRCUTILS_CONSOLE_OUTPUT_FORMATを設定している場合は、設定したformatになる。

===== evaluatorで検索する
logsimの評価ノードは、「ユースケース名_evaluator_node」という名前になっている。
なので、logsimの評価ノードが出しているメッセージを抽出するにはevaluatorという文字を検索すると該当のメッセージを見つけることが出来る。

link:./sample/debug/logsim_runtime_error.log[実行時エラーになる例]
をevaluatorで検索すると、obstacle_detection_evaluator_nodeがjsonのparseに失敗しており、obstacle_detection_evaluator_nodeが死んでlaunchが終了していることがわかる。

```shell
[obstacle_detection_evaluator_node-38] [ERROR] [1640228106.306940806] [logsim.obstacle_detection_evaluator]: detection parse error
・・・
[ERROR] [obstacle_detection_evaluator_node-38]: process has died [pid 486, exit code 1, cmd '/home/autoware/autoware.proj/install/logsim/lib/logsim/obstacle_detection_evaluator_node --ros-args -r __node:=obstacle_detection_evaluator -r __ns:=/logsim --params-file /tmp/launch_params_i_9h9mk1 --params-file /tmp/launch_params_mq4cddau'].
[INFO] [launch.user]: shutdown launch
```

よって、parse errorになった原因であるdetections.jsonを直す必要があるということがわかる。

===== ERRORで検索する
logsimのモジュールはautowareから評価対象のtopicをsubscribeして動いている。
なので、autowareが必要なtopicを出していない場合はlogsimのERRORとしては出てこないケースがある。

link:./sample/debug/process_died.log[autoware側のプロセスが死んでtopicが出ない例]
は点群が一切出なかったケースのログの一部であるが、ERRORで検索するとpointcloud_preprocessorが死んでいる。
autoware側に問題があることがわかり、後のデバッグでスレッドプールを使い切って死んでいることがわかった。

```shell
[ERROR] [component_container_mt-18]: process has died [pid 95, exit code -6, cmd '/opt/ros/galactic/lib/rclcpp_components/component_container_mt --ros-args -r __node:=pointcloud_preprocessor_container -r __ns:=/sensing/lidar/pointcloud_preprocessor --params-file /tmp/launch_params_rh_9gxcs'].
```

===== QoSで検索する
ROS2ではリンクにある通り、publisherとsubscriber間でQoS設定に互換性がないとtopicが出ていても受信されずに捨てられる。
link:https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html[QoS設定]
厄介なことにこれはシステム的にはERRORではなくWARNINGであるため、ERRORで検索しても引っかからないので見逃しがちである。

AutowareEvaluationDashboard上でbagを再生したが何も表示されず、bagをDLしてきてros2 bag infoしたが、bagに必要なtopicが入っていない場合などは、QoSで調べてみると良い。

link:./sample/debug/incompatible_qos.log[QoS非互換でtopicをレコード出来ない例]
をQoSで検索すると、/localization/util/downsample/pointcloudをレコードしようとしたが、QoS非互換で受信出来てないということがわかる。

```shell
[ros2-41] [WARN 1633081042.154102891] [rosbag2_recorder]: New publisher discovered on topic '/localization/util/downsample/pointcloud', offering incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY
```

なので、この場合は、ros2 bag info /localization/util/downsample/pointcloud -vして、送信側のQoS設定を調べて受信側で互換性が保てる設定を追加することが必要だとわかった。


### result.jsonl

### resutl_bag

ros2 bag info resutl_bagを実行して、評価対象のtopicが記録されているか調べる。
ここでtopicが記録されていなかったら、Autowareが出力していないか、QoSの不一致で捨てられている
