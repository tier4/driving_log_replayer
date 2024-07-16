# 認識機能の評価(カメラ)

Autoware の認識機能(perception)の認識結果から mAP(mean Average Precision)などの指標を計算して性能を評価する。

perception モジュールを起動して出力される perception の topic を評価用ライブラリに渡して評価を行う。

## 事前準備

perception では、機械学習の学習済みモデルを使用する。
モデルを事前に準備していないとAutowareから認識結果が出力されない。
何も評価結果が出てこない場合は、この作業が正しく出来ているか確認する。

### モデルファイルのダウンロード

モデルはAutowareのセットアップ時にダウンロードされる。
モデルのダウンロード方法は、使用しているにAutowareのバージョンによって異なるのでどちらの手法が使われているか確認する。
以下のパターンが存在する。

#### ansibleでダウンロード

スクリプト実行時に`Download artifacts? [y/N]`と出てくるので`y`を入力してエンターを押す(Autoware foundationのmainだとこちら)
<https://github.com/autowarefoundation/autoware/blob/main/ansible/roles/artifacts/tasks/main.yaml>

#### パッケージのビルド時に自動でダウンロード

少し古いAutoware.universeを使用している場合はこちら、`13b96ad3c636389b32fea3a47dfb7cfb7813cadc`のコミットハッシュまではこちらが使用される。
[tensorrt_yolox/CMakeList.txt](https://github.com/autowarefoundation/autoware.universe/blob/13b96ad3c636389b32fea3a47dfb7cfb7813cadc/perception/tensorrt_yolox/CMakeLists.txt#L65-L68)

### モデルファイルの変換

ダウンロードした onnx ファイルはそのまま使用するのではなく、TensorRT の engine ファイルに変換して利用する。
変換用のコマンドが用意されているので、autoware のワークスペースを source してコマンドを実行する。

`$HOME/autoware`にautowareをインストールしたとして説明する。

```shell
source $HOME/autoware/install/setup.bash
ros2 launch tensorrt_yolox yolox.launch.xml use_decompress:=false build_only:=true
```

#### ansibleでダウンロード

以下のファイルが出力される。

```shell
$HOME/autoware_data/tensorrt_yolox/yolox-sPlus-T4-960x960-pseudo-finetune.EntropyV2-int8-batch1.engine
```

#### パッケージのビルド時に自動でダウンロード

```shell
$HOME/autoware/install/tensorrt_yolox/share/tensorrt_yolox/data/yolox-sPlus-T4-960x960-pseudo-finetune.EntropyV2-int8-batch1.engine
```

### (PC1台で評価する場合)launchファイルの修正

PC 一台で評価するには、launch をいじって、カメラの認識結果を出力するように変更する必要がある。
以下のように、launch を変更する。

```shell
❯ vcs diff src/
.................................
diff --git a/launch/tier4_perception_launch/launch/object_recognition/detection/camera_lidar_fusion_based_detection.launch.xml b/launch/tier4_perception_launch/launch/object_recognition/detection/camera_lidar_fusion_based_detection.launch.xml
index 9ca8ea3df..a35e8d00f 100644
--- a/launch/tier4_perception_launch/launch/object_recognition/detection/camera_lidar_fusion_based_detection.launch.xml
+++ b/launch/tier4_perception_launch/launch/object_recognition/detection/camera_lidar_fusion_based_detection.launch.xml
@@ -30,6 +30,14 @@
   <arg name="remove_unknown" default="true"/>
   <arg name="trust_distance" default="30.0"/>

+  <group>
+    <include file="$(find-pkg-share tensorrt_yolox)/launch/yolox.launch.xml" />
+  </group>
+
+  <group>
+    <include file="$(find-pkg-share bytetrack)/launch/bytetrack.launch.xml" />
+  </group>
+
   <!-- Jetson AGX -->
   <!-- <include file="$(find-pkg-share tensorrt_yolo)/launch/yolo.launch.xml">
     <arg name="image_raw0" value="$(var image_raw0)"/>
diff --git a/launch/tier4_perception_launch/launch/perception.launch.xml b/launch/tier4_perception_launch/launch/perception.launch.xml
index 0a2ef57f6..9a9b06379 100644
--- a/launch/tier4_perception_launch/launch/perception.launch.xml
+++ b/launch/tier4_perception_launch/launch/perception.launch.xml
@@ -33,7 +33,7 @@
   <arg name="camera_info6" default="/sensing/camera/camera6/camera_info"/>
   <arg name="image_raw7" default="/sensing/camera/camera7/image_rect_color"/>
   <arg name="camera_info7" default="/sensing/camera/camera7/camera_info"/>
-  <arg name="image_number" default="6" description="choose image raw number(0-7)"/>
+  <arg name="image_number" default="1" description="choose image raw number(0-7)"/>
   <arg name="use_vector_map" default="true" description="use vector map in prediction"/>
   <arg name="use_pointcloud_map" default="true" description="use pointcloud map in detection"/>
   <arg name="use_object_filter" default="true" description="use object filter"/>
diff --git a/perception/tensorrt_yolox/launch/yolox.launch.xml b/perception/tensorrt_yolox/launch/yolox.launch.xml
index b697b1f50..b9cb53102 100644
--- a/perception/tensorrt_yolox/launch/yolox.launch.xml
+++ b/perception/tensorrt_yolox/launch/yolox.launch.xml
@@ -1,7 +1,7 @@
 <?xml version="1.0"?>
 <launch>
   <arg name="input/image" default="/sensing/camera/camera0/image_rect_color"/>
-  <arg name="output/objects" default="/perception/object_recognition/detection/rois0"/>
+  <arg name="output/objects_yolox" default="/perception/object_recognition/detection/rois0"/>
   <arg name="model_name" default="yolox-tiny"/>
   <arg name="model_path" default="$(find-pkg-share tensorrt_yolox)/data"/>
   <arg name="score_threshold" default="0.35"/>
@@ -16,7 +16,7 @@

   <node pkg="tensorrt_yolox" exec="tensorrt_yolox_node_exe" name="tensorrt_yolox" output="screen">
     <remap from="~/in/image" to="$(var input/image)"/>
-    <remap from="~/out/objects" to="$(var output/objects)"/>
+    <remap from="~/out/objects" to="$(var output/objects_yolox)"/>
     <param name="score_threshold" value="$(var score_threshold)"/>
     <param name="nms_threshold" value="$(var nms_threshold)"/>
     <param name="model_path" value="$(var model_path)/$(var model_name).onnx"/>
```

## 評価方法

launch を立ち上げると以下のことが実行され、評価される。

1. launch で評価ノード(`perception_2d_evaluator_node`)と `logging_simulator.launch`、`ros2 bag play`コマンドを立ち上げる
2. bag から出力されたセンサーデータを autoware が受け取って、カメラデータを出力し、perception モジュールが認識を行う
3. 評価ノードが/perception/object_recognition/detection{/tracked}/rois{camera_no} を subscribe して、コールバックで perception_eval の関数を用いて評価し結果をファイルに記録する
4. bag の再生が終了すると自動で launch が終了して評価が終了する

## 評価結果

topic の subscribe 1 回につき、以下に記述する判定結果が出力される。

### 正常

perception_eval の評価関数を実行して以下の条件を満たすとき

1. frame_result.pass_fail_result に object が最低 1 つ入っている (`tp_object_results != [] and fp_object_results != [] and fn_objects != []`)
2. 評価失敗のオブジェクトが 0 個 (`frame_result.pass_fail_result.get_fail_object_num() == 0`)

### 異常

正常の条件を満たさない場合

### 評価スキップ

以下の場合に、評価をせずに評価が飛ばされた回数のカウント(FrameSkip)を1足す処理のみ行う

- 受信したobjectのヘッダー時刻の前後75msec以内に真値が存在しない場合

## 評価ノードが使用する Topic 名とデータ型

Subscribed topics:

| topic 名                                                         | データ型                                             |
| ---------------------------------------------------------------- | ---------------------------------------------------- |
| /perception/object_recognition/detection/rois{camera_no}         | tier4_perception_msgs/msg/DetectedObjectsWithFeature |
| /perception/object_recognition/detection/tracked/rois{camera_no} | tier4_perception_msgs/msg/DetectedObjectsWithFeature |

Published topics:

| topic 名 | データ型 |
| -------- | -------- |
| -        | -        |

## logging_simulator.launch に渡す引数

autoware の処理を軽くするため、評価に関係のないモジュールは launch の引数に false を渡すことで無効化する。以下を設定している。

- localization: false
- planning: false
- control: false
- sensing: false / true (デフォルト false、シナリオの `LaunchSensing` キーで t4_dataset 毎に指定する)
- perception_mode: camera_lidar_fusion

**注:アノーテション時とシミュレーション時で自己位置を合わせたいので bag に入っている tf を使い回す。そのため localization は無効である。**

## 依存ライブラリ

認識機能の評価は[perception_eval](https://github.com/tier4/autoware_perception_evaluation)に依存している。

### 依存ライブラリとの log_evaluator の役割分担

log_evaluator が ROS との接続部分を担当し、perception_eval がデータセットを使って実際に評価する部分を担当するという分担になっている。
perception_eval は ROS 非依存のライブラリなので、ROS のオブジェクトを受け取ることができない。
また、timestamp が ROS ではナノ秒、t4_dataset は `nuScenes` をベースしているためミリ秒が採用されている。
このため、ライブラリ使用前に適切な変換が必要となる。

log_evaluator は、autoware の perception モジュールから出力された topic を subscribe し、perception_eval で定義されている class に合わせたデータ形式に変換して渡す。
また、perception_eval から返ってくる評価結果の ROS の topic で publish し可視化する部分も担当する。

perception_eval は、log_evaluator から渡された検知結果と GroundTruth を比較して指標を計算し、結果を出力する部分を担当する。

## simulation

シミュレーション実行に必要な情報を述べる。

### 入力 rosbag に含まれるべき topic

t4_dataset で必要なトピックが含まれていること

車両の ECU の CAN と、使用している sensor の topic が必要
以下は例であり、違うセンサーを使っている場合は適宜読み替える。

LiDAR が複数ついている場合は、搭載されているすべての LiDAR の packets を含める。
/sensing/lidar/concatenated/pointcloud は、シナリオの LaunchSensing: false の場合に使用される。

CAMERA が複数ついている場合は、搭載されているすべての camera_info と image_rect_color_compressed を含める

| topic 名                                             | データ型                                     |
| ---------------------------------------------------- | -------------------------------------------- |
| /gsm8/from_can_bus                                   | can_msgs/msg/Frame                           |
| /sensing/camera/camera\*/camera_info                 | sensor_msgs/msg/CameraInfo                   |
| /sensing/camera/camera\*/image_rect_color/compressed | sensor_msgs/msg/CompressedImage              |
| /sensing/gnss/ublox/fix_velocity                     | geometry_msgs/msg/TwistWithCovarianceStamped |
| /sensing/gnss/ublox/nav_sat_fix                      | sensor_msgs/msg/NavSatFix                    |
| /sensing/gnss/ublox/navpvt                           | ublox_msgs/msg/NavPVT                        |
| /sensing/imu/tamagawa/imu_raw                        | sensor_msgs/msg/Imu                          |
| /sensing/lidar/concatenated/pointcloud               | sensor_msgs/msg/PointCloud2                  |
| /sensing/lidar/\*/velodyne_packets                   | velodyne_msgs/VelodyneScan                   |
| /tf                                                  | tf2_msgs/msg/TFMessage                       |

CAN の代わりに vehicle の topic を含めても良い。

| topic 名                                             | データ型                                       |
| ---------------------------------------------------- | ---------------------------------------------- |
| /sensing/camera/camera\*/camera_info                 | sensor_msgs/msg/CameraInfo                     |
| /sensing/camera/camera\*/image_rect_color/compressed | sensor_msgs/msg/CompressedImage                |
| /sensing/gnss/ublox/fix_velocity                     | geometry_msgs/msg/TwistWithCovarianceStamped   |
| /sensing/gnss/ublox/nav_sat_fix                      | sensor_msgs/msg/NavSatFix                      |
| /sensing/gnss/ublox/navpvt                           | ublox_msgs/msg/NavPVT                          |
| /sensing/imu/tamagawa/imu_raw                        | sensor_msgs/msg/Imu                            |
| /sensing/lidar/concatenated/pointcloud               | sensor_msgs/msg/PointCloud2                    |
| /sensing/lidar/\*/velodyne_packets                   | velodyne_msgs/VelodyneScan                     |
| /tf                                                  | tf2_msgs/msg/TFMessage                         |
| /vehicle/status/control_mode                         | autoware_vehicle_msgs/msg/ControlModeReport    |
| /vehicle/status/gear_status                          | autoware_vehicle_msgs/msg/GearReport           |
| /vehicle/status/steering_status                      | autoware_vehicle_msgs/SteeringReport           |
| /vehicle/status/turn_indicators_status               | autoware_vehicle_msgs/msg/TurnIndicatorsReport |
| /vehicle/status/velocity_status                      | autoware_vehicle_msgs/msg/VelocityReport       |

### 入力 rosbag に含まれてはいけない topic

| topic 名 | データ型                |
| -------- | ----------------------- |
| /clock   | rosgraph_msgs/msg/Clock |

clock は、ros2 bag play の--clock オプションによって出力しているので、bag 自体に記録されていると 2 重に出力されてしまうので bag には含めない

## evaluation

評価に必要な情報を述べる。

### シナリオフォーマット

ユースケース評価とデータベース評価の 2 種類の評価がある。
ユースケースは 1 個のデータセットで行う評価で、データベースは複数のデータセットを用いて、各データセット毎の結果の平均を取る評価である。

データベース評価では、キャリブレーション値の変更があり得るので vehicle_id をデータセット毎に設定出来るようにする。
また、Sensing モジュールを起動するかどうかの設定も行う。

[サンプル](https://github.com/tier4/log_evaluator/blob/main/sample/perception_2d/scenario.ja.yaml)参照

### 評価結果フォーマット

[サンプル](https://github.com/tier4/log_evaluator/blob/main/sample/perception_2d/result.json)参照

perception では、シナリオに指定した条件で perception_eval が評価した結果を各 frame 毎に出力する。
全てのデータを流し終わったあとに、最終的なメトリクスを計算しているため、最終行だけ、他の行と形式が異なる。

以下に、各フレームのフォーマットとメトリクスのフォーマットを示す。
**注:結果ファイルフォーマットで解説済みの共通部分については省略する。**

各フレームのフォーマット

```json
{
  "Frame": {
    "CameraType": "評価したカメラ",
    "FrameName": "評価に使用したt4_datasetのフレーム番号",
    "FrameSkip": "objectの評価を依頼したがdatasetに75msec以内の真値がなく評価を飛ばされた回数",
    "PassFail": {
      "Result": { "Total": "Success or Fail", "Frame": "Success or Fail" },
      "Info": {
        "TP": "TPと判定された数",
        "FP": "FPと判定された数",
        "FN": "FNと判定された数"
      }
    }
  }
}
```

メトリクスデータのフォーマット

```json
{
  "Frame": {
    "FinalScore": {
      "Score": {
        "TP": {
          "ALL": "すべてのラベルのTP率",
          "label0": "label0のTP率",
          "label1": "label1のTP率"
        },
        "FP": {
          "ALL": "すべてのラベルのFP率",
          "label0": "label0のFP率",
          "label1": "label1のFP率"
        },
        "FN": {
          "ALL": "すべてのラベルのFN率",
          "label0": "label0のFN率",
          "label1": "label1のFN率"
        },
        "TN": {
          "ALL": "すべてのラベルのTN率",
          "label0": "label0のTN率",
          "label1": "label1のTN率"
        },
        "AP(Center Distance)": {
          "ALL": "すべてのラベルのAP率(Center Distance)",
          "label0": "label0のAP率(Center Distance)",
          "label1": "label1のAP率(Center Distance)"
        },
        "APH(Center Distance)": {
          "ALL": "すべてのラベルのAPH率(Center Distance)",
          "label0": "label0のAPH率(Center Distance)",
          "label1": "label1のAPH率(Center Distance)"
        },
        "AP(IoU 2D)": {
          "ALL": "すべてのラベルのAP率(IoU 2D)",
          "label0": "label0のAP率(IoU 2D)",
          "label1": "label1のAP率(IoU 2D)"
        },
        "APH(IoU 2D)": {
          "ALL": "すべてのラベルのAPH率(IoU 2D)",
          "label0": "label0のAPH率(IoU 2D)",
          "label1": "label1のAPH率(IoU 2D)"
        }
      },
      "ConfusionMatrix": {
        "label0(真値)": {
          "label0(予測値)": "値",
          "label1(予測値)": "値"
        },
        "label1(真値)": {
          "label0(予測値)": "値",
          "label1(予測値)": "値"
        }
      }
    }
  }
}
```

### pickle ファイル

データベース評価では、複数の bag を再生する必要があるが、ROS の仕様上、1 回の launch で、複数の bag を利用することは出来ない。
1 つの bag、すなわち 1 つの t4_dataset に対して launch を 1 回叩くことなるので、データベース評価では、含まれるデータセットの数だけ launch を実行する必要がある。

データベース評価は 1 回の launch で評価できないため、perception では、result.jsonl の他に scene_result.pkl というファイルを出力する。
pickle ファイルは python のオブジェクトをファイルとして保存したものであり、perception_eval の PerceptionEvaluationManager.frame_results を保存している。
pickle ファイルに記録した object をすべて読み込み、dataset の平均の指標を出力することでデータセット評価が行える。

### データベース評価の結果ファイル

シナリオに複数の dataset を記述したデータベース評価の場合には、結果出力先ディレクトリに database_result.json というファイルが出力される。

形式は[メトリクスのフォーマット](#評価結果フォーマット) と同じ
