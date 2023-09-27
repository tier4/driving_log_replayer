# 認識機能の評価

Autoware の認識機能(perception)の認識結果から mAP(mean Average Precision)などの指標を計算して性能を評価する。

perception モジュールを起動して出力される perception の topic を評価用ライブラリに渡して評価を行う。

デフォルトではperception_mode = lidarで起動されるので、cameraやradarを対象としたい場合は、perception.launch.pyにperception_modeを渡して変更する。

## 事前準備

perception では、機械学習の学習済みモデルを使用する。
モデルはセットアップ時に自動的にダウンロードされる。
[lidar_centerpoint/CMakeList.txt](https://github.com/autowarefoundation/autoware.universe/blob/main/perception/lidar_centerpoint/CMakeLists.txt#L112-L118)

また、ダウンロードした onnx ファイルはそのまま使用するのではなく、TensorRT の engine ファイルに変換して利用する。
変換用のコマンドが用意されているので、autoware のワークスペースを source してコマンドを実行する。
変換コマンドが終了すると、engine ファイルが出力されているので[perception.launch.xml](https://github.com/autowarefoundation/autoware.universe/blob/main/launch/tier4_perception_launch/launch/perception.launch.xml#L12-L14)
に記載のディレクトリを確認する。

autowarefoundation の autoware.universe を使用した場合の例を以下に示す。

```shell
# $HOME/autowareにautowareをインストールした場合
source ~/autoware/install/setup.bash
ros2 launch lidar_centerpoint lidar_centerpoint.launch.xml build_only:=true

# ~/autoware/install/lidar_centerpoint/share/lidar_centerpoint/dataに以下の２つが出力される
# pts_backbone_neck_head_centerpoint_tiny.engine
# pts_voxel_encoder_centerpoint_tiny.engine
```

## 評価方法

`perception.launch.py` を使用して評価する。
launch を立ち上げると以下のことが実行され、評価される。

1. launch で評価ノード(`perception_evaluator_node`)と `logging_simulator.launch`、`ros2 bag play`コマンドを立ち上げる
2. bag から出力されたセンサーデータを autoware が受け取って、点群データを出力し、perception モジュールが認識を行う
3. 評価ノードが/perception/object_recognition/{detection, tracking}/objects を subscribe して、コールバックで perception_eval の関数を用いて評価し結果をファイルに記録する
4. bag の再生が終了すると自動で launch が終了して評価が終了する

## 評価結果

topic の subscribe 1 回につき、以下に記述する判定結果が出力される。

### 正常

perception_eval の評価関数を実行して以下の条件を満たすとき

1. frame_result.pass_fail_result に object が最低 1 つ入っている (`tp_object_results != [] and fp_object_results != [] and fn_objects != []`)
2. 評価失敗のオブジェクトが 0 個 (`frame_result.pass_fail_result.get_fail_object_num() == 0`)

### 異常

正常の条件を満たさない場合

## 評価ノードが使用する Topic 名とデータ型

Subscribed topics:

| topic 名                                         | データ型                                          |
| ------------------------------------------------ | ------------------------------------------------- |
| /perception/object_recognition/detection/objects | autoware_auto_perception_msgs/msg/DetectedObjects |
| /perception/object_recognition/tracking/objects  | autoware_auto_perception_msgs/msg/TrackedObjects  |

Published topics:

| topic 名                                  | データ型                             |
| ----------------------------------------- | ------------------------------------ |
| /driving_log_replayer/marker/ground_truth | visualization_msgs::msg::MarkerArray |
| /driving_log_replayer/marker/results      | visualization_msgs::msg::MarkerArray |

## logging_simulator.launch に渡す引数

autoware の処理を軽くするため、評価に関係のないモジュールは launch の引数に false を渡すことで無効化する。以下を設定している。

- localization: false
- planning: false
- control: false
- sensing: false / true (デフォルト false、シナリオの `LaunchSensing` キーで t4_dataset 毎に指定する)

**注:アノーテション時とシミュレーション時で自己位置を合わせたいので bag に入っている tf を使い回す。そのため localization は無効である。**

## 依存ライブラリ

認識機能の評価は[perception_eval](https://github.com/tier4/autoware_perception_evaluation)に依存している。

### 依存ライブラリとの driving_log_replayer の役割分担

driving_log_replayer が ROS との接続部分を担当し、perception_eval がデータセットを使って実際に評価する部分を担当するという分担になっている。
perception_eval は ROS 非依存のライブラリなので、ROS のオブジェクトを受け取ることができない。
また、timestamp が ROS ではナノ秒、t4_dataset は `nuScenes` をベースしているためミリ秒が採用されている。
このため、ライブラリ使用前に適切な変換が必要となる。

driving_log_replayer は、autoware の perception モジュールから出力された topic を subscribe し、perception_eval で定義されている class に合わせたデータ形式に変換して渡す。
また、perception_eval から返ってくる評価結果の ROS の topic で publish し可視化する部分も担当する。

perception_eval は、driving_log_replayer から渡された検知結果と GroundTruth を比較して指標を計算し、結果を出力する部分を担当する。

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

CAN の代わりに vehicle の topic を含めても良い。

| topic 名                                             | データ型                                            |
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

[サンプル](https://github.com/tier4/driving_log_replayer/blob/main/sample/perception/scenario.ja.yaml)参照

### 評価結果フォーマット

[サンプル](https://github.com/tier4/driving_log_replayer/blob/main/sample/perception/result.json)参照

perception では、シナリオに指定した条件で perception_eval が評価した結果を各 frame 毎に出力する。
全てのデータを流し終わったあとに、最終的なメトリクスを計算しているため、最終行だけ、他の行と形式が異なる。

以下に、各フレームのフォーマットとメトリクスのフォーマットを示す。
**注:結果ファイルフォーマットで解説済みの共通部分については省略する。**

各フレームのフォーマット

```json
{
  "Frame": {
    "FrameName": "評価に使用したt4_datasetのフレーム番号",
    "FrameSkip": "objectの評価を依頼したがdatasetに75msec以内の真値がなく評価を飛ばされた回数",
    "PassFail": {
      "Result": "Success or Fail",
      "Info": [
        {
          "TP": "TPと判定された数",
          "FP": "FPと判定された数",
          "FN": "FNと判定された数"
        }
      ]
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
        },
        "AP(IoU 3D)": {
          "ALL": "すべてのラベルのAP率(IoU 3D)",
          "label0": "label0のAP率(IoU 3D)",
          "label1": "label1のAP率(IoU 3D)"
        },
        "APH(IoU 3D)": {
          "ALL": "すべてのラベルのAPH率(IoU 3D)",
          "label0": "label0のAPH率(IoU 3D)",
          "label1": "label1のAPH率(IoU 3D)"
        },
        "AP(Plane Distance)": {
          "ALL": "すべてのラベルのAP率(Plane Distance)",
          "label0": "label0のAP率(Plane Distance)",
          "label1": "label1のAP率(Plane Distance)"
        },
        "APH(Plane Distance)": {
          "ALL": "すべてのラベルのAPH率(Plane Distance)",
          "label0": "label0のAPH率(Plane Distance)",
          "label1": "label1のAPH率(Plane Distance)"
        }
      },
      "Error": {
        "ALL": {
          "average": {
            "x": "x座標",
            "y": "y座標",
            "yaw": "ヨー角",
            "length": "長さ",
            "width": "幅",
            "vx": "x方向の速度",
            "vy": "y方向の速度",
            "nn_plane": "最近傍面の距離"
          },
          "rms": {
            "x": "x座標",
            "y": "y座標",
            "yaw": "ヨー角",
            "length": "長さ",
            "width": "幅",
            "vx": "x方向の速度",
            "vy": "y方向の速度",
            "nn_plane": "最近傍面の距離"
          },
          "std": {
            "x": "x座標",
            "y": "y座標",
            "yaw": "ヨー角",
            "length": "長さ",
            "width": "幅",
            "vx": "x方向の速度",
            "vy": "y方向の速度",
            "nn_plane": "最近傍面の距離"
          },
          "max": {
            "x": "x座標",
            "y": "y座標",
            "yaw": "ヨー角",
            "length": "長さ",
            "width": "幅",
            "vx": "x方向の速度",
            "vy": "y方向の速度",
            "nn_plane": "最近傍面の距離"
          },
          "min": {
            "x": "x座標",
            "y": "y座標",
            "yaw": "ヨー角",
            "length": "長さ",
            "width": "幅",
            "vx": "x方向の速度",
            "vy": "y方向の速度",
            "nn_plane": "最近傍面の距離"
          }
        },
        "label0": "label0の誤差メトリクス"
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
