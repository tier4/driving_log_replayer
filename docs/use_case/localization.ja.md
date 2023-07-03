# 自己位置推定の評価

Autoware の自己位置推定(localization)が安定して動作しているかを評価する。

自己位置推定の評価では NDT の信頼度と収束性を評価する。

## 評価方法

`localization.launch.py` を使用して評価する。
launch を立ち上げると以下のことが実行され、評価される。

1. launch で評価ノード(`localization_evaluator_node`)と `logging_simulator.launch`、`ros2 bag play`コマンドを立ち上げる
2. bag から出力されたセンサーデータを autoware が受け取って、自己位置推定を行う
3. 評価ノードが topic を subscribe して、NDT の信頼度、収束性が基準を満たしているかを判定して結果をファイルに記録する
4. bag の再生が終了すると自動で launch が終了して評価が終了する

### NDT の信頼度

以下の 2 つの topic のうち、シナリオで指定した方を用いて評価する。

- /localization/pose_estimator/transform_probability
- /localization/pose_estimator/nearest_voxel_transformation_likelihood

### NDT の収束性

以下を用いて評価する

- /localization/pose_estimator/pose
- /localization/pose_twist_fusion_filter/pose

ただし収束性は NDT が収束してから評価開始とし、収束の判定は/localization/pose_estimator/transform_probability > 0 もしくは /localization/pose_estimator/nearest_voxel_transformation_likelihood > 0 を用いる。

### NDT の可用性

下記の出力が定期的に出力されているかどうかを評価する。

- /localization/pose_estimator/pose

これは、Component State MonitorというAutoware内のパッケージを間接的に利用することによって実現される。本ツールは、下記のトピックを監視することによってその情報を取得する。

- /diagnostics_agg


## 評価結果

topic の subscribe 1 回につき、以下に記述する判定結果が出力される。

### 信頼度正常

/localization/pose_estimator/transform_probability、または/localization/pose_estimator/nearest_voxel_transformation_likelihood の data がシナリオに記述した AllowableLikelihood 以上の場合

### 信頼度異常

/localization/pose_estimator/transform_probability、または/localization/pose_estimator/nearest_voxel_transformation_likelihood の data がシナリオに記述した AllowableLikelihood 未満の場合

### 収束正常

以下の 3 つの条件を全て満たす場合

1. /localization/pose_estimator/pose と /localization/pose_twist_fusion_filter/pose から横方向の距離を計算して、シナリオに記述した AllowableDistance 以下
2. /localization/pose_estimator/exe_time_ms が、シナリオに記述した AllowableExeTimeMs 以下
3. /localization/pose_estimator/iteration_num が、シナリオに記述した AllowableIterationNum 以下

ステップ 1 で計算した横方向の距離が/driving_log_replayer/localization/lateral_distance として publish される。

### 収束異常

収束正常の条件を満たさない場合

### 可用性正常

Component State Monitorが出力する `/diagnostics_agg` の中から、監視トピックに関する情報を抽出する。
最新の情報におけるStatusが `Timeout` または `NotReceived` 以外の場合、正常であると判断する。

### 可用性異常

可用性正常の条件を満たさない場合

## 評価ノードが使用する Topic 名とデータ型

Subscribed topics:

| topic 名                                                             | データ型                              |
| -------------------------------------------------------------------- | ------------------------------------- |
| /diagnostics_agg                                                     | diagnostic_msgs::msg::DiagnosticArray |
| /localization/pose_estimator/transform_probability                   | tier4_debug_msgs::msg::Float32Stamped |
| /localization/pose_estimator/nearest_voxel_transformation_likelihood | tier4_debug_msgs::msg::Float32Stamped |
| /localization/pose_estimator/pose                                    | geometry_msgs::msg::PoseStamped       |
| /localization/kinematic_state                                        | nav_msgs::msg::Odometry               |
| /localization/pose_estimator/exe_time_ms                             | tier4_debug_msgs::msg::Float32Stamped |
| /localization/pose_estimator/iteration_num                           | tier4_debug_msgs::msg::Int32Stamped   |
| /tf                                                                  | tf2_msgs/msg/TFMessage                |
| /localization/util/downsample/pointcloud                             | sensor_msgs::msg::PointCloud2         |
| /localization/pose_estimator/points_aligned                          | sensor_msgs::msg::PointCloud2         |

Published topics:

| topic 名                                            | データ型                         |
| --------------------------------------------------- | -------------------------------- |
| /driving_log_replayer/localization/lateral_distance | example_interfaces::msg::Float64 |

## 評価ノードが使用する Service 名とデータ型

| service 名                   | データ型               |
| ---------------------------- | ---------------------- |
| /api/localization/initialize | InitializeLocalization |

## logging_simulator.launch に渡す引数

autoware の処理を軽くするため、評価に関係のないモジュールは launch の引数に false を渡すことで無効化する。以下を設定している。

- perception: false
- planning: false
- control: false

## simulation

シミュレーション実行に必要な情報を述べる。

### 入力 rosbag に含まれるべき topic

車両の ECU の CAN と、使用している sensor の topic が必要
以下は例であり、違うセンサーを使っている場合は適宜読み替える。

LiDAR が複数ついている場合は、搭載されているすべての LiDAR の packets を含める

| topic 名                           | データ型                                     |
| ---------------------------------- | -------------------------------------------- |
| /gsm8/from_can_bus                 | can_msgs/msg/Frame                           |
| /sensing/gnss/ublox/fix_velocity   | geometry_msgs/msg/TwistWithCovarianceStamped |
| /sensing/gnss/ublox/nav_sat_fix    | sensor_msgs/msg/NavSatFix                    |
| /sensing/gnss/ublox/navpvt         | ublox_msgs/msg/NavPVT                        |
| /sensing/imu/tamagawa/imu_raw      | sensor_msgs/msg/Imu                          |
| /sensing/lidar/\*/velodyne_packets | velodyne_msgs/VelodyneScan                   |

CAN の代わりに vehicle の topic を含めても良い。

| topic 名                               | データ型                                            |
| -------------------------------------- | --------------------------------------------------- |
| /sensing/gnss/ublox/fix_velocity       | geometry_msgs/msg/TwistWithCovarianceStamped        |
| /sensing/gnss/ublox/nav_sat_fix        | sensor_msgs/msg/NavSatFix                           |
| /sensing/gnss/ublox/navpvt             | ublox_msgs/msg/NavPVT                               |
| /sensing/imu/tamagawa/imu_raw          | sensor_msgs/msg/Imu                                 |
| /sensing/lidar/\*/velodyne_packets     | velodyne_msgs/VelodyneScan                          |
| /vehicle/status/control_mode           | autoware_auto_vehicle_msgs/msg/ControlModeReport    |
| /vehicle/status/gear_status            | autoware_auto_vehicle_msgs/msg/GearReport           |
| /vehicle/status/steering_status        | autoware_auto_vehicle_msgs/SteeringReport           |
| /vehicle/status/turn_indicators_status | autoware_auto_vehicle_msgs/msg/TurnIndicatorsReport |
| /vehicle/status/velocity_status        | autoware_auto_vehicle_msgs/msg/VelocityReport       |

### 入力 rosbag に含まれてはいけない topic

| topic 名 | データ型                |
| -------- | ----------------------- |
| /clock   | rosgraph_msgs/msg/Clock |

clock は、ros2 bag play の--clock オプションによって出力しているので、bag 自体に記録されていると 2 重に出力されてしまうので bag には含めない

## evaluation

評価に必要な情報を述べる。

### シナリオフォーマット

[サンプル](https://github.com/tier4/driving_log_replayer/blob/main/sample/localization/scenario.ja.yaml)参照

### 評価結果フォーマット

[サンプル](https://github.com/tier4/driving_log_replayer/blob/main/sample/localization/result.json)参照

localization では、収束性と信頼度の 2 つを評価しているので、行毎に収束性または信頼度のどちらかの結果が入っている。
Result は収束性と信頼度両方のパスしていれば true でそれ以外は false 失敗となる。

以下に、それぞれの評価の例を記述する。
**注:結果ファイルフォーマットで解説済みの共通部分については省略する。**

収束性の結果(Frame の中に Convergence 項目がある場合)

```json
{
  "Frame": {
    "Convergence": {
      "Result": "Success or Fail",
      "Info": [
        {
          "LateralDistance": "ndtとekfのposeの横方距離",
          "HorizontalDistance": "ndtとekfの水平距離。参考値",
          "ExeTimeMs": "ndtの計算にかかった時間",
          "IterationNum": "ndtの再計算回数"
        }
      ]
    }
  }
}
```

信頼度の結果(Frame に Reliability の項目がある場合)

```json
{
  "Frame": {
    "Reliability": {
      "Result": "Success or Fail",
      "Info": [
        {
          "Value": {
            "stamp": {
              "sec": "stampの秒",
              "nanosec": "stampのnano秒"
            },
            "data": "NVTL or TPの値"
          },
          "Reference": {
            "stamp": {
              "sec": "stampの秒",
              "nanosec": "stampのnano秒"
            },
            "data": "評価に使用しなかった尤度。参考値。ValueがNVTLならTPが入る"
          }
        }
      ]
    }
  }
}
```
