# 診断機能の評価

Autoware の診断機能(diagnostics)が意図通りに機能しているかを評価する。

現在は、lidar の visibility と blockage の評価に対応している。

- visibility: 霧や雨などで視界が悪くなっていないかを判定する機能
- blockage: LiDAR に葉っぱなどが付着して計測の妨げをしていないかを判定する機能

## 評価方法

`performance_diag.launch.py` を使用して評価する。
launch を立ち上げると以下のことが実行され、評価される。

1. launch で評価ノード(`performance_diag_evaluator_node`)と `logging_simulator.launch`、`ros2 bag play`コマンドを立ち上げる
2. bag から出力されたセンサーデータを autoware が受け取って、/diagnostics を出力する
3. 評価ノードが/diagnostics を subscribe して、コールバックで評価を行い結果をファイルに記録する。
4. bag の再生が終了すると自動で launch が終了して評価が終了する

### visibility 評価

visibility の評価では、雨天時や人工的に雨を降らせられる施設で取得したデータを用いて visibility の ERROR が一定数以上出力されることを確認する。
また、晴天時のデータを利用して、ERROR が一度も出ないことを確認する。

`/diagnostics`の`status.name`が`dual_return_filter: /sensing/lidar/.*: visibility_validation`に該当するものを判定に利用する。

### blockage 評価

blockage の評価では、LiDAR を意図的にレーザー光を通さない素材(箱など)で覆った状態でデータを取得し blockage の ERROR が一定数以上出力されることを確認する。
また、覆ってない LiDAR については ERROR が一度も出ないことを確認する。

`/diagnostics`の`status.name`が`blockage_return_diag: /sensing/lidar/.*: blockage_validation`に該当するものを判定に利用する。

## 評価結果

LiDAR の診断結果の subscribe 1 回につき、以下に記述する判定結果が出力される。

評価したい内容によって ERROR が出る場合を成功とするか失敗とするかが分かれるので、シナリオにタイプを記述することで変更できるようになっている。

- シナリオ種類が TP の場合は Diag が ERROR になれば成功
- シナリオ種類が FP の場合は Diag が ERROR にならなければ成功
- シナリオ種類が null の場合はテストを省略する

### TP 正常

シナリオ種類が TP の場合で、診断情報の level が ERROR(=2)の場合

### TP 異常

シナリオ種類が TP の場合で、診断情報の level が ERROR でない(!=2)の場合

### FP 正常

シナリオ種類が FP の場合で、診断情報の level が ERROR でない(!=2)の場合

### FP 異常

シナリオ種類が FP の場合で、診断情報の level が ERROR(=2)の場合

## 評価ノードが使用する Topic 名とデータ型

Subscribed topics:

| topic 名                                     | データ型                              |
| -------------------------------------------- | ------------------------------------- |
| /perception/obstacle_segmentation/pointcloud | sensor_msgs::msg::PointCloud2         |
| /diagnostics                                 | diagnostic_msgs::msg::DiagnosticArray |
| /tf                                          | tf2_msgs/msg/TFMessage                |

Published topics:

| topic 名                                                 | データ型                         |
| -------------------------------------------------------- | -------------------------------- |
| /driving_log_replayer/visibility/value                   | example_interfaces::msg::Float64 |
| /driving_log_replayer/visibility/level                   | example_interfaces::msg::Byte    |
| /driving_log_replayer/blockage/{lidar_name}/ground/ratio | example_interfaces::msg::Float64 |
| /driving_log_replayer/blockage/{lidar_name}/sky/ratio    | example_interfaces::msg::Float64 |
| /driving_log_replayer/blockage/{lidar_name}/level        | example_interfaces::msg::Byte    |

{lidar_name}には、搭載されている lidar の名前が入る。

## 評価ノードが使用する Service 名とデータ型

| service 名                   | データ型               |
| ---------------------------- | ---------------------- |
| /api/localization/initialize | InitializeLocalization |

## logging_simulator.launch に渡す引数

autoware の処理を軽くするため、評価に関係のないモジュールは launch の引数に false を渡すことで無効化する。以下を設定している。

- planning: false
- control: false
- localization: false / true (デフォルト false、シナリオで指定する)

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
| /tf                                | tf2_msgs/msg/TFMessage                       |

CAN の代わりに vehicle の topic を含めても良い。

| topic 名                               | データ型                                            |
| -------------------------------------- | --------------------------------------------------- |
| /sensing/gnss/ublox/fix_velocity       | geometry_msgs/msg/TwistWithCovarianceStamped        |
| /sensing/gnss/ublox/nav_sat_fix        | sensor_msgs/msg/NavSatFix                           |
| /sensing/gnss/ublox/navpvt             | ublox_msgs/msg/NavPVT                               |
| /sensing/imu/tamagawa/imu_raw          | sensor_msgs/msg/Imu                                 |
| /sensing/lidar/\*/velodyne_packets     | velodyne_msgs/VelodyneScan                          |
| /tf                                    | tf2_msgs/msg/TFMessage                              |
| /vehicle/status/control_mode           | autoware_auto_vehicle_msgs/msg/ControlModeReport    |
| /vehicle/status/gear_status            | autoware_auto_vehicle_msgs/msg/GearReport           |
| /vehicle/status/steering_status        | autoware_auto_vehicle_msgs/SteeringReport           |
| /vehicle/status/turn_indicators_status | autoware_auto_vehicle_msgs/msg/TurnIndicatorsReport |
| /vehicle/status/velocity_status        | autoware_auto_vehicle_msgs/msg/VelocityReport       |

**注:localization が false(デフォルトで false)の場合は/tf が使用される。**

### 入力 rosbag に含まれてはいけない topic

| topic 名 | データ型                |
| -------- | ----------------------- |
| /clock   | rosgraph_msgs/msg/Clock |

clock は、ros2 bag play の--clock オプションによって出力しているので、bag 自体に記録されていると 2 重に出力されてしまうので bag には含めない

## evaluation

評価に必要な情報を述べる。

### シナリオフォーマット

[サンプル](https://github.com/tier4/driving_log_replayer/blob/main/sample/performance_diag/scenario.ja.yaml)参照

### 評価結果フォーマット

[サンプル](https://github.com/tier4/driving_log_replayer/blob/main/sample/performance_diag/result.json)参照

performance_diag では、visibility と blockage の 2 つを評価している。
Result は visibility と blockage の両方をパスしていれば true でそれ以外は false 失敗となる。

以下に、フォーマットを示す。
**注:結果ファイルフォーマットで解説済みの共通部分については省略する。**

visibilityの結果(Frame に Visibility の項目がある場合)

```json
{
  "Visibility": {
    "Result": { "Total": "Success or Fail", "Frame": "Success, Fail, or Invalid" },
    "Info": {
      "Level": "diagのレベル",
      "Visibility": "visibilityの値"
    }
  }
}
```

blockageの結果(Frame に Blockage の項目がある場合)

```json
{
  "Blockage": {
    "LiDAR1の名前": {
      "Result": { "Total": "Success or Fail", "Frame": "Success or Fail" },
      "Info": {
        "Level": "diagのレベル",
        "GroundBlockageRatio": "地上側のblockage比率",
        "GroundBlockageCount": "参考値",
        "SkyBlockageRatio": "空中側のblockage比率",
        "SkyBlockageCount": "参考値"
      }
    },
    "LiDAR2の名前": {
      "Result": { "Total": "Success or Fail", "Frame": "Success or Fail" },
      "Info": {
        "Level": "diagのレベル",
        "GroundBlockageRatio": "地上側のblockage比率",
        "GroundBlockageCount": "参考値",
        "SkyBlockageRatio": "空中側のblockage比率",
        "SkyBlockageCount": "参考値"
      }
    }
  }
}
```
