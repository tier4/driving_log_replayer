# Eagleye自己位置推定の評価

Autoware のEagleye自己位置推定が安定して動作しているかを評価する。

## 評価方法

`eagleye.launch.py` を使用して評価する。
launch を立ち上げると以下のことが実行され、評価される。

1. launch で評価ノード(`eagleye_evaluator_node`)と `logging_simulator.launch`、`ros2 bag play`コマンドを立ち上げる
2. bag から出力されたセンサーデータを autoware が受け取って、自己位置推定を行う
3. 評価ノードが topic を subscribe して、各基準を満たしているかを判定して結果をファイルに記録する
4. bag の再生が終了すると自動で launch が終了して評価が終了する

### Eagleye の可用性

本項目では、Eagleyeの可用性を評価するために用意されている。これは、具体的には、下記のようなケースを検知することを目的とする。

- Runtime error等により、Headingの推定がうまく動作していない
- 初期位置推定周りのアーキテクチャ変更が不十分で、そもそもEagleyeが初期化されない

そのために、本項目では下記の出力が定期的に出力されているかどうかを評価する。

- /localization/pose_twist_estimator/eagleye/enu_absolute_pos_interpolate

これは、Eagleye MonitorというAutoware内のパッケージを間接的に利用することによって実現される。本ツールは、下記のトピックを監視することによってその情報を取得する。

- /diagnostics

## 評価結果

topic の subscribe 1 回につき、以下に記述する判定結果が出力される。

### 可用性正常

Eagleye Monitorが出力する `/diagnostics` の中から、監視トピックに関する情報を抽出する。
最新の情報におけるAvailabilityが `OK` である場合、正常であると判断する。

### 可用性異常

可用性正常の条件を満たさない場合

## 評価ノードが使用する Topic 名とデータ型

Subscribed topics:

| Topic name   | Data type                             |
| ------------ | ------------------------------------- |
| /diagnostics | diagnostic_msgs::msg::DiagnosticArray |

Published topics:

| Topic name | Data type |
| ---------- | --------- |
| N/A        | N/A       |

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

| topic 名                        | データ型                                      |
| ------------------------------- | --------------------------------------------- |
| /sensing/gnss/ublox/nav_sat_fix | sensor_msgs/msg/NavSatFix                     |
| /sensing/gnss/ublox/navpvt      | ublox_msgs/msg/NavPVT                         |
| /sensing/imu/tamagawa/imu_raw   | sensor_msgs/msg/Imu                           |
| /vehicle/status/velocity_status | autoware_auto_vehicle_msgs/msg/VelocityReport |

### 入力 rosbag に含まれてはいけない topic

| topic 名 | データ型                |
| -------- | ----------------------- |
| /clock   | rosgraph_msgs/msg/Clock |

clock は、ros2 bag play の--clock オプションによって出力しているので、bag 自体に記録されていると 2 重に出力されてしまうので bag には含めない

## evaluation

評価に必要な情報を述べる。

### シナリオフォーマット

[サンプル](https://github.com/tier4/driving_log_replayer/blob/main/sample/eagleye/scenario.yaml)参照

### 評価結果フォーマット

[サンプル](https://github.com/tier4/driving_log_replayer/blob/main/sample/eagleye/result.json)参照

以下に、それぞれの評価の例を記述する。
**注:結果ファイルフォーマットで解説済みの共通部分については省略する。**

Availabilityの結果(Frame の中に Availability 項目がある場合)

```json
{
  "Availability": {
   "Result": { "Total": "Success or Fail", "Frame": "Success, Fail, or Warn" },
   "Info": {}
  }
}
```
