# 診断機能の評価

Autoware の診断機能(diagnostics)が意図通りに機能しているかを評価する。

現在は、lidar の visibility と blockage の評価に対応している。

- visibility: 霧や雨などで視界が悪くなっていないかを判定する機能
- blockage: LiDAR に葉っぱなどが付着して計測の妨げをしていないかを判定する機能

## 評価方法

performance_diag.launch.pyを使用して評価する。
launchを立ち上げると以下のことが実行され、評価される。

1. launch で評価ノード(performance_diag_evaluator_node)と logging_simulator.launch、ros2 bag play を立ち上げる
2. bag から出力されたセンサーデータを autoware が受け取って、/diagnostics_agg を出力する
3. 評価ノードが/diagnostics_agg を subscribe して、コールバックで評価を行い結果をファイルに記録する。
4. bagの再生が終了すると自動でlaunchが終了して評価が終了する

### visibility 評価

visibility の評価では、雨天時や人工的に雨を降らせられる施設で取得したデータを用いて visibility の ERROR が一定数以上出力されることを確認する。
また、晴天時のデータを利用して、ERROR が一度も出ないことを確認する。

### blockage 評価

blockage の評価では、LiDAR をビニール袋などでわざと覆った状態でデータを取得し blockage の ERROR が一定数以上出力されることを確認する。
また、覆ってない LiDAR については ERROR が一度も出ないことを確認する。

## 評価結果

LiDARの診断結果のsubscribe 1回につき、以下に記述する判定結果が出力される。

- visibility: /autoware/sensing/lidar/performance_monitoring/visibility/.\*
- blockage: /autoware/sensing/lidar/performance_monitoring/blockage/.\*

評価したい内容によって ERROR が出る場合を成功とするか失敗とするかが分かれるので、シナリオにタイプを記述することで変更できるようになっている。

- シナリオ種類が TP の場合は Diag が一定数以上 ERROR になれば成功
- シナリオ種類が FP の場合は Diag が一度も ERROR にならなければ成功
- シナリオ種類が null の場合はテストを省略する

### TP 正常

シナリオ種類が TP の場合で、診断情報(/diagnostics_agg)に含まれる visibility または blockage の level が ERROR(=2)の場合

### TP 異常

シナリオ種類が TP の場合で、診断情報(/diagnostics_agg)に含まれる visibility または blockage の level が ERROR でない(!=2)の場合

### FP 正常

シナリオ種類が FP の場合で、診断情報(/diagnostics_agg)に含まれる visibility または blockage の level が ERROR でない(!=2)の場合

### FP 異常

シナリオ種類が FP の場合で、診断情報(/diagnostics_agg)に含まれる visibility または blockage の level が ERROR(=2)の場合

## 評価ノードが使用する Topic 名とデータ型

- subscribe

| topic 名                                     | データ型                              |
| -------------------------------------------- | ------------------------------------- |
| /perception/obstacle_segmentation/pointcloud | sensor_msgs::msg::PointCloud2         |
| /diagnostics_agg                             | diagnostic_msgs::msg::DiagnosticArray |
| /tf                                          | tf2_msgs/msg/TFMessage                |

- publish

| topic 名                                                 | データ型                                      |
| -------------------------------------------------------- | --------------------------------------------- |
| /driving_log_replayer/visibility/value                   | example_interfaces::msg::Float64              |
| /driving_log_replayer/visibility/level                   | example_interfaces::msg::Byte                 |
| /driving_log_replayer/blockage/{lidar_name}/ground/ratio | example_interfaces::msg::Float64              |
| /driving_log_replayer/blockage/{lidar_name}/sky/ratio    | example_interfaces::msg::Float64              |
| /driving_log_replayer/blockage/{lidar_name}/level        | example_interfaces::msg::Byte                 |
| /initialpose                                             | geometry_msgs::msg::PoseWithCovarianceStamped |

{lidar_name}には、搭載されている lidar の名前が入る。

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

- /sensing/gnss/ublox/fix_velocity
- /sensing/gnss/ublox/nav_sat_fix
- /sensing/gnss/ublox/navpvt
- /sensing/imu/tamagawa/imu_raw
- /sensing/lidar/\*/velodyne_packets
- /gsm8/from_can_bus
- /tf

### 入力 rosbag に含まれてはいけない topic

- /clock

## evaluation

評価に必要な情報を述べる。

### シナリオフォーマット

```yaml
Evaluation:
  UseCaseName: performance_diag
  UseCaseFormatVersion: 1.0.0
  LaunchLocalization: false # falseのときはbagの中に/tfが入っている必要がある。
  InitialPose: null # LaunchLocalizationが有効のときだけ機能する
  Conditions:
    LiDAR:
      Visibility:
        PassFrameCount: 100 # ScenarioTypeがTPのときにこの値以上ERRORが出ればVisibilityの試験は成功とする。FPの場合はERRORが一切出ないことが条件なので無視される
        ScenarioType: FP # TP/FP/null
      Blockage:
        front_lower: # 搭載されているLidar毎に設定する
          ScenarioType: TP # TP/FP/null
          BlockageType: both # sky/ground/both 空側、地面側、またはその両方、どこでblockageが発生しているか
          PassFrameCount: 100 # ScenarioTypeがTPで、かつ、Blockageのタイプが一致するERRORがこの値以上出ればBlockageの試験は成功とする。FPの場合はERRORが一切出ないことが条件なので無視される
        front_upper:
          ScenarioType: TP
          BlockageType: both
          PassFrameCount: 100
        left_lower:
          ScenarioType: FP
          BlockageType: sky
          PassFrameCount: 30
        left_upper:
          ScenarioType: FP
          BlockageType: both
          PassFrameCount: 40
        rear_lower:
          ScenarioType: FP
          BlockageType: ground
          PassFrameCount: 50
        rear_upper:
          ScenarioType: FP
          BlockageType: sky
          PassFrameCount: 60
        right_lower:
          ScenarioType: FP
          BlockageType: both
          PassFrameCount: 70
        right_upper:
          ScenarioType: FP
          BlockageType: ground
          PassFrameCount: 80
```

### 評価結果フォーマット

performance_diag では、visibility と blockage の 2 つを評価している。
1 回の点群の callback で同時に評価しているが、それぞれ別にカウントしている。
Result は visibility と blockage の両方をパスしていれば true でそれ以外は false 失敗となる。

以下に、フォーマットを示す。
ただし、結果ファイルフォーマットで解説済みの共通部分については省略する。

```json
{
  "Frame": {
    "Visibility": [
      {
        "Result": "Success or Fail or Skipped",
        "Info": [
          {
            "Level": "diagのレベル",
            "Visibility": "visibilityの値"
          }
        ]
      }
    ],
    "Blockage": [
      {
        "Result": "Success or Fail or Skipped",
        "Info": [
          {
            "Name": "lidarの名前",
            "Level": "diagのレベル",
            "GroundBlockageRatio": "地上側のblockage比率",
            "GroundBlockageCount": "参考値",
            "SkyBlockageRatio": "空中側のblockage比率",
            "SkyBlockageCount": "参考値"
          }
        ]
      }
    ]
  }
}
```
