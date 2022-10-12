== ユースケース：診断機能の評価

Autowareの診断機能(diagnostics)が意図通りに機能しているかを評価する。

現在は、lidarのvisibilityとblockageの評価に対応している。

* visibility: 霧や雨などで視界が悪くなっていないかを判定する機能
* blockage: LiDARに葉っぱなどが付着して計測の妨げをしていないかを判定する機能

visibilityの評価では、雨天時や人工的に雨を降らせられる施設で取得したデータを用いてvisibilityのERRORが一定数以上出力されることを確認する。
また、晴天時のデータを利用して、ERRORが一度も出ないことを確認する。

blockageの評価では、LiDARをビニール袋などでわざと覆った状態でデータを取得しblockageのERRORが一定数以上出力されることを確認する。
また、覆ってないLiDARについてはERRORが一度も出ないことを確認する。

上記のように、評価したい内容によってERRORが出る場合を成功とするか失敗とするかが分かれるので、シナリオにタイプを記述することで変更できるようになっている。

* シナリオ種類がTPの場合はDiagが一定数以上ERRORになれば成功
* シナリオ種類がFPの場合はDiagが一度もERRORにならなければ成功
* シナリオ種類がnullの場合はテストを省略する

=== 評価方法
driving_log_replayer/launch/performance_diag.launch.pyを用いて、評価用のノードをautoware_launchのlogging_simulator.launchと一緒に立ち上げる。

=== 評価ノードが使用するTopicとデータ型

* subscribe

|===
|topic名|データ型

|/perception/obstacle_segmentation/pointcloud|sensor_msgs::msg::PointCloud2
|/diagnostics_agg|diagnostic_msgs::msg::DiagnosticArray
|/tf|tf2_msgs/msg/TFMessage
|===

* publish

|===
|topic名|データ型

|/driving_log_replayer/visibility/value|example_interfaces::msg::Float64
|/driving_log_replayer/visibility/level|example_interfaces::msg::Byte
|/driving_log_replayer/blockage/{lidar_name}/ground/ratio|example_interfaces::msg::Float64
|/driving_log_replayer/blockage/{lidar_name}/sky/ratio|example_interfaces::msg::Float64
|/driving_log_replayer/blockage/{lidar_name}/level|example_interfaces::msg::Byte
|/initialpose|geometry_msgs::msg::PoseWithCovarianceStamped
|===

{lidar_name}には、搭載されているlidarの名前が入る。

==== logging_simulator.launchに渡す引数
autowareの処理を軽くするため、評価に関係のないモジュールはlaunchの引数にfalseを渡すことで無効化する。以下を設定している。

- planning: false
- control: false
- localization: false / true (デフォルトfalse、シナリオで指定する)

autowareから出力される診断情報(/diagnostics_agg)を評価する。
メッセージ毎に以下のいずれかとして評価される。

対象となるstatusは、LiDARのvisibilityとblockageに対応していて、

- visibility: /autoware/sensing/lidar/performance_monitoring/visibility/.*
- blockage: /autoware/sensing/lidar/performance_monitoring/blockage/.*

==== TP正常
シナリオ種類がTPの場合で、診断情報(/diagnostics_agg)に含まれるvisibilityまたはblockageのlevelがERROR(=2)の場合

==== TP異常
シナリオ種類がTPの場合で、診断情報(/diagnostics_agg)に含まれるvisibilityまたはblockageのlevelがERRORでない(!=2)の場合

==== FP正常
シナリオ種類がFPの場合で、診断情報(/diagnostics_agg)に含まれるvisibilityまたはblockageのlevelがERRORでない(!=2)の場合

==== FP異常
シナリオ種類がFPの場合で、診断情報(/diagnostics_agg)に含まれるvisibilityまたはblockageのlevelがERROR(=2)の場合

==== 評価フロー
1. launchで評価ノード(performance_diag_evaluator_node)とlogging_simulator.launch、ros2 bag playを立ち上げる
2. bagから出力されたセンサーデータをautowareが受け取って、/diagnostics_aggを出力する
3. 評価ノードが/diagnostics_aggをsubscribeして、コールバックで評価を行う

=== simulation
シミュレーション実行に必要な情報を述べる。

==== 入力rosbagに含まれるべきtopic
車両のECUのCANと、使用しているsensorのtopicが必要
以下は例であり、違うセンサーを使っている場合は適宜読み替える。

LiDARが複数ついている場合は、搭載されているすべてのLiDARのpacketsを含める

- /sensing/gnss/ublox/fix_velocity
- /sensing/gnss/ublox/nav_sat_fix
- /sensing/gnss/ublox/navpvt
- /sensing/imu/tamagawa/imu_raw
- /sensing/lidar/*/velodyne_packets
- /gsm8/from_can_bus
- /tf

==== 入力rosbagに含まれてはいけないtopic
- /clock

autowareのドキュメントサイトにあるbagはclockを含んでいて使用出来ないので注意。

=== evaluation
評価に必要な情報を述べる。

==== シナリオフォーマット

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

==== 評価結果ファイルフォーマット
performance_diagでは、visibilityとblockageの2つを評価している。
1回の点群のcallbackで同時に評価しているが、それぞれ別にカウントしている。
Resultはvisibilityとblockageの両方をパスしていればtrueでそれ以外はfalse失敗となる。

以下に、フォーマットを示す。
ただし、<<result_format.adoc#sec-result-format>>で解説済みの共通部分については省略する。


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
      },
    ]
  }
}
```
