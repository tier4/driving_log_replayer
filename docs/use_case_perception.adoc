== ユースケース：認識機能の評価

Autowareの認識機能(perception)の認識結果からmAP(mean Average Precision)などの指標を計算して性能を評価する。

perceptionモジュールを起動して出力されるperceptionのtopicを評価用ライブラリ<<introduction.adoc#perception_eval>>に渡して評価を行う。

* /perception/object_recognition/detection/objects|autoware_auto_perception_msgs/msg/DetectedObjects
* /perception/object_recognition/tracking/objects|autoware_auto_perception_msgs/msg/TrackedObjects

=== 注意事項
autowareをセットアップしてperceptionのモジュールを初回起動した場合には、lidar_centerpointのonnxファイルの変換処理の待ちが入るため、途中でlaunchが止まったように見えることがある。

青文字太字灰色背景でlidar_centerpoint engine files are generated.
という文字列が表示されるまでは、そのまま待つこと。

=== 依存ライブラリ
<<introduction.adoc#perception_eval>>

==== 依存ライブラリとのdriving_log_replayerの役割分担
ROSとの接続部分がdriving_log_replayer、データセットを扱う部分がperception_evalという役割分担になっている。

perception_evalはROS非依存のライブラリなので、ROSのオブジェクトをそのまま受け取ることができない。
また、timestampがROSではナノ秒、データセットはnuScenesベースでミリ秒が使用されている。

driving_log_replayer側では、autowareのperceptionモジュールから出力されたtopicをsubscribeし、perception_evalが期待するデータ形式に変換する。
変換したデータをperception_evalに渡して評価を依頼し、結果を受け取って、可視化のためにROSのtopicで結果をpublishする部分を担当する。

perception_evalは、driving_log_replayerから渡された検知結果とGroundTruthを比較して指標を計算し、結果を出力する部分を担当する。

=== 評価方法
driving_log_replayer/launch/perception.launch.pyを用いて、評価用のノードをautoware_launchのlogging_simulator.launchと一緒に立ち上げる。

=== 評価ノードが使用するTopicとデータ型

* subscribe

|===
|topic名|データ型

|/perception/object_recognition/detection/objects|autoware_auto_perception_msgs/msg/DetectedObjects
|/perception/object_recognition/tracking/objects|autoware_auto_perception_msgs/msg/TrackedObjects
|===


* publish

|===
|topic名|データ型

|/driving_log_replayer/marker/ground_truth|visualization_msgs::msg::MarkerArray
|/driving_log_replayer/marker/results|visualization_msgs::msg::MarkerArray
|===

==== logging_simulator.launchに渡す引数
autowareの処理を軽くするため、評価に関係のないモジュールはlaunchの引数にfalseを渡すことで無効化する。以下を設定している。
アノーテション時とシミュレーション時で自己位置を合わせたいのでbagに入っているtfを使い回す。そのためlocalizationは無効である。

- localization: false
- planning: false
- control: false
- sensing: false / true (デフォルトfalse、シナリオで指定する)

autowareから出力される認識結果(/perception/object_recognition/detection/objects)をGround Truthデータと比較して評価する。
評価には、<<introduction.adoc#perception_eval>>を利用する。

==== 正常
perception_evalの関数を使って評価失敗のオブジェクトが0個の場合、frame_result.pass_fail_result.get_fail_object_num() == 0

==== 異常
perception_evalの関数を使って評価失敗のオブジェクトが存在する場合、frame_result.pass_fail_result.get_fail_object_num() > 0

==== 評価フロー
1. launchで評価ノード(perception_evaluator_node)とlogging_simulator.launch、ros2 bag playを立ち上げる
2. bagから出力されたセンサーデータをautowareが受け取って、perceptionモジュールで認識を行う
3. 評価ノードが/perception/object_recognition/{detection, tracking}/objectsをsubscribeして、コールバックでperception_evalの関数を用いて評価を行う

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

tfは実験時に取得したものをそのまま使用するか、rosbag replay simulationで取得しなおす。

==== 入力rosbagに含まれてはいけないtopic
- /clock

autowareのドキュメントサイトにあるbagはclockを含んでいて使用出来ないので注意。

=== evaluation
評価に必要な情報を述べる。

==== シナリオフォーマット
ユースケース評価とデータベース評価の2種類の評価がある。
ユースケースは1個のデータセットで行う評価で、データベースは複数のデータセットを用いて、各データセット毎の結果の平均を取る評価である。

クラウドで実行されるときは、並列に実行されることも考慮にいれるとDataset毎の設定を配列で記述する以下のような形式とする。
データベース評価では、キャリブレーション値の変更があり得るのでvehicle_idをデータセット毎に設定出来るようにする。
また、Sensingモジュールを起動するかどうかの設定も行う。
追加でデータセット毎に必要な設定項目が出てきたら、キーを追加して設定する。

成否判定に関してはperception_evalに判定の条件を渡すことで結果を出すときに成否の判定も同時にされるので、判定条件をシナリオに記載する。

記述形式は以下のyamlのようになっており、PerceptionEvaluationConfig、CriticalObjectFilterConfig、PerceptionPassFailConfigの3種類を記述する。
各設定オブジェクトに追加や削除があればそれに合わせてシナリオのConfigの項目も追加・削除する。

```yaml
Evaluation:
  UseCaseName: perception
  UseCaseFormatVersion: 0.3.0
  Datasets:
    - f72e1065-7c38-40fe-a4e2-c5bbe6ff6443:
        VehicleId: ps1/20210620/CAL_000015 # データセット毎にVehicleIdを指定する
        LaunchSensing: false # データセット毎にsensing moduleを起動するかを指定する
        LocalMapPath: $HOME/map/perception # データセット毎にLocalMapPathを指定する
  Conditions:
    PassRate: 99.0 # 評価試行回数の内、どの程度(%)評価成功だったら成功とするか
  PerceptionEvaluationConfig:
    evaluation_config_dict:
      evaluation_task: detection # detection/tracking ここで指定したobjectsを評価する
      target_labels: [car, bicycle, pedestrian, motorbike] # 評価ラベル
      max_x_position: 102.4 # 評価対象 object の最大 x 位置
      max_y_position: 102.4 # 評価対象 object の最大 y 位置
      max_distance: null # 評価対象 object の base_link からの最大距離、max_x_potion, max_y_positionと排他利用、この例ではこちらはnull
      min_distance: null # 評価対象 object の base_link からの最小距離、max_x_potion, max_y_positionと排他利用、この例ではこちらはnull
      min_point_numbers: [0, 0, 0, 0] # ground truth object における，bbox 内の最小点群数．min_point_numbers=0 の場合は，全 ground truth object を評価
      confidence_threshold: null # 評価対象の estimated object の confidence の閾値
      target_uuids: null # 特定の ground truth のみに対して評価を行いたい場合，対象とする ground truth の UUID を指定する。nullなら全てが対象
      center_distance_thresholds: [[1.0, 1.0, 1.0, 1.0], [2.0, 2.0, 2.0, 2.0]] # 中心間距離マッチング時の閾値
      plane_distance_thresholds: [2.0, 30.0] # 平面距離マッチング時の閾値
      iou_bev_thresholds: [0.5] # BEV IoU 　マッチング時の閾値
      iou_3d_thresholds: [0.5] # 3D IoU マッチング時の閾値
  CriticalObjectFilterConfig:
    target_labels: [car, bicycle, pedestrian, motorbike] # 評価対象ラベル名
    max_x_position_list: [30.0, 30.0, 30.0, 30.0] # 評価対象 object の最大 x 位置
    max_y_position_list: [30.0, 30.0, 30.0, 30.0] # 評価対象 object の最大 y 位置
    max_distance_list: null # 評価対象 object の base_link からの最大距離、max_x_potion, max_y_positionと排他利用、この例ではこちらはnull
    min_distance_list: null # 評価対象 object の base_link からの最小距離、max_x_potion, max_y_positionと排他利用、この例ではこちらはnull
    min_point_numbers: [0, 0, 0, 0] # ground truth object における，bbox 内の最小点群数．min_point_numbers=0 の場合は，全 ground truth object を評価
    confidence_threshold_list: null # 評価対象の estimated object の confidence の閾値
    target_uuids: null # 特定の ground truth のみに対して評価を行いたい場合，対象とする ground truth の UUID を指定する。nullなら全てが対象
  PerceptionPassFailConfig:
    target_labels: [car, bicycle, pedestrian, motorbike]
    plane_distance_threshold_list: [2.0, 2.0, 2.0, 2.0] # 平面距離マッチング時の閾値
```

==== 評価結果ファイルフォーマット
perceptionでは、シナリオに指定した条件でperception_evalが評価した結果を各frame毎に出力する。
全てのデータを流し終わったあとに、最終的なメトリクスを計算しているため、最終行だけ、他の行と形式が異なる。

以下に、各フレームのフォーマットとメトリクスのフォーマットを示す。
ただし、<<result_format.adoc#sec-result-format>>で解説済みの共通部分については省略する。

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
        "TP": "ラベル毎のTP率",
        "FP": "ラベル毎のFP率",
        "FN": "ラベル毎のFN率",
        "AP": "ラベル毎のAP値",
        "APH": "ラベル毎のAPH値"
      },
      "Error": {
        "ラベル": "ラベル毎の値との誤差メトリクス"
      }
    }
  }
}
```

==== pickleファイル
perceptionの評価では、result.jsonの他にscene_result.pklというファイルを出力する。
pickleファイルはpythonのオブジェクトをファイルとして保存したものであり、perception_evalのPerceptionEvaluationManager.frame_resultsを保存している。

perceptionの評価は、1個のデータセットで行うユースケース評価と、複数のデータセットを用いて各データセット毎の結果の平均を取るデータベース評価がある。
ROSの仕様上、1回のlaunchで、複数のデータセット(bag、地図、アノテーションファイルの集合)を渡して連続で実行することは出来ない。
1組のデータセットに対してlaunchを1回叩くことなるので、データベース評価を実施するには、含まれるデータセットの数だけlaunchを実行する必要がある。

なので、それぞれのデータセットに対して実行したframe_resultsをpickleファイルで保存し、全てのデータセットでpickleファイルが作成されたあとにframe_resultsを結合して最終的な結果を出すという運用を想定している。

pickleファイルを使用するサンプル実装をdriving_log_replayer/scripts/perception_load_scene_result.pyに置いているので参照されたい。
