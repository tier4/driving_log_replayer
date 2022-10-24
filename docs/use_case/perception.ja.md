# 認識機能の評価

Autoware の認識機能(perception)の認識結果から mAP(mean Average Precision)などの指標を計算して性能を評価する。

perception モジュールを起動して出力される perception の topic を評価用ライブラリに渡して評価を行う。

## 評価方法

perception.launch.pyを使用して評価する。
launchを立ち上げると以下のことが実行され、評価される。

1. launch で評価ノード(perception_evaluator_node)と logging_simulator.launch、ros2 bag play を立ち上げる
2. bag から出力されたセンサーデータを autoware が受け取って、点群データを出力し、perception モジュールが認識を行う
3. 評価ノードが/perception/object_recognition/{detection, tracking}/objects を subscribe して、コールバックで perception_eval の関数を用いて評価し結果をファイルに記録する
4. bagの再生が終了すると自動でlaunchが終了して評価が終了する

### 注意事項

評価方法ステップ1のlaunch起動時にbag playが始まらずに止まったように見えることがある。
これはautoware をワークスペースをセットアップして perception のモジュールを初回起動した場合には、lidar_centerpoint の onnx ファイルの変換処理の待ちが入るためである。

lidar_centerpoint engine files are generated.という文字列が表示されるまでは、そのまま待つこと。

## 評価結果

topicのsubscribe 1回につき、以下に記述する判定結果が出力される。

### 正常

perception_eval の関数を使って評価失敗のオブジェクトが 0 個の場合、frame_result.pass_fail_result.get_fail_object_num() == 0

### 異常

perception_eval の関数を使って評価失敗のオブジェクトが存在する場合、frame_result.pass_fail_result.get_fail_object_num() > 0

## 評価ノードが使用する Topic とデータ型

- subscribe

| topic 名                                         | データ型                                          |
| ------------------------------------------------ | ------------------------------------------------- |
| /perception/object_recognition/detection/objects | autoware_auto_perception_msgs/msg/DetectedObjects |
| /perception/object_recognition/tracking/objects  | autoware_auto_perception_msgs/msg/TrackedObjects  |

- publish

| topic 名                                  | データ型                             |
| ----------------------------------------- | ------------------------------------ |
| /driving_log_replayer/marker/ground_truth | visualization_msgs::msg::MarkerArray |
| /driving_log_replayer/marker/results      | visualization_msgs::msg::MarkerArray |

## logging_simulator.launch に渡す引数

autoware の処理を軽くするため、評価に関係のないモジュールは launch の引数に false を渡すことで無効化する。以下を設定している。
アノーテション時とシミュレーション時で自己位置を合わせたいので bag に入っている tf を使い回す。そのため localization は無効である。

- localization: false
- planning: false
- control: false
- sensing: false / true (デフォルト false、シナリオで指定する)

## 依存ライブラリ

[perception_eval](https://github.com/tier4/autoware_perception_evaluation)

### 依存ライブラリとの driving_log_replayer の役割分担

driving_log_replayerがROS との接続部分を担当し、perception_eval がデータセットを扱う部分がを担当するという分担になっている。
perception_eval は ROS 非依存のライブラリなので、ROS のオブジェクトを受け取ることができない。
また、timestamp が ROS ではナノ秒、データセットは ミリ秒が使用されている。
t4_datasetはnuScenesをベースとしており、nuScenesがミリ秒を採用しているのでt4_datasetもミリ秒となっている。

driving_log_replayerは、autoware の perception モジュールから出力された topic を subscribe し、perception_eval が期待するデータ形式に変換して渡す。
また、perception_eval から返ってくる評価結果のROS の topic でpublish し可視化する部分も担当する。

perception_eval は、driving_log_replayer から渡された検知結果と GroundTruth を比較して指標を計算し、結果を出力する部分を担当する。

## simulation

シミュレーション実行に必要な情報を述べる。

### 入力 rosbag に含まれるべき topic

t4_dataset で必要なトピックが含まれていること

## evaluation

評価に必要な情報を述べる。

### シナリオフォーマット

ユースケース評価とデータベース評価の 2 種類の評価がある。
ユースケースは 1 個のデータセットで行う評価で、データベースは複数のデータセットを用いて、各データセット毎の結果の平均を取る評価である。

データベース評価では、キャリブレーション値の変更があり得るので vehicle_id をデータセット毎に設定出来るようにする。
また、Sensing モジュールを起動するかどうかの設定も行う。

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

### 評価結果フォーマット

perception では、シナリオに指定した条件で perception_eval が評価した結果を各 frame 毎に出力する。
全てのデータを流し終わったあとに、最終的なメトリクスを計算しているため、最終行だけ、他の行と形式が異なる。

以下に、各フレームのフォーマットとメトリクスのフォーマットを示す。
ただし、結果ファイルフォーマットで解説済みの共通部分については省略する。

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
        "TP": "ラベルのTP率",
        "FP": "ラベルのFP率",
        "FN": "ラベルのFN率",
        "AP": "ラベルのAP値",
        "APH": "ラベルのAPH値"
      },
      "Error": {
        "ラベル": "ラベルの誤差メトリクス"
      }
    }
  }
}
```

### pickle ファイル

データベース評価では、複数のbagを再生する必要があるが、ROS の仕様上、1 回の launch で、複数のbagを利用することは出来ない。
1つのbag、すなわち1つのt4_datasetに対して launch を 1 回叩くことなるので、データベース評価では、含まれるデータセットの数だけ launch を実行する必要がある。

データベース評価は1回のlaunchで評価できないため、perception では、result.jsonl の他に scene_result.pkl というファイルを出力する。
pickle ファイルは python のオブジェクトをファイルとして保存したものであり、perception_eval の PerceptionEvaluationManager.frame_results を保存している。
pickleファイルに記録したobjectをすべて読み込み、datasetの平均の指標を出力することでデータセット評価が行える。

### データベース評価の結果ファイル

シナリオに複数のdatasetを記述したデータベース評価の場合には、結果出力先ディレクトリにdatabase_result.jsonというファイルが出力される。

形式はメトリクスのフォーマットと同じ
