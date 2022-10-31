# 自己位置推定の評価

Autoware の自己位置推定(localization)が安定して動作しているかを評価する。

自己位置推定の評価では NDT の信頼度と収束性を評価する。

## 評価方法

localization.launch.py を使用して評価する。
launch を立ち上げると以下のことが実行され、評価される。

1. launch で評価ノード(localization_evaluator_node)と logging_simulator.launch、ros2 bag play を立ち上げる
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

ただし収束性はNDTが収束してから評価開始とし、収束の判定は/localization/pose_estimator/transform_probability > 0 もしくは /localization/pose_estimator/nearest_voxel_transformation_likelihood > 0 を用いる。

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

## 評価ノードが使用する Topic 名とデータ型

- subscribe

| topic 名                                                             | データ型                              |
| -------------------------------------------------------------------- | ------------------------------------- |
| /localization/pose_estimator/transform_probability                   | tier4_debug_msgs::msg::Float32Stamped |
| /localization/pose_estimator/nearest_voxel_transformation_likelihood | tier4_debug_msgs::msg::Float32Stamped |
| /localization/pose_estimator/pose                                    | geometry_msgs::msg::PoseStamped       |
| /localization/kinematic_state                                        | nav_msgs::msg::Odometry               |
| /tf                                                                  | tf2_msgs/msg/TFMessage                |
| /localization/util/downsample/pointcloud                             | sensor_msgs::msg::PointCloud2         |
| /localization/pose_estimator/points_aligned                          | sensor_msgs::msg::PointCloud2         |
| /localization/pose_estimator/exe_time_ms                             | tier4_debug_msgs::msg::Float32Stamped |
| /localization/pose_estimator/iteration_num                           | tier4_debug_msgs::msg::Int32Stamped   |

- publish

| topic 名                                            | データ型                                      |
| --------------------------------------------------- | --------------------------------------------- |
| /driving_log_replayer/localization/lateral_distance | example_interfaces::msg::Float64              |
| /initialpose                                        | geometry_msgs::msg::PoseWithCovarianceStamped |

## logging_simulator.launch に渡す引数

autoware の処理を軽くするため、評価に関係のないモジュールは launch の引数に false を渡すことで無効化する。以下を設定している。

- planning: false
- control: false

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

### 入力 rosbag に含まれてはいけない topic

- /clock

## evaluation

評価に必要な情報を述べる。

### シナリオフォーマット

```yaml
Evaluation:
  UseCaseName: localization
  UseCaseFormatVersion: 1.2.0
  Conditions:
    Convergence: # 収束性評価
      AllowableDistance: 0.2 # 直線距離でこの距離以内だったら収束とみなす
      AllowableExeTimeMs: 100.0 # NDTの計算時間がこの値以下なら成功とみなす
      AllowableIterationNum: 30 # NDTの計算回数がこの値以下なら成功とみなす
      PassRate: 95.0 # 収束性の評価試行回数の内、どの程度(%)評価成功だったら成功とするか
    Reliability: # 信頼度評価
      Method: NVTL # NVTL or TPのどちらで評価を行うか
      AllowableLikelihood: 3.0 # この値以上なら信頼度は正常とみなす
      NGCount: 10 # 信頼度異常が連続でこの回数続いたら信頼度評価失敗とみなす
  InitialPose:
    position:
      x: 16876.271484375
      y: 36087.9453125
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.23490284404117467
      w: 0.9720188546840887
```

### 評価結果フォーマット

localization では、収束性と信頼度の 2 つを評価しているので、行毎に収束性または信頼度のどちらかの結果が入っている。
Result は収束性と信頼度両方のパスしていれば true でそれ以外は false 失敗となる。

以下に、それぞれの評価の例を記述する。
ただし、結果ファイルフォーマットで解説済みの共通部分については省略する。

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
          "Value": "NVTL or TPの値",
          "Reference": "評価に使用しなかった尤度。参考値。ValueがNVTLならTPが入る"
        }
      ]
    }
  }
}
```
