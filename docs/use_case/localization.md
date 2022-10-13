== ユースケース：自己位置推定の評価

Autowareの自己位置推定(localization)が安定して動作しているかを評価する。

Autowareでは自己位置推定の確からしさを示す以下の指標がpublishされており、これらの値がシナリオで指定した値より大きかどうかで自己位置推定が安定しているかを判断する。
また、NDT Scan Matchingが収束しているかを判定するため、NDTとEKFで計算されたposeの横方向の距離誤差が一定以内に収まっているかを判定する。


=== 評価方法
driving_log_replayer/launch/localization.launch.pyを用いて、評価用のノードをautoware_launchのlogging_simulator.launchと一緒に立ち上げる。

=== 評価ノードが使用するTopicとデータ型

* subscribe

|===
|topic名|データ型

|/localization/pose_estimator/transform_probability|tier4_debug_msgs::msg::Float32Stamped
|/localization/pose_estimator/nearest_voxel_transformation_likelihood|tier4_debug_msgs::msg::Float32Stamped
|/localization/pose_estimator/pose|geometry_msgs::msg::PoseStamped
|/localization/kinematic_state|nav_msgs::msg::Odometry
|/tf|tf2_msgs/msg/TFMessage
|/localization/util/downsample/pointcloud|sensor_msgs::msg::PointCloud2
|/localization/pose_estimator/points_aligned|sensor_msgs::msg::PointCloud2
|/localization/pose_estimator/exe_time_ms|tier4_debug_msgs::msg::Float32Stamped
|/localization/pose_estimator/iteration_num|tier4_debug_msgs::msg::Int32Stamped
|===


* publish

|===
|topic名|データ型

|/driving_log_replayer/localization/lateral_distance|example_interfaces::msg::Float64
|/initialpose|geometry_msgs::msg::PoseWithCovarianceStamped
|===

==== logging_simulator.launchに渡す引数
autowareの処理を軽くするため、評価に関係のないモジュールはlaunchの引数にfalseを渡すことで無効化する。以下を設定している。

* planning: false
* control: false

NDTの信頼度を以下の2つのtopicのうち、シナリオで設定した方式で評価する。

* /localization/pose_estimator/transform_probability
* /localization/pose_estimator/nearest_voxel_transformation_likelihood

NDTの収束性を以下から評価する

* /localization/pose_estimator/pose

上記のトピックを処理して、以下のいずれかとして評価される。

ただし評価開始は、/localization/pose_estimator/transform_probability > 0 もしくは /localization/pose_estimator/nearest_voxel_transformation_likelihood > 0になった時点からとする。

==== 信頼度正常
/localization/pose_estimator/transform_probability、または/localization/pose_estimator/nearest_voxel_transformation_likelihood のdataがシナリオに記述したAllowableLikelihood以上の場合

==== 信頼度異常
/localization/pose_estimator/transform_probability、または/localization/pose_estimator/nearest_voxel_transformation_likelihood のdataがシナリオに記述したAllowableLikelihood未満の場合

==== 収束正常
以下の3つの条件を全て満たす場合

* /localization/pose_estimator/poseと /localization/pose_twist_fusion_filter/poseから横方向の距離を計算して、シナリオに記述したAllowableDistance以下
* /localization/pose_estimator/exe_time_msが、シナリオに記述したAllowableExeTimeMs以下
* /localization/pose_estimator/iteration_numが、シナリオに記述したAllowableIterationNum以下

==== 収束異常
収束正常の条件を満たさない場合

==== 評価フロー
1. launchで評価ノード(localization_evaluator_node)とlogging_simulator.launch、ros2 bag playを立ち上げる
2. bagから出力されたセンサーデータをautowareが受け取って、自己位置推定を行う
3. 評価ノードが/localization/pose_estimator/transform_probability、または/localization/pose_estimator/nearest_voxel_transformation_likelihood、及び/localization/pose_estimator/poseをsubscribeして、コールバックで評価を行う

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

==== 入力rosbagに含まれてはいけないtopic
- /clock

=== evaluation
評価に必要な情報を述べる。

==== シナリオフォーマット
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

==== 評価結果ファイルフォーマット
localizationでは、収束性と信頼度の2つを評価しているので、行毎に収束性または信頼度のどちらかの結果が入っている。
Resultは収束性と信頼度両方のパスしていればtrueでそれ以外はfalse失敗となる。

以下に、それぞれの評価の例を記述する。
ただし、<<result_format.adoc#sec-result-format>>で解説済みの共通部分については省略する。

収束性の結果(Frameの中にConvergence項目がある場合)

```json
{
  "Frame": {
    "Convergence": {
      "Result": "Success or Fail",
      "Info": [
        {
          "LateralDistance": "評価に使用したndtとekfの横方向の距離差",
          "HorizontalDistance": "ndtとekfの水平距離。参考値",
          "ExeTimeMs": "評価に使用したndtの計算にかかった時間",
          "IterationNum": "評価に使用したndtの再計算回数"
        }
      ]
    }
  }
}
```

信頼度の結果(FrameにReliabilityの項目がある場合)

```json
{
  "Frame": {
    "Reliability": {
      "Result": "Success or Fail",
      "Info": [
        {
          "Value": "評価に使用したNVTL or TPの値",
          "Reference": "評価に使用しなかった尤度。参考値。ValueがNVTLならTPが入る"
        }
      ]
    }
  }
}
```
