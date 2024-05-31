# Planning Controlの評価

Planning ControlのMetricsが指定の時刻、条件で出力されているか評価する

## 評価方法

`planning_control.launch.py` を使用して評価する。
launch を立ち上げると以下のことが実行され、評価される。

1. launch で評価ノード(`planning_control_evaluator_node`)と `logging_simulator.launch`、`ros2 bag play`コマンドを立ち上げる
2. bag から出力されたセンサーデータを autoware が受け取って、perception モジュールが認識を行う
3. perceptionの結果を使って、planningは `/planning/planning_evaluator/metrics(仮)` に controlは `/control/control_evaluator/metrics`にMetricsを出力する
4. 評価ノードが topic を subscribe して、各基準を満たしているかを判定して結果をファイルに記録する
5. bag の再生が終了すると自動で launch が終了して評価が終了する

## 評価結果

controlが出力するtopicは以下のサンプルのような形式となっている。
[topicサンプル](https://github.com/tier4/driving_log_replayer/blob/main/sample/planning_control/diag_topic.txt)

topic の subscribe 1 回につき、headerの時刻で評価する条件があれば評価され判定結果が出力される。
※headerの時刻に該当する評価条件がない場合は捨てられるのでログも出力されない。

### 正常

評価条件を満たすtopic が (現在時刻-評価開始時刻)\*Hertz\*AllowableRate(=0.95)個以上取得できた場合。

以下の例の場合、現在時刻が2だとすると、1秒からスタートして2秒の時点で (2-1)*10.0=10 topic程度metricsが出てくるはず。
実際にはちょっとぶれるのでAllowableRateをかけて floor(10*0.95)=9個以上 autonomous_emergency_brakingのdecisionの値stopが出ていたら成功
3秒時点では、floor(20\*0.95)=19以上で成功となる。

```yaml
Conditions:
  Hertz: 10.0 # metricsが何Hzで来るか。 (現在時刻-評価開始時刻)* Hertz * AllowableRate(=0.95)以上条件に合致するtopicが出力される必要がある。低レートは弾かれる。AllowableRateは一旦固定
  ControlConditions:
    - TimeRange: { start: 1, end: 3 } # 評価開始時間と終了時刻、endは省略可能で省略した場合はsys.float_info.max
      Module: autonomous_emergency_braking # 評価対象のモジュール
      Value0Key: decision # 評価対象のキー
      Value0Value: stop # 評価対象の値
      DetailedConditions: null # 位置、速度など、追加で判定したい条件。nullの場合はValue0Valueが一致した時点で成功。記載がある場合はDetailedConditionsも条件を満たす必要がある
```

### 異常

正常の条件を満たさないとき

## 評価ノードが使用する Topic 名とデータ型

Subscribed topics:

| Topic name                                | Data type                             |
| ----------------------------------------- | ------------------------------------- |
| /control/control_evaluator/metrics        | diagnostic_msgs::msg::DiagnosticArray |
| /planning/planning_evaluator/metrics (仮) | diagnostic_msgs::msg::DiagnosticArray |

Published topics:

| Topic name | Data type |
| ---------- | --------- |
| N/A        | N/A       |

## logging_simulator.launch に渡す引数

autoware の処理を軽くするため、評価に関係のないモジュールは launch の引数に false を渡すことで無効化する。以下を設定している。

- sensing: false
- localization: false
- perception: true
- planning: true
- control: true

## simulation

シミュレーション実行に必要な情報を述べる。

### 入力 rosbag に含まれるべき topic

| topic 名                               | データ型                                     |
| -------------------------------------- | -------------------------------------------- |
| /gsm8/from_can_bus                     | can_msgs/msg/Frame                           |
| /localization/kinematic_state          | nav_msgs/msg/Odometry                        |
| /localization/acceleration             | geometry_msgs/msg/AccelWithCovarianceStamped |
| /sensing/lidar/concatenated/pointcloud | sensor_msgs/msg/PointCloud2                  |
| /tf                                    | tf2_msgs/msg/TFMessage                       |
| /planning/mission_planning/route       | autoware_planning_msgs/msg/LaneletRoute      |

CAN の代わりに vehicle の topic を含めても良い。

| topic 名                               | データ型                                            |
| -------------------------------------- | --------------------------------------------------- |
| /localization/kinematic_state          | nav_msgs/msg/Odometry                               |
| /localization/acceleration             | geometry_msgs/msg/AccelWithCovarianceStamped        |
| /sensing/lidar/concatenated/pointcloud | sensor_msgs/msg/PointCloud2                         |
| /tf                                    | tf2_msgs/msg/TFMessage                              |
| /planning/mission_planning/route       | autoware_planning_msgs/msg/LaneletRoute             |
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

[サンプル](https://github.com/tier4/driving_log_replayer/blob/main/sample/planning_control/scenario.ja.yaml)参照

### 評価結果フォーマット

[サンプル](https://github.com/tier4/driving_log_replayer/blob/main/sample/planning_control/result.json)参照

以下に、それぞれの評価の例を記述する。
**注:結果ファイルフォーマットで解説済みの共通部分については省略する。**

planning と controlで設定した全ての評価条件で成功している場合にシナリオ全体で成功となる。

```json
{
  "Frame": {
    "CONDITION_INDEX": {
      // 評価条件毎に結果が出力される
      "Result": { "Total": "Success or Fail", "Frame": "Success or Fail" },
      "Info": {
        "TotalPassed": "評価条件をパスしたtopicの総数",
        "RequiredSuccess": "現在時刻で必要な成功数(TotalPassed >= RequiredSuccessでTotalが成功になる)"
      }
    }
  }
}
```
