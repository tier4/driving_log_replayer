# Annotationless認識機能の評価

## 実行方法

1. シミュレーションの実行

   ```shell
   ros2 launch log_evaluator dlr.launch.py scenario_path:=$HOME/ros_ws/awf/src/simulator/log_evaluator/sample/annotationless_perception/scenario.yaml output_dir $HOME/dlr_data/out resource_dir:=$HOME/dlr_data/sample_dataset
   dlr simulation run -p annotationless_perception -l play_rate:=0.5
   ```

2. 結果の確認

   以下のような結果がターミナルに表示されます。

   ```shell
   scenario: sample
   --------------------------------------------------
   TestResult: Passed
   Passed:
   CAR (Success)
   BUS (Success)
   PEDESTRIAN (Success)
   BICYCLE (Success)
   MOTORCYCLE (Success)
   TRAILER (Success)
   UNKNOWN (Success)
   TRUCK (Success)
   ```
