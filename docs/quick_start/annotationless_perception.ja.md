# Annotationless認識機能の評価

## 準備

1. サンプルのシナリオのコピー

   ```shell
   mkdir -p ~/driving_log_replayer_data/annotationless_perception/sample
   cp -r ~/autoware/src/simulator/driving_log_replayer/sample/annotationless_perception/scenario.yaml ~/driving_log_replayer_data/annotationless_perception/sample
   ```

2. サンプルのデータセットをコピー

   ```shell
   cp -r ~/driving_log_replayer_data/sample_dataset/input_bag ~/driving_log_replayer_data/annotationless_perception/sample
   ```

## 実行方法

1. シミュレーションの実行

   ```shell
   dlr simulation run -p annotationless_perception  -l "play_rate:=0.5"
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
   TRUCK (Success)
   ```
