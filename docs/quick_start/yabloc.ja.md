# YabLoc自己位置推定機能の評価

## 準備

1. サンプルのシナリオのコピー

   ```shell
   mkdir -p ~/driving_log_replayer_data/yabloc/sample
   cp -r ~/autoware/src/simulator/driving_log_replayer/sample/yabloc/scenario.yaml ~/driving_log_replayer_data/yabloc/sample
   ```

2. サンプルのbagをコピー

   ```shell
   cp -r ~/driving_log_replayer_data/sample_bag/yabloc/input_bag ~/driving_log_replayer_data/yabloc/sample
   ```

## 実行方法

1. シミュレーションの実行

   ```shell
   dlr simulation run -p yabloc -l play_rate:=0.5
   ```

2. 結果の確認

   以下のような結果がターミナルに表示されます。

   ```shell
   test case 1 / 1 : use case: sample
   --------------------------------------------------
   TestResult: Passed
   Passed: YabLoc Availability (Passed): OK
   ```
