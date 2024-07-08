# Eagleye自己位置推定機能の評価

## 準備

1. シナリオをコピーする

   ```shell
   mkdir -p ~/log_evaluator_data/eagleye/sample
   cp -r ~/autoware/src/simulator/log_evaluator/sample/eagleye/scenario.yaml ~/log_evaluator_data/eagleye/sample
   ```

2. サンプルのbagをコピー

   ```shell
   cp -r ~/log_evaluator_data/sample_bag/eagleye/input_bag ~/log_evaluator_data/eagleye/sample
   ```

## 実行方法

1. シミュレーションの実行

   ```shell
   dlr simulation run -p eagleye -l play_rate:=0.5
   ```

2. 結果の確認

   以下のような結果がターミナルに表示されます。

   ```shell
   test case 1 / 1 : use case: sample
   --------------------------------------------------
   TestResult: Passed
   Passed: Eagleye Availability (Passed): OK
   ```
