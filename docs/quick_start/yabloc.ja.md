# YabLoc自己位置推定機能の評価

## 準備

1. サンプルのシナリオのコピー

   ```shell
   mkdir -p ~/log_evaluator_data/yabloc/sample
   cp -r ~/autoware/src/simulator/log_evaluator/sample/yabloc/scenario.yaml ~/log_evaluator_data/yabloc/sample
   ```

2. サンプルのbagをコピー

   ```shell
   cp -r ~/log_evaluator_data/sample_bag/yabloc/input_bag ~/log_evaluator_data/yabloc/sample
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
