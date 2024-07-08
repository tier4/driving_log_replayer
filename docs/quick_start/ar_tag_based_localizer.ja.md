# ArTagBasedLocalizer自己位置推定機能の評価

## 準備

1. サンプルのシナリオのコピー

   ```shell
   mkdir -p ~/log_evaluator_data/ar_tag_based_localizer/sample
   cp -r ~/autoware/src/simulator/log_evaluator/sample/ar_tag_based_localizer/scenario.yaml ~/log_evaluator_data/ar_tag_based_localizer/sample
   ```

2. サンプルのbagと地図のコピー

   ```shell
   cp -r ~/log_evaluator_data/sample_bag/ar_tag_based_localizer/input_bag ~/log_evaluator_data/ar_tag_based_localizer/sample
   cp -r ~/log_evaluator_data/sample_bag/ar_tag_based_localizer/map ~/log_evaluator_data/ar_tag_based_localizer/sample
   ```

## 実行方法

1. シミュレーションの実行

   ```shell
   dlr simulation run -p ar_tag_based_localizer -l play_rate:=0.5
   ```

2. 結果の確認

   以下のような結果がターミナルに表示されます。

   ```shell
   test case 1 / 1 : use case: sample
   --------------------------------------------------
   TestResult: Passed
   Passed: ArTagBasedLocalizer Availability (Success): Detected 1 AR tags
   ```
