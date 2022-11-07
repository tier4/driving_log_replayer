# 認識機能の評価

## 準備

1. サンプルのシナリオのコピー

   ```bash
   mkdir -p ~/driving_log_replayer_data/perception/sample
   cp -r ~/autoware/src/simulator/driving_log_replayer/sample/perception/scenario.yaml ~/driving_log_replayer_data/perception/sample
   ```

2. サンプルのデータセットをコピー

   ```bash
   mkdir -p ~/driving_log_replayer_data/perception/sample/t4_dataset
   cp -r ~/driving_log_replayer_data/sample_dataset ~/driving_log_replayer_data/perception/sample/t4_dataset
   ```

## 実行方法

1. シミュレーションの実行

   ```bash
   driving_log_replayer simulation run -p perception --rate 0.5
   ```

   ![perception](images/perception.png)

2. 結果の確認

   以下のような結果がターミナルに表示されます。
   PC の性能や CPU の負荷状況によってテスト回数が若干異なることがありますが、多少の差は問題ありません。

   ```bash
    test case 1 / 1 : use case: sample
    --------------------------------------------------
    TestResult: Passed
    Passed: 682 / 682 -> 100.00%
   ```
