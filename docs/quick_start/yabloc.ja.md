# YabLoc自己位置推定機能の評価

## 準備

1. サンプルのシナリオのコピー

   ```bash
   mkdir -p ~/driving_log_replayer_data/yabloc/sample
   cp -r ~/autoware/src/simulator/driving_log_replayer/sample/yabloc/scenario.yaml ~/driving_log_replayer_data/yabloc/sample
   ```

2. サンプルのデータセットをコピー

   [Google Drive Link](https://drive.google.com/file/d/1UqULyfidxcA5JidfHWAsSqNy8itampAX/view)からRosbagをダウンロードし、下記のコマンドを実行してください。

   ```bash
   unzstd yabloc_autoware_test_made_in_awsim_0.db3.zst
   mkdir input_bag
   mv yabloc_autoware_test_made_in_awsim_0.db3 input_bag
   ros2 bag reindex input_bag -s sqlite3
   mv input_bag ~/driving_log_replayer_data/yabloc/sample
   ```

3. 入力の bag ファイルのフィルタとスライス処理

   ```bash
   source ~/autoware/install/setup.bash
   cd ~/driving_log_replayer_data/yabloc/sample
   ros2 bag filter input_bag -o filtered_bag -x "/clock"
   ros2 bag slice filtered_bag -o sliced_bag -e 580
   rm -rf input_bag
   rm -rf filtered_bag
   mv sliced_bag input_bag
   ```


## 実行方法

1. シミュレーションの実行

   ```bash
   driving_log_replayer simulation run -p yabloc --rate 0.5
   ```

2. 結果の確認

   以下のような結果がターミナルに表示されます。

   ```bash
   test case 1 / 1 : use case: sample
   --------------------------------------------------
   TestResult: Passed
   Passed: YabLoc Availability (Passed): OK
   ```
