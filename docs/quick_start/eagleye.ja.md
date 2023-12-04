# Eagleye自己位置推定機能の評価

## 準備

1. シナリオをコピーする

   ```shell
   mkdir -p ~/driving_log_replayer_data/eagleye/sample
   cp -r ~/autoware/src/simulator/driving_log_replayer/sample/eagleye/scenario.yaml ~/driving_log_replayer_data/eagleye/sample
   ```

2. mapデータをdownloadする

   [AWSIM Quick Start Demo page](https://tier4.github.io/AWSIM/GettingStarted/QuickStartDemo/) のページから地図ファイル (.pcd and .osm) をダウンロードし `$HOME/autoware_map/awsim-shinjuku` に設置してください。

3. bagデータをdownloadする

   ROSBAGをダウンロードしてください： [Google Drive Link](https://drive.google.com/file/d/1Zgv9eP0j2hAgTj7pW8n-YaECPQGGQjO2/view)。
   その後、下記のコマンドで解答してください。

   ```shell
   unzip awsim_eagleye_rosbag.zip
   mv input_bag ~/driving_log_replayer/eagleye/sample
   ```

## 実行方法

1. シミュレーションの実行

   ```shell
   dlr simulation run -p eagleye --rate 0.5
   ```

2. 結果の確認

   以下のような結果がターミナルに表示されます。

   ```shell
   test case 1 / 1 : use case: sample
   --------------------------------------------------
   TestResult: Passed
   Passed: Eagleye Availability (Passed): OK
   ```
