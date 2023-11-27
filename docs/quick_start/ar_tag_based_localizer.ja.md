# ArTagBasedLocalizer自己位置推定機能の評価

## 準備

1. サンプルのシナリオのコピー

   ```shell
   mkdir -p ~/driving_log_replayer_data/ar_tag_based_localizer/sample
   cp -r ~/autoware/src/simulator/driving_log_replayer/sample/ar_tag_based_localizer/scenario.yaml ~/driving_log_replayer_data/ar_tag_based_localizer/sample
   ```

2. サンプルのデータセットをコピー

   [Google Drive Link](https://drive.google.com/file/d/1uMVwQQFcfs8JOqfoA1FqfH_fLPwQ71jK/view)からRosbagをダウンロードし、下記のコマンドを実行してください。

   ```shell
   unzip sample_data_for_artag_awsim.zip
   mv sample_data_for_artag_awsim/rosbag/ ~/driving_log_replayer_data/ar_tag_based_localizer/sample/input_bag
   mv sample_data_for_artag_awsim ~/driving_log_replayer_data/ar_tag_based_localizer/sample/map
   ```

3. 入力の bag ファイルのフィルタとスライス処理

"/clock"トピックは削除する必要があります。
また、デフォルトで準備されているrosbagファイルはAWSIMで生成したデータであり、タイムスタンプはStart:68.666, End:199.516の、約130秒のデータとなっています。長いので適宜カットしてください。例では126.5秒の3つ目のARタグが映るところで切っています。

```shell
source ~/autoware/install/setup.bash
cd ~/driving_log_replayer_data/ar_tag_based_localizer/sample
ros2 bag filter input_bag -o filtered_bag -x "/clock"
ros2 bag slice filtered_bag -o sliced_bag -e 126.5
rm -rf input_bag
rm -rf filtered_bag
mv sliced_bag input_bag
```

## 実行方法

1. シミュレーションの実行

   ```shell
   dlr simulation run -p ar_tag_based_localizer --rate 0.5
   ```

2. 結果の確認

   以下のような結果がターミナルに表示されます。

   ```shell
   test case 1 / 1 : use case: sample
   --------------------------------------------------
   TestResult: Passed
   Passed: ArTagBasedLocalizer Availability (Success): Detected 1 AR tags
   ```
