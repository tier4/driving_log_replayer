# トラブルシューティング

うまく行かないときに確認する。

## 問題の種類

問題のタイプを以下にわける

## Autowareが起動しない

シナリオで指定したsensor_model、vehicle_model、vehicle_idがAutowareに含まれていない
cliのバージョンがdriving_log_replayerのバージョンと一致していない

### Autoware起動後すぐに終了してしまう

シナリオフォーマットが不正な場合に評価不能で終了している

### 評価結果がNoDataとなる

Autowareから評価対象のtopicが出力されていない
QoSの不一致で取得できていない
cuda, cuDNN, TensorRTの不整合が発生していて、perceptionの認識結果が出てこない

### 評価数が異常に少ない

PCの性能不足でAutowareが出力するtopic数が少ない
PCの性能不足でsubscriberがcallbackを処理仕切る前に次のtopicが来て、topicが捨てられる
事前にml modelをengineに変換していない場合

### 終了しない

rosbag playは終了しているのに、ずっとrvizはでっぱなし。コンソールのログからも動いてるように見えない。
途中で実行時エラーでノードが死んでいる可能性がある。ノードが正常に動作していないと

### 途中で終了する

終了しない場合と同様で途中で実行時エラーでノードが死んでいる。

## 原因解析

driving_log_replayerのcliを使った

### run.bash


### console.log

コンソールに表示されているログを保存したもの
エラーが発生していないかを確認する


rosのlogging機能を使って出力されているので、以下のような形式で出力されている。

```shell
[重大度] [時刻] [ロガー名]: メッセージ
```

ローカル環境で.bashrcにRCUTILS_CONSOLE_OUTPUT_FORMATを設定している場合は、設定したformatになる。

===== evaluatorで検索する
logsimの評価ノードは、「ユースケース名_evaluator_node」という名前になっている。
なので、logsimの評価ノードが出しているメッセージを抽出するにはevaluatorという文字を検索すると該当のメッセージを見つけることが出来る。

link:./sample/debug/logsim_runtime_error.log[実行時エラーになる例]
をevaluatorで検索すると、obstacle_detection_evaluator_nodeがjsonのparseに失敗しており、obstacle_detection_evaluator_nodeが死んでlaunchが終了していることがわかる。

```shell
[obstacle_detection_evaluator_node-38] [ERROR] [1640228106.306940806] [logsim.obstacle_detection_evaluator]: detection parse error
・・・
[ERROR] [obstacle_detection_evaluator_node-38]: process has died [pid 486, exit code 1, cmd '/home/autoware/autoware.proj/install/logsim/lib/logsim/obstacle_detection_evaluator_node --ros-args -r __node:=obstacle_detection_evaluator -r __ns:=/logsim --params-file /tmp/launch_params_i_9h9mk1 --params-file /tmp/launch_params_mq4cddau'].
[INFO] [launch.user]: shutdown launch
```

よって、parse errorになった原因であるdetections.jsonを直す必要があるということがわかる。

===== ERRORで検索する
logsimのモジュールはautowareから評価対象のtopicをsubscribeして動いている。
なので、autowareが必要なtopicを出していない場合はlogsimのERRORとしては出てこないケースがある。

link:./sample/debug/process_died.log[autoware側のプロセスが死んでtopicが出ない例]
は点群が一切出なかったケースのログの一部であるが、ERRORで検索するとpointcloud_preprocessorが死んでいる。
autoware側に問題があることがわかり、後のデバッグでスレッドプールを使い切って死んでいることがわかった。

```shell
[ERROR] [component_container_mt-18]: process has died [pid 95, exit code -6, cmd '/opt/ros/galactic/lib/rclcpp_components/component_container_mt --ros-args -r __node:=pointcloud_preprocessor_container -r __ns:=/sensing/lidar/pointcloud_preprocessor --params-file /tmp/launch_params_rh_9gxcs'].
```

===== QoSで検索する
ROS2ではリンクにある通り、publisherとsubscriber間でQoS設定に互換性がないとtopicが出ていても受信されずに捨てられる。
link:https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html[QoS設定]
厄介なことにこれはシステム的にはERRORではなくWARNINGであるため、ERRORで検索しても引っかからないので見逃しがちである。

AutowareEvaluationDashboard上でbagを再生したが何も表示されず、bagをDLしてきてros2 bag infoしたが、bagに必要なtopicが入っていない場合などは、QoSで調べてみると良い。

link:./sample/debug/incompatible_qos.log[QoS非互換でtopicをレコード出来ない例]
をQoSで検索すると、/localization/util/downsample/pointcloudをレコードしようとしたが、QoS非互換で受信出来てないということがわかる。

```shell
[ros2-41] [WARN 1633081042.154102891] [rosbag2_recorder]: New publisher discovered on topic '/localization/util/downsample/pointcloud', offering incompatible QoS. No messages will be sent to it. Last incompatible policy: RELIABILITY_QOS_POLICY
```

なので、この場合は、ros2 bag info /localization/util/downsample/pointcloud -vして、送信側のQoS設定を調べて受信側で互換性が保てる設定を追加することが必要だとわかった。


### result.jsonl

### resutl_bag

ros2 bag info resutl_bagを実行して、評価対象のtopicが記録されているか調べる。
ここでtopicが記録されていなかったら、Autowareが出力していないか、QoSの不一致で捨てられている
