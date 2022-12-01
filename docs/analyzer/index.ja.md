# Driving Log Replayer Analyzer

Driving Log Replayerで行ったテストの結果ファイルを分析するパッケージ。

## 注意

開発中であり、現状ではobstacle_segmentationのresult.jsonlの分析のみ可能

## インストール方法

- driving_log_replayer_cliと一緒にインストールされる
- driving_log_replayerと一緒にrosのパッケージとしてインストールされる

## 構成

現状、ROSの機能を使用しないpythonパッケージとなっているが、ROSのパッケージとしてもインストールされる。
[perception評価用の依存ライブラリ](https://github.com/tier4/autoware_perception_evaluation)と同じように構成している。

### ROSパッケージにする理由

[Autoware Evaluator](https://docs.web.auto/user-manuals/evaluator/introduction)でビルドして実行されることを考えると、pipなどで別途インストールする構成にしてしまうとクラウド基盤側にライブラリを導入するための特別な処理が必要になる。
ROSのパッケージになっていれば、他のパッケージと自動でインストールされる。

## 使い方

```shell
# pythonのcliコマンドで使う場合
driving_log_replayer_analyzer ${jsonl_path} -c ${config_path}

# ros2 launchから使う。こちらは後で消す予定
ros2 launch driving_log_replayer obstacle_segmentation_analyze.launch.py result_json_path=${jsonl_path}
```

## これからやる

現状では、cliと、ros2のlaunchで同じことをやっているだけに過ぎないが、今後用途を以下のように分けていく。

![usage](./images/future_work.drawio.svg)

### cli

ユーザーがローカルで分析するために利用する
jsonlから分析結果のhtmlや画像ファイルを出力するために利用する

### ROS側

driving_log_replayer_analyzerをjsonl分析のライブラリとしてのみ使用する。
pandasのdata frameをjsonlまたはrosbagで保存しておくことで、Autoware Evaluatorのフロントで可視化するための元データにする。
