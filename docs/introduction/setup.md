## セットアップ手順
本パッケージはAutowareの機能を使用するので、Autowareと一緒にセットアップする。
autowareのルートにあるautoware.reposにdriving_log_replayerとdriving_log_replayerが依存するperception_eval、ros2_numpyを追加することで、autowareと一緒にビルドされて使用することが出来る。

```yaml
simulator/driving_log_replayer:
  type: git
  url: https://github.com/tier4/driving_log_replayer.git
  version: main
simulator/perception_eval:
  type: git
  url: https://github.com/tier4/autoware_perception_evaluation.git
  version: main
simulator/vendor/ros2_numpy:
  type: git
  url: https://github.com/Box-Robotics/ros2_numpy.git
  version: humble
```

## driving_log_replayer利用フロー

. 評価用のbagを実車で取得する
. 取得したbagを必要な時間、topicだけ残るようにフィルタする
.. ROS2の場合はgalacticでも未だにfilterコマンドが実装されていないのでtier4で開発した<<ros2bag_extensions>>を使用する
.. 収録時にautowareが出力したトピックを落としてセンサートピックだけを残す
.. 走行前や走行後の評価に不要な時間をカットする(ただし、初期位置位置合わせに3秒程度かかるので走行開始前の3秒程度は残しておく)
. シナリオを作成する。
.. sampleフォルダ内にシナリオの例あり
.. 記述方法はフォーマット定義<<scenario_format.adoc#sec-scenario-format>>を参照
. ユースケースがobstacle_segmentation, perceptionの場合、t4_datasetへの変換に対応したアノテーションツールでアノテーションを実施する。
.. t4_dataset変換ツールは公開準備中
. 評価を実行する。

## フォルダ構成、ファイル命名規則

driving_log_replayerを実行するために必要なフォルダ構成、ファイル命名規則について解説する。

ROS2版driving_log_replayerでは、フォルダ構成、ファイル名などを固定にすることで、シナリオに記述するパスや、コマンドに渡す引数を少なくしている。また、テストを連続で回せるようになっている。

### データフォルダ

シミュレーション実行時に使用するリソースを保存しておくフォルダ。

各ユースケース毎に、シナリオと、bag、datasetを配置する。

### localization, performance_diagのデータフォルダ構成

```
driving_log_replayer_data                           // 複数個のシナリオを収めた親ディレクトリ 任意の名称をつけてよい
│
├── S001                           // シナリオディレクトリ 任意の名称をつけてよい
│   ├── scenario.yaml             // シナリオ
│   └── input_bag                 // センサーデータが収録された入力用のbag
│       ├── input_bag_0.db3       // bagの実態
│       └── metadata.yaml         // bagのmetadata
│
├── S002                           // シナリオディレクトリ S001と構成は同じ
...

```

### obstacle_segmentation, perceptionのデータフォルダ構成

```
driving_log_replayer_data                            // 複数個のシナリオを収めた親ディレクトリ 任意の名称をつけてよい
│
├── S001                            // シナリオディレクトリ 任意の名称をつけてよい
│   ├── scenario.yaml                   // シナリオ
│   └── t4_dataset
│       ├── T4D001                 // t4_datasetディレクトリ、sensingの場合は1個
│       │   ├── annotation
│       │   ├── data
│       │   │   ├── CAM_BACK
│       │   │   ├── CAM_BACK_LEFT
│       │   │   ├── CAM_BACK_RIGHT
│       │   │   ├── CAM_FRONT
│       │   │   ├── CAM_FRONT_LEFT
│       │   │   ├── CAM_FRONT_RIGHT
│       │   │   └── LIDAR_CONCAT
│       │   └── input_bag
│       └── T4D002                 // t4_datasetディレクトリ、perceptionの場合は複数個持てる
│           ├── annotation
│           ├── data
│           │   ├── CAM_BACK
│           │   ├── CAM_BACK_LEFT
│           │   ├── CAM_BACK_RIGHT
│           │   ├── CAM_FRONT
│           │   ├── CAM_FRONT_LEFT
│           │   ├── CAM_FRONT_RIGHT
│           │   └── LIDAR_CONCAT
│           └── input_bag
│          ...
│
├── S002                            // シナリオディレクトリ S001と構成は同じ
```

### マップフォルダ
シミュレーション実行時に使用する地図をまとめて保存しておくフォルダ。

```
map
│
├── LocalMapPathName         // ローカルでの任意のフォルダ名
│   ├── lanelet2_map.osm    // laneletファイル
│   └── pointcloud_map.pcd  // pcdファイル

```

=# 実行コマンド
driving_log_replayerパッケージの実行コマンドについて解説する。

driving_log_replayerのパッケージはros2コマンドを叩いて実行するのではなく、
本パッケージに含まれるdriving_log_replayer_cliを利用して実行するようになっている。
なので、事前にcliをインストールしておく必要がある。

driving_log_replayer_cliをインストールすると、ターミナルでdriving_log_replayerというコマンドが実行できるようになる。
driving_log_replayerコマンドは、サブコマンドを持っている。
各コマンドに必要な引数は--helpオプションを指定すると表示できるようになっている。

```shell
# driving_log_replayer top level help
driving_log_replayer --help

# show version
driving_log_replayer --version

# show subcommand help
driving_log_replayer subcommand --help

# show subsubcommand help
driving_log_replayer subcommand subsubcommand --help
```

また、TIER IVが提供しているlink:https://docs.web.auto/user-manuals/evaluator/introduction[Autoware Evaluator]へ
アクセス権がある場合はlink:https://docs.web.auto/developers-guides/wasim/introduction[wasim]を利用することもできる。

driving_log_replayer_cliは、Autoware Evaluatorを使用しない場合のローカルテスト用のrunnerとして利用できる。
wasimは、Autoware Evaluatorに登録済みのシナリオをダウンロードして実行するので、クラウド環境に登録済みのシナリオしか実行出来ないという違いがある。

==# cli設定
driving_log_replayer_cliでは、cliに渡す引数を少なくするために引数に指定するディレクトリを設定ファイルに記載し設定ファイルから読み込む形式を取る。

よってcliを使う前に以下の形式で$HOME/.driving_log_replayer.config.tomlファイルを作成しておく。
手動で作成、もしくはdriving_log_replayer configureコマンドで作成する。

profileは最低1つ必要で、1つはdefaultという名前である必要がある。

以降で説明するコマンドで-p ${profile}でprofile名を指定すると、プロファイルに指定した設定が読み込まれる。
複数のautowareを切り替えて利用することが出来、プロファイルを何も指定しない場合はdefaultが使用される。

```toml
[profile]
data_directory = "シミュレーションの入力に使うデータフォルダのパス"
output_directory = "シミュレーションの結果を出力するフォルダのパス"
autoware_path = "autowareのprojフォルダのパス"
```

設定例

```toml
# defaultは必ず必要、profile名を省略したときに選択される
[default]
data_directory = "$HOME/data/x1"
output_directory = "$HOME/out/x1"
autoware_path = "$HOME/autoware.proj.x1"

[x2]
data_directory = "$HOME/data/x2"
output_directory = "$HOME/out/x2"
autoware_path = "$HOME/autoware.proj.x2"

[xx1]
data_directory = "$HOME/data/xx1"
output_directory = "$HOME/out/xx1"
autoware_path = "$HOME/autoware.proj.xx1"
```

==# cliサブコマンド
サブコマンドとして以下が存在する

* configure
* simulation

==# driving_log_replayer configure
設定ファイル.driving_log_replayer.config.tomlを操作するコマンド。

```shell
# -pで指定したprofile名デフォルト値defaultにdata_directory、output_directory、autoware_pathを設定する
driving_log_replayer configure register -d ${data_directory} -o ${output_directory} -a ${autoware_path} [-p ${profile}]
```

==# driving_log_replayer simulation
simulation実行に利用する。

```shell
# simulation 実行、jsonlとjsonの両方の結果ファイルが出力される
driving_log_replayer simulation run -p ${profile}

# simulation 実行、jsonlをjsonに変換しない
driving_log_replayer simulation run -p ${profile} --no-json

# 結果の確認、output_directory以下の結果ファイルのサマリーを表示する
driving_log_replayer simulation show-result ${output_directory}

# 結果ファイルをjson変換、クラウドで実行したjsonlやno-jsonで実行した結果ファイルを変換する
driving_log_replayer simulation convert-result ${output_directory}
```

==# wasimによるdriving_log_replayer実行

link:https://docs.web.auto/developers-guides/wasim/use-cases/run-simulations-locally/[ドキュメントサイト]を参照
