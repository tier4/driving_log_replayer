## セットアップ手順

本パッケージはAutowareの機能を使用するので、Autowareと一緒にセットアップする。
autowareのルートにあるsimulator.reposにdriving_log_replayerとdriving_log_replayerが依存するperception_eval、ros2_numpyを追加し、autowareと一緒にビルドする。

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

driving_log_replayerのパッケージはros2コマンドを叩いて実行するのではなく、
本パッケージに含まれるdriving_log_replayer_cliを利用して実行するようになっている。
なので、事前にcliをインストールしておく必要がある。

## フォルダ構成、ファイル命名規則

driving_log_replayerを実行するために必要なフォルダ構成、ファイル命名規則について解説する。

ROS2版driving_log_replayerでは、フォルダ構成、ファイル名などを固定にすることで、シナリオに記述するパスや、コマンドに渡す引数を少なくしている。また、テストを連続で回せるようになっている。

### データフォルダ

シミュレーション実行時に使用するリソースを保存しておくフォルダ。

各ユースケース毎に、シナリオと、bag、datasetを配置する。

### localization, performance_diagのデータフォルダ構成

```shell
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

```shell
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

```shell
map
│
├── LocalMapPathName         // ローカルでの任意のフォルダ名
│   ├── lanelet2_map.osm    // laneletファイル
│   └── pointcloud_map.pcd  // pcdファイル

```

### cli設定

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
