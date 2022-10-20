# Preparation

The following work is required to use driving_log_replayer.

1. Build driving_log_replayer with autoware
2. install driving_log_replayer_cli
3. create the cli configuration file
4. Place the scenario and dataset for evaluation in the specified directory

## build driving_log_replayer

This package uses Autoware functionality, so it should be set up with Autoware.
Add driving_log_replayer and driving_log_replayer dependencies perception_eval and ros2_numpy to simulator.repos in the root of autoware and build it with autoware.

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

## install cli

The driving_log_replayer is invoked by setting the parameters described in the scenario as launch arguments.
Since cli parses the scenario, sets the launch argument, and executes it, it is necessary to install cli.
The actual ros2 launch command is displayed in the terminal, and the executed command is saved as a file named run.bash in the output folder.

```shell
pipx install git+https://github.com/tier4/driving_log_replayer.git
```

### set up cli

In driving_log_replayer_cli, in order to reduce the number of arguments to be passed to cli, the directories specified as arguments are described in a configuration file, which is then read from the configuration file.

Therefore, before using the cli, create a $HOME/.driving_log_replayer.config.toml file in the following format.
You can create it manually or by using the driving_log_replayer configure command.

At least one profile is required, and one must be named default.

Specifying the profile name with -p ${profile} in the commands described below will load the settings specified in the profile.
You can switch between multiple autoware profiles, and if no profile is specified, default is used.

```toml
[profile_name]
data_directory = "path of the data folder used as input for the simulation"
output_directory = "path of the folder to output the simulation results"
autoware_path = "path of the autoware proj folder"
```

設定例

```toml
# defaultは必ず必要、profile名を省略したときに選択される
[default]
data_directory = "$HOME/driving_log_replayer_data/default"
output_directory = "$HOME/driving_log_replayer_output/default"
autoware_path = "$HOME/autoware"

[localization]
data_directory = "$HOME/driving_log_replayer_data/localization"
output_directory = "$HOME/driving_log_replayer_output/localization"
autoware_path = "$HOME/autoware"

[obstacle_segmentation]
data_directory = "$HOME/driving_log_replayer_data/obstacle_segmentation"
output_directory = "$HOME/driving_log_replayer_output/obstacle_segmentation"
autoware_path = "$HOME/autoware"

[perception]
data_directory = "$HOME/driving_log_replayer_data/perception"
output_directory = "$HOME/driving_log_replayer_output/perception"
autoware_path = "$HOME/autoware"
```

## フォルダ構成、ファイル命名規則

driving_log_replayer が期待するフォルダ構成、ファイル命名規則について解説する。

ROS2 版 driving_log_replayer では、フォルダ構成、ファイル名などを固定にすることで、シナリオに記述するパスや、コマンドに渡す引数を少なくしている。また、テストを連続で回せるようになっている。

### データフォルダ

シミュレーション実行時に使用するリソースを保存しておくフォルダ。

各ユースケース毎に、シナリオと、bag、dataset を配置する。

### localization, performance_diag のデータフォルダ構成

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

### obstacle_segmentation, perception のデータフォルダ構成

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
autoware_map
│
├── LocalMapPathName         // ローカルでの任意のフォルダ名
│   ├── lanelet2_map.osm    // laneletファイル
│   └── pointcloud_map.pcd  // pcdファイル

```
