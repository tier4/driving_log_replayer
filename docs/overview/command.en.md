# Command

After installing `driving_log_replayer_cli`, the `dlr` command can be executed in the terminal.
The `dlr` command has subcommands.
The arguments required for each command can be displayed by specifying the `--help` option.

```shell
# driving_log_replayer top level help
dlr --help

# show version
dlr --version

# show subcommand help
dlr subcommand --help

# show subsubcommand help
dlr subcommand subsubcommand --help
```

## CLI subcommands

The list of supported subcommands can be found below:

- configure
- simulation

### dlr configure

Command to manipulate the configuration file `.driving_log_replayer.config.toml`.

```shell
# Set data_directory, output_directory, and autoware_path to the profile name specified by -p.
# If -p is omitted, default is specified for the profile name.
dlr configure register -d ${data_directory} -o ${output_directory} -a ${autoware_path} [-p ${profile}]
```

### dlr simulation

Available commands to run the Autoware evaluation:

```shell
# simulation run, both jsonl and json result files are output
dlr simulation run -p ${profile}

# Check results and display summary of result files under output_directory
dlr simulation show-result ${output_directory}

# Convert result files to json
dlr simulation convert-result ${output_directory}
```

#### dlr simulation run launch argument option

driving_log_replayerのcliは、シナリオファイルからsensor_modelなどlaunchに必要な引数を読み取って、launchコマンドを生成する。
しかし、bagの再生速度など、実行時に変更したいlaunchの引数に関してはオプションで渡すことで指定可能である。
複数の引数を指定する場合には、カンマで区切って引数を連結する。

以下に例を示す。

```shell
# bagの再生速度、すなわち、simulation時間を0.5倍速にする
dlr simulation run -p default -l "play_rate:=0.5"

# bagの再生速度を0.5倍速、かつ、input_pointcloudを /sensing/lidar/concatenated/pointcloudにする
dlr simulation run -p default -l "play_rate:=0.5,input_pointcloud:=/sensing/lidar/concatenated/pointcloud"

# perception_modeをcamera_lidar_fusionにする
dlr simulation run -p default -l "perception_mode:=camera_lidar_fusion"
```

指定可能な引数は、ros2 launchの-sオプションを使うことで調べることができる。

```shell
# ❯ ros2 launch driving_log_replayer ${use_case}.launch.py -s
❯ ros2 launch driving_log_replayer localization.launch.py -s
Arguments (pass arguments as '<name>:=<value>'):

    'with_autoware':
        Whether to launch autoware or not
        (default: 'true')

    'scenario_path':
        scenario path
...
```

ただし、launchの引数として表示されていても、localizationの評価でlocalization:=falseといった矛盾した設定が出来ないように、driving_log_replayer側で固定されている引数もある。
固定している引数は指定しても無視される。固定されている引数は、以下のファイルを参照。

```shell
driving_log_replayer/driving_log_replayer/launch_common.pyのget_autoware_launch関数
driving_log_replayer/launch/${use_case}.launch.pyでget_autoware_launchを呼び出しているときの引数
```

#### simulation実行で作成されるファイル

simulation runコマンドを実行するとプロファイルの出力先フォルダに実行時間のディレクトリが作成され、その下にファイルが出力される。
出力例を以下に示す。

```shell
# t4_datasetを使用しない場合
output_direcotry
└── YYYY-mmDD-HHMMSS               // 実行時刻
    └── TC01                       // テストケース名
    │   ├── console.log           // ターミナルに出力されているログ
    │   ├── result.json　         // 変換済み結果ファイル
    │   ├── result.jsonl          // 変換元結果ファイル
    │   ├── result_bag            // recordしたbag
    │   │   ├── metadata.yaml
    │   │   └── result_bag_0.db3
    │   └── run.bash              // シミュレーション実行コマンド
    └── TC02
...
```

```shell
output_direcotry
└── YYYY-mmDD-HHMMSS                         // 実行時刻
    └── TC01                                 // テストケース名
│       ├── console.log                     // ターミナルに出力されているログ
│       ├── run.bash                        // シミュレーション実行コマンド
│       ├── DATASET01
│       │   ├── perception_eval_log        // percepiton_evalのログ
│       │   │   ...
│       │   ├── result.json                // 変換済み結果ファイル
│       │   ├── result.jsonl               // 変換元結果ファイル
│       │   ├── result_archive             // json以外の評価結果を出力するフォルダ
│       │   │   └── scene_result.pkl      // perception_evalで評価したframe_resultsのオブジェクトファイル
│       │   └── result_bag                 // recordしたbag
│       │       ├── metadata.yaml
│       │       └── result_bag_0.db3
│       └── DATASET02
...
```

## Run driving_log_replayer with wasim

If you have access rights to [Autoware Evaluator](https://docs.web.auto/user-manuals/evaluator/introduction) provided by TIER IV,
you can also use [wasim](https://docs.web.auto/developers-guides/wasim/introduction).

Please see the [wasim documentation site](https://docs.web.auto/developers-guides/wasim/use-cases/run-simulations-locally/) for an example of tool usage.

Since wasim downloads and executes scenarios from Autoware Evaluator, it can only execute scenarios that are already registered in the cloud environment.
For scenarios not registered in the cloud, use `driving_log_replayer_cli`.
