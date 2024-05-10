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

The driving_log_replayer cli reads the necessary launch arguments such as sensor_model from the scenario file and generates the launch command.
On the other hand, launch arguments that you want to change at runtime, such as bag playback speed, can be specified by passing them as options.
Multiple arguments can be specified by arranging them in a comma-separated list.

An example is shown below.

```shell
# The playback speed of bag, i.e., simulation time, is set to 0.5x speed.
dlr simulation run -p default -l play_rate:=0.5

# Set bag playback speed to 0.5x and input_pointcloud to /sensing/lidar/concatenated/pointcloud
dlr simulation run -p default -l play_rate:=0.5 -l input_pointcloud:=/sensing/lidar/concatenated/pointcloud

# Set perception_mode to camera_lidar_fusion
dlr simulation run -p default -l perception_mode:=camera_lidar_fusion

# Dry-run mode to obtain metrics without creating a scenario file
# set bag, map, sensor_model, vehicle_model, [and vehicle_id] from arguments
dlr simulation dry-run -p ${profile} -u ${use_case} -l sensor_model:=${sensor_model} -l vehicle_model:=${vehicle_model} -l map_path:=${map_path} -l input_bag:=${bag_path} [-l vehicle_id:=${vehicle_id}]

# Command example
# The -p option is used to determine the output destination for bag and result.jsonl. If omitted, the output is sent to the output_directory of the default profile.
# Currently, use_case is only annotationless_perception, so if -u is omitted, it automatically becomes annotationless_perception.
dlr simulation dry-run -l input_bag:=$HOME/dlr_data/auto/annotationless/sample/input_bag -l sensor_model:=sample_sensor_kit -l vehicle_model:=sample_vehicle -l map_path:=$HOME/map/sample_map
```

The arguments that can be specified can be displayed by using the -s option of ros2 launch.

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

However, some arguments are fixed on the driving_log_replayer side so that they cannot be inconsistently set, such as localization:=false in the localization evaluation, even if they are displayed as launch arguments.
Fixed arguments are ignored even if specified. See the following file for fixed arguments.

```shell
get_autoware_launch fucntion in driving_log_replayer/driving_log_replayer/launch_common.py　
argument of get_autoware_launch in driving_log_replayer/launch/${use_case}.launch.py
```

#### Files created by simulation run

When the simulation run command is executed, a run time directory is created in the output folder of the profile, and the files are output under the directory.
An example of the output file is shown below.

```shell
# t4_datasetを使用しない場合
output_direcotry
└── YYYY-mmDD-HHMMSS               // Execution time
    └── TC01                       // Name of test case
    │   ├── console.log           // Log output to the terminal
    │   ├── result.json　         // Converted result file
    │   ├── result.jsonl          // Original result file
    │   ├── result_bag            // recorded bag
    │   │   ├── metadata.yaml
    │   │   └── result_bag_0.db3
    │   └── run.bash              // Simulation execution command
    └── TC02
...
```

```shell
output_direcotry
└── YYYY-mmDD-HHMMSS                         // Execution time
    └── TC01                                 // Name of test case
│       ├── console.log                     // Log output to the terminal
│       ├── run.bash                        // Simulation execution command
│       ├── DATASET01
│       │   ├── perception_eval_log        // Log files of percepiton_eval
│       │   │   ...
│       │   ├── result.json                // Converted result file
│       │   ├── result.jsonl               // Original result file
│       │   ├── result_archive             // Directory to output evaluation results other than json
│       │   │   └── scene_result.pkl      // Object file of frame_results evaluated by perception_eval
│       │   └── result_bag                 // recorded bag
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
