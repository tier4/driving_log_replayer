# Preparation

The following steps are required to use driving_log_replayer.

1. Build driving_log_replayer with Autoware
2. Install driving_log_replayer_cli
3. Create the cli configuration file
4. Place the scenario and dataset for evaluation in the specified directory (described below)

## build driving_log_replayer

This package uses Autoware's functionalities and depends on packages which are implemented there so it should be set up with Autoware.
To build the `driving_log_replayer` with an Autoware please add the following dependencies into `simulator.repos` file in the Autoware repository.

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
Since cli parses the scenario, sets the launch arguments, and executes it, it is necessary to install CLI. The CLI can be installed with the following command:
The actual ros2 launch command is displayed in the terminal, and the executed command is saved as a file named run.bash in the output folder.

```shell
pipx install git+https://github.com/tier4/driving_log_replayer.git
```

### Set up cli

In order to reduce the number of arguments to be passed to `driving_log_replayer_cli`, the directories specified as arguments are described in a configuration file.

Therefore, before using the cli, create a $HOME/.driving_log_replayer.config.toml file in the format presented below.
You can create it manually or by using the driving_log_replayer configure command.

At least one profile is required, and one must be named default.

Specifying the profile name with `-p ${profile_name}` in the commands described below will load the settings specified in the profile.
You can switch between multiple autoware profiles, and if no profile is specified, default is used.

```toml
[profile_name]
data_directory = "path of the data folder used as input for the simulation"
output_directory = "path of the folder to output the simulation results"
autoware_path = "path of the autoware workspace folder"
```

example setting

```toml
# default is required and selected when profile name is omitted
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

## Folder structure and file naming rules

This section describes the folder structure and file naming rules expected by `driving_log_replayer`.

In `driving_log_replayer` the folder structure and file names are fixed to reduce the number of paths to be described in the scenario and the arguments to be passed to the command.
Also, by placing multiple folders in `data_directory` multiple tests can be executed in succession.

### Data Folder

A folder where resources used in the simulation are stored.

For each test case: a scenario, rosbag, and dataset are placed.

### Data Folder Structure for localization and performance_diag

```shell
driving_log_replayer_data             // .driving_log_replayer.config の data_directory
│
├── TC001                          // Test case directory. Directry name can be named arbitrarily
│   ├── scenario.yaml             // Scenario
│   └── input_bag                 // Bag for input containing sensor data
│       ├── input_bag_0.db3       // Binary file of bag
│       └── metadata.yaml         // Metadata file of bag
│
├── TC002                           // Test case directory. Same structure as TC001
...

```

### Data Folder Structure for obstacle_segmentation and perception

```shell
driving_log_replayer_data              // .driving_log_replayer.config の data_directory
│
├── TC001                           // Test case directory. Directry name can be named arbitrarily
│   ├── scenario.yaml              // Scenario
│   └── t4_dataset
│       ├── T4D001                 // t4_dataset directory. If use case is sensing, t4_dataset is always one.
│       │   ├── annotation
│       │   ├── data
│       │   │   └── LIDAR_CONCAT
│       │   └── input_bag
│       └── T4D002                 // t4_dataset directory. If use case is peception, t4_dataset can be multiple.
│           ├── annotation
│           ├── data
│           │   └── LIDAR_CONCAT
│           └── input_bag
│          ...
│
├── TC002                           // Test case directory. Same structure as TC001
...

```

### Map Folder

A folder where all the maps used in the simulation are stored.

```shell
autoware_map
│
├── LocalMapPath1            // Path specified by LocalMapPath in the scenario
│   ├── lanelet2_map.osm    // lanelet file
│   └── pointcloud_map.pcd  // pcd file
│
├── LocalMapPath2            // Path specified by LocalMapPath in the scenario
...

```
