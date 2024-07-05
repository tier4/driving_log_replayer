# log evaluator migration tool

Tool to migrate data_directory of driving_log_replayer

## What this tool does

1. Convert driving_log_replayer scenario version 2.2.0 to 3.0.0 format
2. Move input_bag from the same directory as the scenario to the t4_dataset/input_bag directory
3. Move the dataset up one level from the t4_dataset directory on the same level as the scenario
4. remove LocalMapPath from the scenario file and if LocalMapPath exists copy the map to the dataset directory

## usage

```shell
python3 log_evaluator_migration_tool ${driving_log_replayer_data_directory_root}
```
