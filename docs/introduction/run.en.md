# How to run

To run `driving_log_replayer` all the steps described in [Preparation](./preparation.en.md) Section should be previously executed. Also, the rosbag input data should be prepared in advance.

## Command


After installing `driving_log_replayer_cli`, the `driving_log_replayer` command can be executed in the terminal.
The `driving_log_replayer` command has subcommands.
The arguments required for each command can be displayed by specifying the `--help` option.

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

## CLI subcommands

The list of supported subcommands can be found below:

- configure
- simulation

### driving_log_replayer configure

Command to manipulate the configuration file `.driving_log_replayer.config.toml`.

```shell
# Set data_directory, output_directory, and autoware_path to the profile name specified by -p.
# If -p is omitted, default is specified for the profile name.
driving_log_replayer configure register -d ${data_directory} -o ${output_directory} -a ${autoware_path} [-p ${profile}]
```

### driving_log_replayer simulation

Available commands to run the Autoware evaluation:

```shell
# simulation run, both jsonl and json result files are output
driving_log_replayer simulation run -p ${profile}

# simulation run, do not convert jsonl to json
driving_log_replayer simulation run -p ${profile} --no-json

# Check results and display summary of result files under output_directory
driving_log_replayer simulation show-result ${output_directory}

# Convert result files to json
driving_log_replayer simulation convert-result ${output_directory}
```

## Run driving_log_replayer with wasim

If you have access rights to [Autoware Evaluator](https://docs.web.auto/user-manuals/evaluator/introduction) provided by TIER IV,
you can also use [wasim](https://docs.web.auto/developers-guides/wasim/introduction).

Please see the [wasim documentation site](https://docs.web.auto/developers-guides/wasim/use-cases/run-simulations-locally/) for an example of tool usage.

Since wasim downloads and executes scenarios from Autoware Evaluator, it can only execute scenarios that are already registered in the cloud environment.
For scenarios not registered in the cloud, use `driving_log_replayer_cli`.
