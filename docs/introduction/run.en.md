# How to run

To run driving_log_replayer, the autoware workspace must have been built, the cli installed, and the data to be evaluated prepared in advance.

## Command

The following is a description of the driving_log_replayer_cli commands.

After installing driving_log_replayer_cli, the command driving_log_replayer can be executed in the terminal.
The driving_log_replayer command has subcommands.
The arguments required for each command can be displayed by specifying the --help option.

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

## cli subcommand

The following subcommands exist

- configure
- simulation

### driving_log_replayer configure

Command to manipulate the configuration file .driving_log_replayer.config.toml.

```shell
# Set data_directory, output_directory, and autoware_path to the profile name specified by -p.
# If -p is omitted, default is specified for the profile name.
driving_log_replayer configure register -d ${data_directory} -o ${output_directory} -a ${autoware_path} [-p ${profile}]
```

### driving_log_replayer simulation

Command to run simulation

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

See [documentation site](https://docs.web.auto/developers-guides/wasim/use-cases/run-simulations-locally/) for usage.

Since wasim downloads and executes scenarios from Autoware Evaluator, it can only execute scenarios that are already registered in the cloud environment.
For scenarios not registered in the cloud, use driving_log_replayer_cli.
