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

## Run driving_log_replayer with wasim

If you have access rights to [Autoware Evaluator](https://docs.web.auto/user-manuals/evaluator/introduction) provided by TIER IV,
you can also use [wasim](https://docs.web.auto/developers-guides/wasim/introduction).

Please see the [wasim documentation site](https://docs.web.auto/developers-guides/wasim/use-cases/run-simulations-locally/) for an example of tool usage.

Since wasim downloads and executes scenarios from Autoware Evaluator, it can only execute scenarios that are already registered in the cloud environment.
For scenarios not registered in the cloud, use `driving_log_replayer_cli`.
