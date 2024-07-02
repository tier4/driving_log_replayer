# Command

The driving_log_replayer can be started by specifying the scenario path and the output directory of the evaluation results.

```shell
ros2 launch driving_log_replayer dlr.launch.py scenario_path:=${scenario_path} output_dir:=${output_dir}
```

## CLI

The driving_log_replayer_cli used up to version 2 is discontinued.

## Run driving_log_replayer with wasim

If you have access rights to [Autoware Evaluator](https://docs.web.auto/user-manuals/evaluator/introduction) provided by TIER IV,
you can also use [wasim](https://docs.web.auto/developers-guides/wasim/introduction).

Please see the [wasim documentation site](https://docs.web.auto/developers-guides/wasim/use-cases/run-simulations-locally/) for an example of tool usage.

Since wasim downloads and executes scenarios from Autoware Evaluator, it can only execute scenarios that are already registered in the cloud environment.
For scenarios not registered in the cloud, use `driving_log_replayer_cli`.
