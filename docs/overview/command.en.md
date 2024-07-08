# Command

The log_evaluator can be started by specifying the scenario path.

```shell
ros2 launch log_evaluator log_evaluator.launch.py scenario_path:=${scenario_path} [output_dir:=${output_dir} dataset_dir:=${dataset_dir}]
```

## Run log_evaluator with wasim

If you have access rights to [Autoware Evaluator](https://docs.web.auto/user-manuals/evaluator/introduction) provided by TIER IV,
you can also use [wasim](https://docs.web.auto/developers-guides/wasim/introduction).

Please see the [wasim documentation site](https://docs.web.auto/developers-guides/wasim/use-cases/run-simulations-locally/) for an example of tool usage.

Since wasim downloads and executes scenarios from Autoware Evaluator, it can only execute scenarios that are already registered in the cloud environment.
