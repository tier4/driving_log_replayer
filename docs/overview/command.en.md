# Command

The log_evaluator can be started by specifying the scenario path and the output directory of the evaluation results.

```shell
ros2 launch log_evaluator dlr.launch.py scenario_path:=${scenario_path} output_dir:=${output_dir}
```

## CLI

The log_evaluator_cli used up to version 2 is discontinued.

## Run log_evaluator with wasim

If you have access rights to [Autoware Evaluator](https://docs.web.auto/user-manuals/evaluator/introduction) provided by TIER IV,
you can also use [wasim](https://docs.web.auto/developers-guides/wasim/introduction).

Please see the [wasim documentation site](https://docs.web.auto/developers-guides/wasim/use-cases/run-simulations-locally/) for an example of tool usage.

Since wasim downloads and executes scenarios from Autoware Evaluator, it can only execute scenarios that are already registered in the cloud environment.
For scenarios not registered in the cloud, use `log_evaluator_cli`.
