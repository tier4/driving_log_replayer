# Run log_evaluator evaluation

```shell
cd ${AUTOWARE_WORKSPACE}
source install/setup.bash
ros2 launch log_evaluator log_evaluator.launch.py scenario_path:=${scenario_file}
# example
# ros2 launch log_evaluator log_evaluator.launch.py scenario_path:=$HOME/log_evaluator/yabloc.yaml
```
