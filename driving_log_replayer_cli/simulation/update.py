from pathlib import Path

import termcolor

from driving_log_replayer_cli.core.result import load_final_metrics
from driving_log_replayer_cli.core.scenario import backup_scenario_file
from driving_log_replayer_cli.core.scenario import load_scenario
from driving_log_replayer_cli.core.scenario import Scenario


def update_class_conditions(scenario: Scenario, final_metrics: dict, keys_to_update: dict) -> None:
    # Convert comma-separated string to a set for filtering
    keys_set = set(keys_to_update.split(","))

    for class_name, class_data in scenario.Evaluation["Conditions"]["ClassConditions"].items():
        if class_name in final_metrics:
            for metric_name, metric_values in final_metrics[class_name].items():
                # Only update specified metrics
                filtered_values = {k: v for k, v in metric_values.items() if k in keys_set}
                class_data["Threshold"][metric_name] = filtered_values
        else:
            # Remove classes not present in the results data
            del scenario.Evaluation["Conditions"]["ClassConditions"][class_name]


def update_annotationless_scenario_condition(
    scenario_path: Path,
    result_path: Path,
    keys: str,
) -> None:
    # Backup the original file
    backup_file_path = backup_scenario_file(scenario_path)
    termcolor.cprint(f"Original scenario file backed up to: {backup_file_path}", "yellow")

    # Load data
    metrics_data = load_final_metrics(result_path)
    scenario_data = load_scenario(scenario_path)

    # Update scenario file conditions
    update_class_conditions(scenario_data, metrics_data, keys)

    # Save the updated scenario file
    scenario_data.dump(scenario_path)
    termcolor.cprint(f"{scenario_path.as_posix()} updated with new conditions.", "green")
