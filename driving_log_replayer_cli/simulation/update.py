from pathlib import Path

import termcolor

from driving_log_replayer_cli.core.result import load_final_metrics
from driving_log_replayer_cli.core.scenario import backup_scenario_file
from driving_log_replayer_cli.core.scenario import load_scenario
from driving_log_replayer_cli.core.scenario import Scenario


def update_class_conditions(scenario: Scenario, final_metrics: dict, update_method: str) -> None:
    class_cond = scenario.Evaluation["Conditions"]["ClassConditions"]
    for class_name, class_data in class_cond.items():
        if class_name not in final_metrics:
            # Remove classes not present in the results data
            del scenario.Evaluation["Conditions"]["ClassConditions"][class_name]
            continue
        if update_method == "all":
            class_cond[class_name]["Threshold"] = final_metrics[class_name]
        elif update_method == "existing":
            for diag_key, diag_value in class_data["Threshold"].items():
                if final_metrics[class_name].get(diag_key) is None:
                    continue
                metrics_value = final_metrics[class_name].get(diag_key)
                class_cond[class_name]["Threshold"][diag_key] = {
                    k: v for k, v in metrics_value.items() if k in diag_value
                }


def update_annotationless_scenario_condition(
    scenario_path: Path,
    result_path: Path,
    update_method: str,
) -> None:
    if update_method == "keep":
        return
    # Backup the original file
    backup_file_path = backup_scenario_file(scenario_path)
    termcolor.cprint(f"Original scenario file backed up to: {backup_file_path}", "yellow")

    # Load data
    metrics_data = load_final_metrics(result_path)
    scenario_data = load_scenario(scenario_path)

    # Update scenario file conditions
    update_class_conditions(scenario_data, metrics_data, update_method)

    # Save the updated scenario file
    scenario_data.dump(scenario_path)
    termcolor.cprint(f"{scenario_path.as_posix()} updated with new conditions.", "green")
