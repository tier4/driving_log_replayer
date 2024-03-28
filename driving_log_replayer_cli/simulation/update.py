import contextlib
import json
from pathlib import Path

import termcolor
import yaml


def load_metrics_data(jsonl_file_path: Path) -> dict:
    with jsonl_file_path.open("r") as file:
        last_line = file.readlines()[-1]
        with contextlib.suppress(json.JSONDecodeError):
            result_json_dict = json.loads(last_line)
            return result_json_dict["Frame"]["FinalMetrics"]


def update_conditions(scenario_data: dict, metrics_data: dict, keys_to_update: dict) -> None:
    # Convert comma-separated string to a set for filtering
    keys_set = set(keys_to_update.split(","))

    for class_name, class_data in scenario_data["Evaluation"]["Conditions"][
        "ClassConditions"
    ].items():
        if class_name in metrics_data:
            for metric_name, metric_values in metrics_data[class_name].items():
                # Only update specified metrics
                filtered_values = {k: v for k, v in metric_values.items() if k in keys_set}
                class_data["Threshold"][metric_name] = filtered_values
        else:
            # Remove classes not present in the results data
            del scenario_data["Evaluation"]["Conditions"]["ClassConditions"][class_name]


def save_scenario_file(scenario_data: dict, scenario_file_path: Path) -> None:
    with scenario_file_path.open("w") as file:
        yaml.safe_dump(scenario_data, file, sort_keys=False)


def update_annotationless_scenario_condition(scenario: Path, result: Path, keys: str) -> None:
    # Backup the original file
    backup_file_path = backup_scenario_file(scenario)
    termcolor.cprint(f"Original scenario file backed up to: {backup_file_path}", "yellow")
    # Load data
    metrics_data = load_metrics_data(result)
    scenario_data = load_scenario_file(scenario)
    # Update scenario file conditions
    update_conditions(scenario_data, metrics_data, keys)
    # Save the updated scenario file
    save_scenario_file(scenario_data, scenario)
    termcolor.cprint(f"{scenario.as_posix()} updated with new conditions.", "green")
