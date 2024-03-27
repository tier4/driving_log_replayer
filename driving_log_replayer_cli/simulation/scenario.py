import argparse
from datetime import datetime
import json
import shutil

import yaml


def load_jsonl_data(jsonl_file_path):
    data = []
    with open(jsonl_file_path, "r") as file:
        for line in file:
            data.append(json.loads(line))
    return data[-1]["Frame"]["FinalMetrics"]


def load_scenario_file(scenario_file_path):
    with open(scenario_file_path, "r") as file:
        data = yaml.safe_load(file)
    return data


def update_conditions(scenario_data, metrics_data, keys_to_update):
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


def save_scenario_file(scenario_data, scenario_file_path):
    with open(scenario_file_path, "w") as file:
        yaml.safe_dump(scenario_data, file, sort_keys=False)


def backup_scenario_file(scenario_file_path):
    backup_file_path = f"{scenario_file_path}.{datetime.now().strftime('%Y%m%d%H%M%S')}.bak"
    shutil.copy(scenario_file_path, backup_file_path)
    return backup_file_path


def main():
    parser = argparse.ArgumentParser(
        description="Updates scenario file conditions with data from JSONL results.",
    )
    parser.add_argument(
        "-s",
        "--scenario",
        required=True,
        help="Path to the scenario YAML file",
    )
    parser.add_argument(
        "-r",
        "--result",
        required=True,
        help="Path to the result JSONL file",
    )
    parser.add_argument(
        "-k",
        "--keys",
        default="min,max,mean",
        help="Metrics to update (specified as a comma-separated list, e.g., 'min,max,mean')",
    )

    args = parser.parse_args()

    # Backup the original file
    backup_file_path = backup_scenario_file(args.scenario)
    print(f"Original scenario file backed up to: {backup_file_path}")

    # Load data
    metrics_data = load_jsonl_data(args.result)
    scenario_data = load_scenario_file(args.scenario)

    # Update scenario file conditions
    update_conditions(scenario_data, metrics_data, args.keys)

    # Save the updated scenario file
    save_scenario_file(scenario_data, args.scenario)
    print("Scenario file updated with new conditions.")


if __name__ == "__main__":
    main()
