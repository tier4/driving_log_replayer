import argparse
from pathlib import Path

import yaml


def convert(scenario_path: Path) -> None:
    scenario_file = scenario_path.open("r+")
    yaml_obj = yaml.safe_load(scenario_file)

    if yaml_obj["ScenarioFormatVersion"] != "2.2.0":
        print(f"{scenario_path} does not require conversion")  # noqa
        scenario_file.close()
        return

    yaml_obj["ScenarioFormatVersion"] = "3.0.0"
    vehicle_id = yaml_obj["VehicleId"]
    map_path = yaml_obj["LocalMapPath"]
    yaml_obj.pop("VehicleId")
    yaml_obj.pop("LocalMapPath")

    yaml_obj["Evaluation"]["Datasets"] = []
    yaml_obj["Evaluation"]["Datasets"].append(
        {"t4_dataset": {"LocalMapPath": map_path, "VehicleId": vehicle_id}},
    )
    use_case_version: str = yaml_obj["Evaluation"]["UseCaseFormatVersion"]
    major, minor, patch = use_case_version.split(".")
    major_int = int(major)
    yaml_obj["Evaluation"]["UseCaseFormatVersion"] = f"{major_int + 1}.0.0"

    # 既存の内容を消す
    scenario_file.seek(0)
    scenario_file.truncate()  # ファイルの内容を空にする

    # 更新済みの内容を書き込む
    yaml.safe_dump(yaml_obj, scenario_file, sort_keys=False)
    scenario_file.close()


def move_dataset(scenario_path: Path) -> None:
    scenario_root = scenario_path.parent
    bag_path = scenario_root.joinpath("input_bag")
    t4_dataset_path = scenario_root.joinpath("t4_dataset")
    if bag_path.exists():
        # bag 移動
        t4_dataset_path.mkdir()
        bag_path.rename(Path(t4_dataset_path, "input_bag"))
    # t4_datasetの移動


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "scenario_root_dir",
        help="root directory where the scenario files to be converted are located",
    )
    args = parser.parse_args()
    scenario_paths = Path(args.scenario_root_dir).resolve().glob("**/scenario.y*ml")  # yaml or yml
    for scenario_path in sorted(scenario_paths):
        # debug print(scenario_path)
        convert(scenario_path)
        move_dataset(scenario_path)


if __name__ == "__main__":
    main()
