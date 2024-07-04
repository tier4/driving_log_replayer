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
    local_map_path = yaml_obj.get("LocalMapPath")
    launch_localization = yaml_obj.get("LaunchLocalization")
    initial_pose = yaml_obj.get("InitialPose")
    direct_initial_pose = yaml_obj.get("DirectInitialPose")
    yaml_obj.pop("LocalMapPath")
    if local_map_path is not None:
        yaml_obj.pop("VehicleId")
    if launch_localization is not None:
        yaml_obj.pop("LaunchLocalization")
    if initial_pose is not None:
        yaml_obj.pop("InitialPose")
    if direct_initial_pose is not None:
        yaml_obj.pop("DirectInitialPose")

    yaml_obj["Evaluation"]["Datasets"] = []
    migration_dict = {"VehicleId": vehicle_id}
    if local_map_path is not None:
        migration_dict |= {"LocalMapPath": local_map_path}
    if launch_localization is not None:
        migration_dict |= {"LaunchLocalization": launch_localization}
    if initial_pose is not None:
        migration_dict |= {"InitialPose": initial_pose}
    if direct_initial_pose is not None:
        migration_dict |= {"DirectInitialPose": direct_initial_pose}
    yaml_obj["Evaluation"]["Datasets"].append(
        {"t4_dataset": migration_dict},
    )
    use_case_version: str = yaml_obj["Evaluation"]["UseCaseFormatVersion"]
    major, minor, patch = use_case_version.split(".")
    major_int = int(major)
    yaml_obj["Evaluation"]["UseCaseFormatVersion"] = f"{major_int + 1}.0.0"

    # 既存の内容を消す
    scenario_file.seek(0)
    scenario_file.truncate()

    # 更新済みの内容を書き込む
    yaml.safe_dump(yaml_obj, scenario_file, sort_keys=False)
    scenario_file.close()


def move_dataset(scenario_path: Path) -> None:
    # ファイル移動
    scenario_root = scenario_path.parent
    bag_path = scenario_root.joinpath("input_bag")
    t4_dataset_path = scenario_root.joinpath("t4_dataset")
    # bag 移動
    if bag_path.exists():
        t4_dataset_path.mkdir()
        bag_path.rename(Path(t4_dataset_path, "input_bag"))
    # t4_datasetの移動
    elif t4_dataset_path.exists():
        scenario_file = scenario_path.open()
        yaml_obj = yaml.safe_load(scenario_file)
        for t4_dataset_dict in yaml_obj["Evaluation"]["Datasets"]:
            t4_dataset_dict: dict
            for dataset_name in t4_dataset_dict:
                new_dataset_dir = scenario_root.joinpath(dataset_name)
                new_dataset_dir.mkdir()
                t4_dataset_path.joinpath(dataset_name).rename(new_dataset_dir)
        # remove base dir
        t4_dataset_path.rmdir()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "scenario_root_dir",
        help="root directory where the scenario files to be converted are located",
    )
    args = parser.parse_args()
    scenario_paths = Path(args.scenario_root_dir).resolve().glob("**/scenario*.y*ml")  # yaml or yml
    for scenario_path in sorted(scenario_paths):
        # debug print(scenario_path)
        convert(scenario_path)
        move_dataset(scenario_path)


if __name__ == "__main__":
    main()
