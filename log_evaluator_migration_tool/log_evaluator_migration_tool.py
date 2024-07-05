import argparse
from os.path import expandvars
from pathlib import Path
from shutil import copytree

import yaml

KEY_NOT_EXIT = "key_not_exist"


def convert_scenario(scenario_path: Path) -> None:
    scenario_file = scenario_path.open("r+")
    yaml_obj = yaml.safe_load(scenario_file)

    if yaml_obj["ScenarioFormatVersion"] != "2.2.0":
        print(f"{scenario_path} does not require conversion")  # noqa
        scenario_file.close()
        return

    yaml_obj["ScenarioFormatVersion"] = "3.0.0"
    vehicle_id = yaml_obj["VehicleId"]

    # keyが存在しないのか、keyが存在した上でnull(None)なのか区別するためにgetのデフォルト値を固定値にする

    local_map_path = yaml_obj.get("LocalMapPath", KEY_NOT_EXIT)
    launch_localization = yaml_obj["Evaluation"].get("LaunchLocalization", KEY_NOT_EXIT)
    initial_pose = yaml_obj["Evaluation"].get("InitialPose", KEY_NOT_EXIT)
    direct_initial_pose = yaml_obj["Evaluation"].get("DirectInitialPose", KEY_NOT_EXIT)
    yaml_obj.pop("VehicleId")
    yaml_obj["Evaluation"]["Datasets"] = []
    migration_dict = {"VehicleId": vehicle_id}
    if local_map_path != KEY_NOT_EXIT:
        yaml_obj.pop("LocalMapPath")
        migration_dict |= {"LocalMapPath": local_map_path}
    if launch_localization != KEY_NOT_EXIT:
        yaml_obj["Evaluation"].pop("LaunchLocalization")
        migration_dict |= {"LaunchLocalization": launch_localization}
    if initial_pose != KEY_NOT_EXIT:
        yaml_obj["Evaluation"].pop("InitialPose")
        migration_dict |= {"InitialPose": initial_pose}
    if direct_initial_pose != KEY_NOT_EXIT:
        yaml_obj["Evaluation"].pop("DirectInitialPose")
        migration_dict |= {"DirectInitialPose": direct_initial_pose}
    yaml_obj["Evaluation"]["Datasets"].append(
        {"bag_only_dataset": migration_dict},
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


def move_dataset_and_map(scenario_path: Path) -> None:
    scenario_file = scenario_path.open("r+")
    yaml_obj = yaml.safe_load(scenario_file)  # scenario file is 3.0.0 format

    scenario_root = scenario_path.parent
    bag_path = scenario_root.joinpath("input_bag")
    t4_dataset_path = scenario_root.joinpath("t4_dataset")
    bag_only_dataset_path = t4_dataset_path.joinpath("bag_only_dataset")

    # bag 移動
    if bag_path.exists():
        t4_dataset_path.mkdir()
        bag_only_dataset_path.mkdir()
        bag_path.rename(
            bag_only_dataset_path.joinpath("input_bag"),
        )  # 一旦t4_dataset配下に移動させて、3.0.0系統と同じフォルダ構成に変更しておく
    # t4_datasetの移動
    if t4_dataset_path.exists():
        for t4_dataset_dict in yaml_obj["Evaluation"]["Datasets"]:
            t4_dataset_dict: dict
            for dataset_name, v in t4_dataset_dict.items():
                new_dataset_dir = scenario_root.joinpath(dataset_name)
                new_dataset_dir.mkdir()
                t4_dataset_path.joinpath(dataset_name).rename(new_dataset_dir)
                # copy map
                local_map_path_str: str | None = v.get("LocalMapPath")
                if local_map_path_str is not None:
                    v.pop("LocalMapPath")
                    local_map_path = Path(expandvars(local_map_path_str))
                    if local_map_path.exists():
                        copytree(
                            local_map_path.as_posix(),
                            new_dataset_dir.joinpath("map").as_posix(),
                        )
                    else:
                        print(f"cannot copy {local_map_path}")  # noqa
        # remove base dir
        t4_dataset_path.rmdir()

    # 既存の内容を消す
    scenario_file.seek(0)
    scenario_file.truncate()

    # 更新済みの内容を書き込む
    yaml.safe_dump(yaml_obj, scenario_file, sort_keys=False)
    scenario_file.close()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "data_directory",
        help="data_directory of driving_log_replayer",
    )
    args = parser.parse_args()
    scenario_paths = Path(args.data_directory).resolve().glob("**/scenario*.y*ml")  # yaml or yml
    for scenario_path in sorted(scenario_paths):
        print(f"convert {scenario_path}")  # noqa
        convert_scenario(scenario_path)
        move_dataset_and_map(scenario_path)


if __name__ == "__main__":
    main()
