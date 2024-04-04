import datetime
from pathlib import Path
import subprocess
import sys

from pydantic import ValidationError
import termcolor

from driving_log_replayer_cli.core.config import Config
from driving_log_replayer_cli.core.result import convert_all
from driving_log_replayer_cli.core.result import display_all
from driving_log_replayer_cli.core.scenario import Datasets
from driving_log_replayer_cli.core.scenario import get_dry_run_scenario_path
from driving_log_replayer_cli.core.scenario import load_scenario
from driving_log_replayer_cli.core.scenario import Scenario
from driving_log_replayer_cli.simulation.update import update_annotationless_scenario_condition

USE_T4_DATASET = ("perception", "obstacle_segmentation", "perception_2d", "traffic_light")


def run_with_log(cmd: list, log_path: Path) -> None:
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    f = log_path.open("w", encoding="utf-8")

    try:
        while True:
            line = proc.stdout.readline().decode("utf-8")
            sys.stdout.write(line)
            f.write(line)
            if not line and proc.poll() is not None:
                break
    finally:
        f.close()


def run(
    config: Config,
    launch_args: list[str],
    update_method: str,
) -> None:
    base_scenario_path = config.data_directory.joinpath("base_scenario.yaml")
    output_dir_by_time = create_output_dir_by_time(config.output_directory)
    for dataset_path in config.data_directory.glob("*"):
        # open scenario file and make ros2 launch command
        if not dataset_path.is_dir():
            continue
        output_case = output_dir_by_time.joinpath(dataset_path.name)
        scenario_file = get_scenario_file(dataset_path, base_scenario_path)
        if scenario_file is None:
            continue
        scenario = load_scenario(scenario_file)
        launch_cmd = None
        launch_arg_dict = args_to_dict(launch_args)
        if scenario.Evaluation["UseCaseName"] in USE_T4_DATASET:
            launch_cmd = cmd_use_t4_dataset(
                scenario_file,
                dataset_path,
                output_case,
                config.autoware_path,
                launch_arg_dict,
            )
        else:
            launch_cmd = cmd_use_bag_only(
                scenario_file,
                dataset_path,
                output_case,
                config.autoware_path,
                launch_arg_dict,
            )

        if launch_cmd is None:
            continue

        # create dir for output bag and result.jsonl
        output_case.mkdir(exist_ok=True)

        # save command as bash script
        run_script = output_case.joinpath("run.bash")
        with run_script.open("w") as f:
            f.write(launch_cmd)

        # run simulation
        cmd = ["/bin/bash", run_script.as_posix()]
        try:
            run_with_log(cmd, output_case.joinpath("console.log"))
        except KeyboardInterrupt:
            termcolor.cprint("Simulation execution canceled by Ctrl+C", "red")
            break
        if scenario.Evaluation["UseCaseName"] == "annotationless_perception":
            update_annotationless_scenario_condition(
                scenario_file,
                output_case.joinpath("result.jsonl"),
                update_method,
            )

    # convert result file and display result
    convert_all(output_dir_by_time)
    display_all(output_dir_by_time)


def dry_run(
    config: Config,
    use_case: str,
    launch_args: list[str],
) -> None:
    output_dir_by_time = create_output_dir_by_time(config.output_directory)
    output_case = output_dir_by_time.joinpath("dry_run")
    scenario_file = get_dry_run_scenario_path(use_case)
    scenario = load_scenario(scenario_file)
    launch_cmd = None
    launch_arg_dict = args_to_dict(launch_args)
    if scenario.Evaluation["UseCaseName"] in USE_T4_DATASET:
        launch_cmd = cmd_use_t4_dataset(
            scenario_file,
            output_case,
            config.autoware_path,
            launch_arg_dict,
        )
    else:
        launch_cmd = cmd_use_bag_only(
            scenario_file,
            output_case,
            config.autoware_path,
            launch_arg_dict,
        )

    if launch_cmd is None:
        return

    # create dir for output bag and result.jsonl
    output_case.mkdir(exist_ok=True)

    # save command as bash script
    run_script = output_case.joinpath("run.bash")
    with run_script.open("w") as f:
        f.write(launch_cmd)

    # run simulation
    cmd = ["/bin/bash", run_script.as_posix()]
    try:
        run_with_log(cmd, output_case.joinpath("console.log"))
    except KeyboardInterrupt:
        termcolor.cprint("Simulation execution canceled by Ctrl+C", "red")
    # convert result file and display result
    convert_all(output_dir_by_time)
    display_all(output_dir_by_time)


def create_output_dir_by_time(base_path: Path) -> Path:
    output_dir_by_time = base_path.joinpath(
        datetime.datetime.now().strftime("%Y-%m%d-%H%M%S")  # noqa
    )
    output_dir_by_time.mkdir()
    symlink_dst = base_path.joinpath("latest").as_posix()
    update_symlink = ["ln", "-snf", output_dir_by_time.as_posix(), symlink_dst]
    subprocess.run(update_symlink, check=False)
    return output_dir_by_time


def get_scenario_file(dataset_path: Path, base_scenario_path: Path) -> Path | None:
    scenario_file_path = dataset_path.joinpath("scenario.yaml")
    if scenario_file_path.exists():
        return scenario_file_path
    scenario_file_path = dataset_path.joinpath("scenario.yml")
    if scenario_file_path.exists():
        return scenario_file_path
    scenario_file_path = base_scenario_path
    if scenario_file_path.exists():
        return scenario_file_path
    termcolor.cprint(
        f"scenario file does not exist in {dataset_path.as_posix()} and no base_scenario",
        "red",
    )
    return None


def args_to_dict(launch_args: list[str]) -> dict[str, str]:
    launch_arg_dict = {}
    for l_arg in launch_args:
        try:
            key, value = l_arg.split(":=")
            launch_arg_dict[key] = value
        except ValueError:
            # invalid argument
            termcolor.cprint(
                f"{l_arg} is ignored because it is invalid",
                "red",
            )
    return launch_arg_dict


def launch_dict_to_str(launch_arg_dict: dict) -> str:
    rtn_str = ""
    for k, v in launch_arg_dict.items():
        if isinstance(v, str) and ("{" in v or "[" in v):
            rtn_str += f" '{k}:={v}'"
        else:
            rtn_str += f" {k}:={v}"
    return rtn_str


def clean_up_cmd() -> str:
    # echo return value of ros2 launch (0: ok, others: ng)
    # kill zombie ros2 process
    # kill rviz
    # sleep 1 sec
    # new line
    return """
echo \"exit status: $?\"
pgrep ros | awk \'{ print "kill -9 $(pgrep -P ", $1, ") > /dev/null 2>&1" }\' | sh
pgrep ros | awk \'{ print "kill -9 ", $1, " > /dev/null 2>&1" }\' | sh
pgrep rviz | awk \'{ print "kill -9 $(pgrep -P ", $1, ") > /dev/null 2>&1" }\' | sh
pgrep rviz | awk \'{ print "kill -9 ", $1, " > /dev/null 2>&1" }\' | sh
sleep 1
"""


def cmd_use_bag_only(
    scenario_path: Path,
    dataset_path: Path,
    output_path: Path,
    autoware_path: Path,
    launch_args_dict: dict[str, str],
) -> str | None:
    scenario: Scenario = load_scenario(scenario_path)
    launch_command = f"source {autoware_path.joinpath('install', 'setup.bash')}\n"
    launch_command += (
        f"ros2 launch driving_log_replayer {scenario.Evaluation['UseCaseName']}.launch.py"
    )
    launch_arg_dict_scenario = {
        "map_path": scenario.LocalMapPath,
        "vehicle_model": scenario.VehicleModel,
        "sensor_model": scenario.SensorModel,
        "vehicle_id": scenario.VehicleId,
        "scenario_path": scenario_path,
        "result_json_path": output_path.joinpath("result.json"),
        "input_bag": dataset_path.joinpath("input_bag"),
        "result_bag_path": output_path.joinpath("result_bag"),
    }
    launch_localization = scenario.Evaluation.get("LaunchLocalization")
    if launch_localization is not None:
        launch_arg_dict_scenario["localization"] = launch_localization
    direct_initialpose = scenario.Evaluation.get("DirectInitialPose")
    if direct_initialpose is not None:
        launch_arg_dict_scenario["initial_pose"] = direct_initialpose
    launch_arg_dict_scenario.update(launch_args_dict)
    launch_command += launch_dict_to_str(launch_arg_dict_scenario) + "\n"
    return launch_command + clean_up_cmd()


def cmd_use_t4_dataset(
    scenario_path: Path,
    dataset_path: Path,
    output_path: Path,
    autoware_path: Path,
    launch_args_dict: dict[str, str],
) -> str | None:
    scenario: Scenario = load_scenario(scenario_path)
    launch_command_for_all_dataset = f"source {autoware_path.joinpath('install', 'setup.bash')}\n"
    t4_dataset_base_path = dataset_path.joinpath("t4_dataset")
    try:
        t4_datasets: Datasets = Datasets(Datasets=scenario.Evaluation["Datasets"])
    except ValidationError:
        return None
    else:
        is_database_evaluation = bool(len(t4_datasets.Datasets) > 1)
        for t4_dataset in t4_datasets.Datasets:
            # get dataset_id
            key = next(iter(t4_dataset))
            # create sub directory for the dataset
            output_dir_per_dataset = output_path.joinpath(key)
            output_dir_per_dataset.mkdir(parents=True)
            vehicle_id = t4_dataset[key].VehicleId
            map_path = t4_dataset[key].LocalMapPath
            t4_dataset_path = t4_dataset_base_path.joinpath(key)
            if not t4_dataset_path.exists():
                termcolor.cprint(
                    f"t4_dataset: {key} does not exist",
                    "red",
                )
                continue

            launch_command = (
                f"ros2 launch driving_log_replayer {scenario.Evaluation['UseCaseName']}.launch.py"
            )
            launch_arg_dict_dataset = {
                "map_path": map_path,
                "vehicle_model": scenario.VehicleModel,
                "sensor_model": scenario.SensorModel,
                "vehicle_id": vehicle_id,
                "scenario_path": scenario_path,
                "result_json_path": output_dir_per_dataset.joinpath("result.json"),
                "input_bag": t4_dataset_path.joinpath("input_bag"),
                "result_bag_path": output_dir_per_dataset.joinpath("result_bag"),
                "t4_dataset_path": t4_dataset_path,
                "result_archive_path": output_dir_per_dataset.joinpath("result_archive"),
            }
            if t4_dataset[key].LaunchSensing is not None:
                launch_arg_dict_dataset["sensing"] = t4_dataset[key].LaunchSensing
            launch_arg_dict_dataset.update(launch_args_dict)
            launch_command += launch_dict_to_str(launch_arg_dict_dataset) + "\n"
            launch_command += clean_up_cmd()
            launch_command_for_all_dataset += launch_command
        if is_database_evaluation:
            database_result_script_path = autoware_path.joinpath(
                "install",
                "driving_log_replayer",
                "lib",
                "driving_log_replayer",
                "perception_database_result.py",
            )
            database_result_command = (
                f"python3 {database_result_script_path} -s {scenario_path} -r {output_path}\n"
            )
            launch_command_for_all_dataset += database_result_command
        return launch_command_for_all_dataset
