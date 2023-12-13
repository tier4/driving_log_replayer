import datetime
from pathlib import Path
import subprocess
import sys

from pydantic import ValidationError
import termcolor

from driving_log_replayer_cli.core.config import Config
from driving_log_replayer_cli.core.scenario import Datasets
from driving_log_replayer_cli.core.scenario import load_scenario
from driving_log_replayer_cli.core.scenario import Scenario
from driving_log_replayer_cli.simulation.result import convert_all
from driving_log_replayer_cli.simulation.result import display_all

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
    launch_args: str,
) -> None:
    output_dir_by_time = create_output_dir_by_time(config.output_directory)
    for dataset_path in config.data_directory.glob("*"):
        # open scenario file and make ros2 launch command
        if not dataset_path.is_dir():
            continue
        output_case = output_dir_by_time.joinpath(dataset_path.name)
        scenario_file = get_scenario_file(dataset_path)
        if scenario_file is None:
            continue
        launch_cmd = None
        if scenario_file.parent.joinpath("t4_dataset").exists():
            launch_cmd = cmd_use_t4_dataset(
                scenario_file,
                output_case,
                config.autoware_path,
                launch_args,
            )
        if scenario_file.parent.joinpath("input_bag").exists():
            launch_cmd = cmd_use_bag_only(
                scenario_file,
                output_case,
                config.autoware_path,
                launch_args,
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


def get_scenario_file(dataset_path: Path) -> Path | None:
    scenario_file_path = dataset_path.joinpath("scenario.yaml")
    if not scenario_file_path.exists():
        scenario_file_path = dataset_path.joinpath("scenario.yml")
        if not scenario_file_path.exists():
            termcolor.cprint(
                scenario_file_path.as_posix() + " does not exist.",
                "red",
            )
            return None
    return scenario_file_path


def extract_arg(launch_args: str) -> str:
    if launch_args == "":
        return ""
    extract_launch_arg = ""
    keys_values_list = launch_args.split(",")
    for kv_str in keys_values_list:
        if len(kv_str.split(":=")) != 2:  # noqa
            # invalid argument
            termcolor.cprint(
                f"{kv_str} is ignored because it is invalid",
                "red",
            )
        else:
            extract_launch_arg += f" {kv_str}"
    return extract_launch_arg


def clean_up_cmd() -> str:
    # echo return value of ros2 launch (0: ok, others: ng)
    # kill zombie ros2 process
    # kill rviz
    # sleep 1 sec
    return """
echo $?
pgrep ros | awk \'{ print "kill -9 $(pgrep -P ", $1, ") > /dev/null 2>&1" }\' | sh
pgrep ros | awk \'{ print "kill -9 ", $1, " > /dev/null 2>&1" }\' | sh
pgrep rviz | awk \'{ print "kill -9 $(pgrep -P ", $1, ") > /dev/null 2>&1" }\' | sh
pgrep rviz | awk \'{ print "kill -9 ", $1, " > /dev/null 2>&1" }\' | sh
sleep 1"""


def cmd_use_bag_only(
    scenario_path: Path,
    output_path: Path,
    autoware_path: Path,
    launch_args: str,
) -> str | None:
    scenario: Scenario = load_scenario(scenario_path)
    launch_command = f"source {autoware_path.joinpath('install', 'setup.bash').as_posix()}\n"
    launch_command += f"ros2 launch driving_log_replayer {scenario.Evaluation['UseCaseName']}.launch.py map_path:={scenario.LocalMapPath} vehicle_model:={scenario.VehicleModel} sensor_model:={scenario.SensorModel} vehicle_id:={scenario.VehicleId}"
    launch_command += f" scenario_path:={scenario_path.as_posix()} result_json_path:={output_path.joinpath('result.json').as_posix()} input_bag:={scenario_path.parent.joinpath('input_bag').as_posix()} result_bag_path:={output_path.joinpath('result_bag').as_posix()}"
    launch_command += f" localization:={scenario.Evaluation.get('LaunchLocalization', True)}"
    launch_command += extract_arg(launch_args)
    return launch_command + clean_up_cmd()


def cmd_use_t4_dataset(
    scenario_path: Path,
    output_path: Path,
    autoware_path: Path,
    launch_args: str,
) -> str | None:
    dataset_path = scenario_path.parent
    scenario: Scenario = load_scenario(scenario_path)
    launch_command_for_all_dataset = (
        f"source {autoware_path.joinpath('install', 'setup.bash').as_posix()}\n"
    )
    t4_dataset_base_path = dataset_path.joinpath("t4_dataset")
    try:
        t4_datasets: Datasets = Datasets(Datasets=scenario.Evaluation["Datasets"])
    except ValidationError:
        return None
    else:
        is_database_evaluation = bool(len(t4_datasets) > 1)
        for t4_dataset in t4_datasets.Datasets:
            # get dataset_id
            key = next(iter(t4_dataset))
            # create sub directory for the dataset
            output_dir_per_dataset = output_path.joinpath(dataset_path.name, key)
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

            launch_command = f"ros2 launch driving_log_replayer {scenario.Evaluation['UseCaseName']}.launch.py map_path:={map_path} vehicle_model:={scenario.VehicleModel} sensor_model:={scenario.SensorModel} vehicle_id:={vehicle_id}"
            launch_command += f" scenario_path:={scenario_path.as_posix()} result_json_path:={output_dir_per_dataset.joinpath('result.json').as_posix()} input_bag:={t4_dataset_path.parent.joinpath('input_bag').as_posix()} result_bag_path:={output_dir_per_dataset.joinpath('result_bag').as_posix()}"
            launch_command += f" t4_dataset_path:={t4_dataset_path} result_archive_path:={output_dir_per_dataset.joinpath('result_archive').as_posix()} sensing:={t4_dataset[key].get('LaunchSensing', True)}"
            launch_command += extract_arg(launch_args)
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
            database_result_command = f"python3 {database_result_script_path.as_posix()} -s {scenario_path} -r {output_path.joinpath(dataset_path.name)}\n"
            launch_command_for_all_dataset += database_result_command
        return launch_command_for_all_dataset
