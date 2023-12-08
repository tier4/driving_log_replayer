import datetime
from pathlib import Path
import subprocess
import sys

import termcolor
import yaml

from driving_log_replayer_cli.core.config import Config
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
        launch_cmd: str | None = create_launch_cmd(dataset_path)
        if launch_cmd is None:
            continue

        # create dir for output bag and result.jsonl
        output_case = output_dir_by_time.joinpath(dataset_path.name)
        output_case.mkdir()

        # create bash command
        bash_cmd = create_bash_cmd(config, launch_args, launch_cmd)

        # save command as bash script
        run_script = output_case.joinpath("run.bash")
        with run_script.open("w") as f:
            f.write(bash_cmd)

        # run simulation
        cmd = ["/bin/bash", run_script.as_posix()]
        # tryない方がいいかも
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


def create_launch_cmd(dataset_path: Path) -> str | None:
    scenario_file_path = dataset_path.joinpath("scenario.yaml")
    if not scenario_file_path.exists():
        scenario_file_path = dataset_path.joinpath("scenario.yml")
        if not scenario_file_path.exists():
            termcolor.cprint(
                scenario_file_path.as_posix() + " does not exist.",
                "red",
            )
            return None
    with scenario_file_path.open("r") as scenario_file:
        scenario_dict = yaml.safe_load(scenario_file)
    if scenario_dict["Evaluation"]["UseCaseName"] in USE_T4_DATASET:
        return cmd_use_t4_dataset(scenario_file_path)
    return cmd_use_bag_only(scenario_file_path)


def create_bash_cmd(launch_cmd: str, launch_args: str, config: Config) -> str:
    bash_cmd = f"source {config.autoware_path.joinpath('install', 'setup.bash').as_posix()}\n"
    bash_cmd += launch_cmd + extract_arg(launch_args) + "\n"
    bash_cmd += clean_up_cmd()
    return bash_cmd


def clean_up_cmd() -> str:
    # echo return value of ros2 launch (0: ok, others: ng)
    # kill zombie ros2 process
    # kill rviz
    # sleep 1 sec
    return """echo $?
pgrep ros | awk \'{ print "kill -9 $(pgrep -P ", $1, ") > /dev/null 2>&1" }\' | sh
pgrep ros | awk \'{ print "kill -9 ", $1, " > /dev/null 2>&1" }\' | sh
pgrep rviz | awk \'{ print "kill -9 $(pgrep -P ", $1, ") > /dev/null 2>&1" }\' | sh
pgrep rviz | awk \'{ print "kill -9 ", $1, " > /dev/null 2>&1" }\' | sh
sleep 1"""


def cmd_use_t4_dataset(scenario_file_path: Path) -> str:
    return ""


def cmd_use_bag_only(scenario_file_path: Path) -> str:
    return ""


def extract_arg(launch_args: str) -> str:
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
