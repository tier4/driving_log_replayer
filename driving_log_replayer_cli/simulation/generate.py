from dataclasses import dataclass
from os.path import expandvars
from pathlib import Path
import subprocess
import sys

import termcolor
import yaml


@dataclass(frozen=True)
class AutowareEssentialParameter:
    map_path: Path = Path("/dlr_not_exist_path")
    vehicle_model: str = ""
    vehicle_id: str = "default"
    sensor_model: str = ""


class TestScriptGenerator:
    USE_T4_DATASET = ("perception", "obstacle_segmentation", "perception_2d", "traffic_light")

    def __init__(
        self,
        data_directory: str,
        output_directory: str,
        autoware_path: str,
        rate: float,
        delay: float,
        perception_mode: str | None,
        override_recode_topics: bool,  # noqa
        override_topics_regex: str,
    ) -> None:
        self.__data_directory = Path(data_directory)
        self.__output_directory = Path(output_directory)
        self.__autoware_path = Path(autoware_path)
        self.__script_path = self.__output_directory.joinpath("run.bash")
        self.__rate = rate
        self.__delay = delay
        self.__perception_mode = perception_mode
        self.__override_topics_regex = override_topics_regex
        self.__override_recode_topics = override_recode_topics
        #  os.path.join(config.output_directory, datetime.datetime.now().strftime("%Y-%m%d-%H%M%S"))が渡ってくるので被ることはない
        self.__output_directory.mkdir(parents=True)

        symlink_dst = self.__output_directory.parent.joinpath("latest").as_posix()
        update_symlink = ["ln", "-snf", self.__output_directory.as_posix(), symlink_dst]
        subprocess.run(update_symlink, check=False)

    @property
    def script_path(self) -> Path:
        return self.__script_path

    @classmethod
    def clean_up_cmd(cls) -> str:
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
sleep 1
"""

    def run(self) -> bool:
        if not self.__output_directory.exists():
            return False
        generated_cmd = (
            f"source {self.__autoware_path.joinpath('install', 'setup.bash').as_posix()}\n"
        )
        for dataset_path in self.__data_directory.glob("*"):
            if not dataset_path.is_dir():
                continue
            launch_command = self._create_launch_cmd(dataset_path)
            if launch_command is None:
                continue
            generated_cmd += launch_command
            generated_cmd += TestScriptGenerator.clean_up_cmd()
        # create shell script file
        with self.script_path.open("w") as f:
            f.write(generated_cmd)
        return True

    def _create_launch_cmd(self, dataset_path: Path) -> str | None:
        scenario_output_dir = self.__output_directory.joinpath(dataset_path.name)
        scenario_output_dir.mkdir()
        # check scenario
        scenario_path = dataset_path.joinpath("scenario.yaml")
        if not scenario_path.exists():
            scenario_path = scenario_path.joinpath("scenario.yml")
            if not scenario_path.exists():
                termcolor.cprint(
                    scenario_path.as_posix() + " does not exist.",
                    "red",
                )
                return None

        with scenario_path.open("r") as scenario_file:
            scenario_yaml_obj = yaml.safe_load(scenario_file)
        if scenario_yaml_obj["Evaluation"]["UseCaseName"] in TestScriptGenerator.USE_T4_DATASET:
            return self._cmd_use_t4_dataset(
                scenario_path,
                scenario_yaml_obj,
            )
        return self._cmd_use_bag_only(
            scenario_path,
            scenario_yaml_obj,
        )

    def _create_common_arg(
        self,
        optional_arg: str,
        scenario_path: Path,
        output_path: Path,
        input_bag: Path,
        essential_param: AutowareEssentialParameter,
    ) -> str:
        return f"""{optional_arg}\
scenario_path:={scenario_path.as_posix()} \
result_json_path:={output_path.joinpath("result.json").as_posix()} \
play_rate:={self.__rate} \
play_delay:={self.__delay} \
input_bag:={input_bag.as_posix()} \
result_bag_path:={output_path.joinpath("result_bag").as_posix()} \
map_path:={essential_param.map_path} \
vehicle_model:={essential_param.vehicle_model} \
sensor_model:={essential_param.sensor_model} \
vehicle_id:={essential_param.vehicle_id} \
override_record_topics:={self.__override_recode_topics} \
override_topics_regex:={self.__override_topics_regex} \
rviz:=true\
"""

    def _cmd_use_bag_only(
        self,
        scenario_path: Path,
        scenario_yaml_obj: dict,
    ) -> str | None:
        dataset_path = scenario_path.parent
        # check resource
        map_path = Path(expandvars(scenario_yaml_obj.get("LocalMapPath", "/dlr_not_exist_path")))
        if not map_path.exists():
            termcolor.cprint(
                f"map: {map_path.as_posix()} used in scenario: {dataset_path.as_posix()} does not exist ",
                "red",
            )
            return None
        input_bag = dataset_path.joinpath("input_bag")
        if not input_bag.exists():
            termcolor.cprint(f"input_bag file {input_bag.as_posix()} does not exist.", "red")
            return None
        output_path = self.__output_directory.joinpath(dataset_path.name)
        use_case_name = scenario_yaml_obj["Evaluation"]["UseCaseName"]
        launch_base_command = f"ros2 launch driving_log_replayer {use_case_name}.launch.py "
        optional_arg = ""
        launch_localization = scenario_yaml_obj["Evaluation"].get("LaunchLocalization")
        if launch_localization is not None:
            optional_arg = f"localization:={launch_localization} "

        launch_command = launch_base_command + self._create_common_arg(
            optional_arg,
            scenario_path,
            output_path,
            input_bag,
            AutowareEssentialParameter(
                map_path,
                scenario_yaml_obj["VehicleModel"],
                scenario_yaml_obj["VehicleId"],
                scenario_yaml_obj["SensorModel"],
            ),
        )
        print("launch command generated! => " + launch_command)  # noqa
        return launch_command

    def _cmd_use_t4_dataset(
        self,
        scenario_path: Path,
        scenario_yaml_obj: dict,
    ) -> str | None:
        dataset_path = scenario_path.parent
        launch_command_for_all_dataset = ""
        t4_dataset_base_path = dataset_path.joinpath("t4_dataset")
        t4_datasets = scenario_yaml_obj["Evaluation"]["Datasets"]
        is_database_evaluation = bool(len(t4_datasets) > 1)
        for dataset in t4_datasets:
            # get dataset_id
            key = next(iter(dataset))
            # create sub directory for the dataset
            output_dir_per_dataset = self.__output_directory.joinpath(dataset_path.name, key)
            output_dir_per_dataset.mkdir()
            vehicle_id = dataset[key].get("VehicleId", "")
            map_path = Path(expandvars(dataset[key].get("LocalMapPath", "/dlr_not_exist_path")))
            t4_dataset_path = t4_dataset_base_path.joinpath(key)
            if vehicle_id == "":
                termcolor.cprint(
                    f"vehicle_id is not defined in dataset: {key}",
                    "red",
                )
                continue

            if not map_path.exists():
                termcolor.cprint(
                    "map: " + map_path.as_posix() + " used in dataset: " + key + " does not exist ",
                    "red",
                )
                continue

            if not t4_dataset_path.exists():
                termcolor.cprint(
                    f"t4_dataset: {key} does not exist",
                    "red",
                )
                continue

            use_case_name = scenario_yaml_obj["Evaluation"]["UseCaseName"]
            launch_base_command = f"ros2 launch driving_log_replayer {use_case_name}.launch.py "
            optional_arg = (
                f"perception_mode:={self.__perception_mode} "
                if isinstance(self.__perception_mode, str)
                else ""
            )
            launch_args = self._create_common_arg(
                optional_arg,
                scenario_path,
                output_dir_per_dataset,
                t4_dataset_path.joinpath("input_bag"),
                AutowareEssentialParameter(
                    map_path,
                    scenario_yaml_obj["VehicleModel"],
                    vehicle_id,
                    scenario_yaml_obj["SensorModel"],
                ),
            )
            # t4_dataset
            launch_args += f" t4_dataset_path:={t4_dataset_path}"
            launch_args += (
                f" result_archive_path:={output_dir_per_dataset.joinpath('result_archive')}"
            )
            launch_args += f" sensing:={dataset[key].get('LaunchSensing', True)}"
            launch_command = launch_base_command + launch_args + "\n"
            launch_command_for_all_dataset += launch_command
        if is_database_evaluation:
            database_result_script_path = self.__autoware_path.joinpath(
                "install",
                "driving_log_replayer",
                "lib",
                "driving_log_replayer",
                "perception_database_result.py",
            )
            database_result_command = f"python3 {database_result_script_path.as_posix()} -s {scenario_path} -r {self.__output_directory.joinpath(dataset_path.name)}\n"
            launch_command_for_all_dataset += database_result_command
        print("launch command generated! => " + launch_command_for_all_dataset)  # noqa
        return launch_command_for_all_dataset
