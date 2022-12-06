import glob
import os
from pathlib import Path
import subprocess
from typing import Optional

from natsort import natsorted
import termcolor
import yaml


class TestScriptGenerator:
    def __init__(
        self,
        data_directory: str,
        output_directory: str,
        autoware_path: str,
        rate: float,
        delay: float,
    ):
        self.__data_directory = data_directory
        self.__output_directory = output_directory
        self.__autoware_path = autoware_path
        self.__script_path = os.path.join(self.__output_directory, "run.bash")
        self.__rate = rate
        self.__delay = delay
        #  os.path.join(config.output_directory, datetime.datetime.now().strftime("%Y-%m%d-%H%M%S"))が渡ってくるので被ることはない
        os.makedirs(self.__output_directory)

        symlink_dst = Path(self.__output_directory).parent.joinpath("latest").as_posix()
        update_latest_dir = f"ln -snf {self.__output_directory} {symlink_dst}"
        subprocess.run(update_latest_dir, shell=True)

    @property
    def script_path(self):
        return self.__script_path

    def run(self):
        is_executable = self.__create_script()
        return is_executable

    def __create_script(self) -> bool:
        if Path(self.__output_directory).exists():
            generated_cmd = ""
            if self.__autoware_path != "":
                setup_bash = Path(self.__autoware_path).joinpath("install", "setup.bash").as_posix()
                generated_cmd += f"source {setup_bash}\n"
            regex = os.path.join(self.__data_directory, "*")
            dataset_paths = glob.glob(regex)
            for dataset_path in natsorted(dataset_paths):
                if Path(dataset_path).is_dir():
                    command = self.__parse_scenario(dataset_path)
                    if command is not None:
                        generated_cmd += command + "\n"
                        # kill zombie ros2 process
                        generated_cmd += "ps aux | grep -i ros | grep -v Microsoft | grep -v ros2_daemon | grep -v grep | grep -v /bin/sh | grep -v /bin/bash |awk '{ print \"kill -9\", $2 }' | sh\n"
                        # kill rviz
                        generated_cmd += "ps aux | grep rviz | grep -v grep | awk '{ print \"kill -9\", $2 }' | sh\n"
                        # sleep 1 sec
                        generated_cmd += "sleep 1\n"
            with open(self.script_path, "w") as f:
                f.write(generated_cmd)
            return True
        else:
            return False

    def __parse_scenario(self, scenario_directory: str) -> Optional[str]:
        scenario_root = Path(scenario_directory)
        scenario_output_dir = os.path.join(self.__output_directory, scenario_root.name)
        os.makedirs(scenario_output_dir)
        scenario_path = scenario_root.joinpath("scenario.yaml")
        if scenario_path.exists():
            with open(scenario_path) as scenario_file:
                scenario_yaml_obj = yaml.safe_load(scenario_file)
                if scenario_yaml_obj["Evaluation"]["UseCaseName"] in [
                    "perception",
                    "obstacle_segmentation",
                ]:
                    return self.__create_launch_command_with_t4_dataset(
                        scenario_root.as_posix(), scenario_output_dir, scenario_yaml_obj
                    )
                else:
                    return self.__create_launch_command(
                        scenario_root.as_posix(), scenario_output_dir, scenario_yaml_obj
                    )
        else:
            termcolor.cprint(
                scenario_path.as_posix() + " does not exist.",
                "red",
            )
            return None

    def __create_launch_command(
        self, scenario_root: str, scenario_output_dir: str, scenario_yaml_obj
    ) -> Optional[str]:
        map_path = ""
        if "LocalMapPath" in scenario_yaml_obj:
            map_path = os.path.expandvars(scenario_yaml_obj["LocalMapPath"])
            if not os.path.exists(map_path):
                termcolor.cprint(
                    "map: " + map_path + " used in scenario: " + scenario_root + " is not exist ",
                    "red",
                )
                return None
        else:
            termcolor.cprint("LocalMapPath is not described in your Scenario.", "red")
            return None
        need_annotation = scenario_yaml_obj["Evaluation"]["UseCaseName"] == "obstacle_detection"
        input_bag = Path(scenario_root).joinpath("input_bag")
        if not input_bag.exists():
            termcolor.cprint("input_bag file" + input_bag.as_posix() + " does not exist.", "red")
            return None
        annotation_bag = Path(scenario_root).joinpath("annotation_bag")
        if need_annotation and not annotation_bag.exists():
            termcolor.cprint(
                "annotation_bag file" + annotation_bag.as_posix() + " does not exist.", "red"
            )
            return None

        use_case_name = scenario_yaml_obj["Evaluation"]["UseCaseName"]
        launch_base_command = f"ros2 launch driving_log_replayer {use_case_name}.launch.py"
        # evaluation component args
        launch_args = " scenario_path:=" + os.path.join(scenario_root, "scenario.yaml")
        launch_args += " result_json_path:=" + os.path.join(scenario_output_dir, "result.json")

        # ros2 bag play args
        launch_args += " play_rate:=" + str(self.__rate)
        launch_args += " play_delay:=" + str(self.__delay)
        launch_args += " input_bag:=" + input_bag.as_posix()

        # ros2 bag record args
        result_bag_path = os.path.join(scenario_output_dir, "result_bag")
        launch_args += " result_bag_path:=" + result_bag_path

        if need_annotation:
            launch_args += " annotation_path:=" + os.path.join(scenario_root, "Annotations")
            # annotation bag play args
            launch_args += " annotation_bag:=" + annotation_bag.as_posix()
            if "RecordRate" in scenario_yaml_obj["Annotation"]:
                launch_args += " annotation_record_rate:=" + str(
                    scenario_yaml_obj["Annotation"]["RecordRate"]
                )

        # logging_simulator.launch args
        launch_args += " map_path:=" + map_path
        launch_args += " vehicle_model:=" + scenario_yaml_obj["VehicleModel"]
        launch_args += " sensor_model:=" + scenario_yaml_obj["SensorModel"]
        launch_args += " vehicle_id:=" + scenario_yaml_obj["VehicleId"]
        launch_args += " rviz:=true"
        # diag launch localization
        if scenario_yaml_obj["Evaluation"]["UseCaseName"] == "performance_diag":
            launch_args += " localization:=" + str(
                scenario_yaml_obj["Evaluation"]["LaunchLocalization"]
            )
        if scenario_yaml_obj["Evaluation"]["UseCaseName"] == "obstacle_detection":
            launch_planning = scenario_yaml_obj["Evaluation"].get("LaunchPlanning", False)
            launch_args += " planning:=" + str(launch_planning)
            target_pointcloud = scenario_yaml_obj["Evaluation"]["Conditions"].get(
                "TargetPointCloud", "/perception/obstacle_segmentation/pointcloud"
            )
            launch_args += " target_pointcloud:=" + target_pointcloud
        launch_command = launch_base_command + launch_args
        if scenario_yaml_obj["Evaluation"]["UseCaseName"] == "ndt_convergence":
            # I/O sync for rosbag save
            launch_command += "\nsync\n"
            # add ndt_convergence launch command
            ndt_launch_base_command = (
                "ros2 launch ndt_convergence_evaluation ndt_convergence_evaluation.launch.py"
            )
            ndt_launch_args = " rosbag_file_name:=" + result_bag_path
            ndt_launch_args += " map_path:=" + map_path
            ndt_launch_args += " save_dir:=" + os.path.join(scenario_output_dir, "result_archive")
            launch_command += ndt_launch_base_command + ndt_launch_args
        print("launch command generated! => " + launch_command)
        return launch_command

    def __create_launch_command_with_t4_dataset(
        self, scenario_root: str, scenario_output_dir: str, scenario_yaml_obj
    ) -> Optional[str]:
        launch_command_for_all_dataset = ""
        scenario_path = os.path.join(scenario_root, "scenario.yaml")
        t4_dataset_base_path = os.path.join(scenario_root, "t4_dataset")
        t4_datasets = scenario_yaml_obj["Evaluation"]["Datasets"]
        is_database_evaluation = True if len(t4_datasets) > 1 else False
        for dataset in t4_datasets:
            # get dataset_id
            key = next(iter(dataset))
            # create sub directory for the dataset
            output_dir_per_dataset = os.path.join(scenario_output_dir, key)
            os.makedirs(output_dir_per_dataset)
            vehicle_id = dataset[key].get("VehicleId", "")
            map_path = os.path.expandvars(dataset[key].get("LocalMapPath", ""))
            t4_dataset_path = os.path.join(t4_dataset_base_path, key)
            launch_sensing = str(dataset[key].get("LaunchSensing", False))

            if vehicle_id == "":
                termcolor.cprint(
                    f"vehicle_id is not defined in dataset: {key}",
                    "red",
                )
                continue

            if map_path != "":
                if not os.path.exists(map_path):
                    termcolor.cprint(
                        "map: " + map_path + " used in dataset: " + key + " is not exist ",
                        "red",
                    )
                    continue
            else:
                termcolor.cprint(f"LocalMapPath is not described in {key}.", "red")
                continue

            if not os.path.exists(t4_dataset_path):
                termcolor.cprint(
                    f"t4_dataset: {key} is not exist",
                    "red",
                )
                continue

            use_case_name = scenario_yaml_obj["Evaluation"]["UseCaseName"]
            launch_base_command = f"ros2 launch driving_log_replayer {use_case_name}.launch.py"
            # evaluation component args

            launch_args = " scenario_path:=" + scenario_path
            launch_args += " result_json_path:=" + os.path.join(
                output_dir_per_dataset, "result.json"
            )

            # ros2 bag play args
            launch_args += " play_rate:=" + str(self.__rate)
            launch_args += " play_delay:=" + str(self.__delay)
            launch_args += " input_bag:=" + os.path.join(t4_dataset_path, "input_bag")

            # ros2 bag record args
            result_bag_path = os.path.join(output_dir_per_dataset, "result_bag")
            launch_args += " result_bag_path:=" + result_bag_path

            # logging_simulator.launch args
            launch_args += " map_path:=" + map_path
            launch_args += " vehicle_model:=" + scenario_yaml_obj["VehicleModel"]
            launch_args += " sensor_model:=" + scenario_yaml_obj["SensorModel"]
            launch_args += " vehicle_id:=" + vehicle_id
            launch_args += " rviz:=true"

            # t4_dataset
            launch_args += " t4_dataset_path:=" + t4_dataset_path
            if scenario_yaml_obj["Evaluation"]["UseCaseName"] == "perception":
                launch_args += " result_archive_path:=" + os.path.join(
                    output_dir_per_dataset, "result_archive"
                )
                launch_args += " sensing:=" + launch_sensing
            launch_command = launch_base_command + launch_args + "\n"
            launch_command_for_all_dataset += launch_command
        if is_database_evaluation:
            database_result_script_path = os.path.join(
                self.__autoware_path,
                "install",
                "driving_log_replayer",
                "lib",
                "driving_log_replayer",
                "perception_database_result.py",
            )
            database_result_command = f"python3 {database_result_script_path}"
            database_result_command += f" -s {scenario_path} -r {scenario_output_dir}\n"
            launch_command_for_all_dataset += database_result_command
        print("launch command generated! => " + launch_command_for_all_dataset)
        return launch_command_for_all_dataset
