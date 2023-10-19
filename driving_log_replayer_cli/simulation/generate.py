from pathlib import Path
import subprocess

import termcolor
import yaml


class TestScriptGenerator:
    PERCEPTION_MODES = (
        "camera_lidar_radar_fusion",
        "camera_lidar_fusion",
        "lidar_radar_fusion",
        "lidar",
        "radar",
    )

    USE_T4_DATASET = ("perception", "obstacle_segmentation", "perception_2d", "traffic_light")

    def __init__(
        self,
        data_directory: str,
        output_directory: str,
        autoware_path: str,
        rate: float,
        delay: float,
    ) -> None:
        self.__data_directory = Path(data_directory)
        self.__output_directory = Path(output_directory)
        self.__autoware_path = Path(autoware_path)
        self.__script_path = self.__output_directory.joinpath("run.bash")
        self.__rate = rate
        self.__delay = delay
        #  os.path.join(config.output_directory, datetime.datetime.now().strftime("%Y-%m%d-%H%M%S"))が渡ってくるので被ることはない
        self.__output_directory.mkdir()

        symlink_dst = self.__output_directory.parent.joinpath("latest").as_posix()
        update_symlink = f"ln -snf {self.__output_directory.as_posix()} {symlink_dst}"
        subprocess.run(update_symlink, shell=True)

    @property
    def script_path(self) -> Path:
        return self.__script_path

    def run(self) -> bool:
        if not self.__output_directory.exists():
            return False
        generated_cmd = (
            f"source {self.__autoware_path.joinpath('install', 'setup.bash').as_posix()}\n"
        )
        for dataset_path in self.__data_directory.glob("**"):
            # pathlib.Path.glob("**") detect directories only
            launch_command = self.__parse_scenario(dataset_path)
            if launch_command is None:
                continue
            # echo return value of ros2 launch (0: ok, others: ng)
            generated_cmd += "echo $?\n"
            # kill zombie ros2 process
            generated_cmd += 'pgrep ros | awk \'{ print "kill -9 $(pgrep -P ", $1, ") > /dev/null 2>&1" }\' | sh\n'
            generated_cmd += (
                'pgrep ros | awk \'{ print "kill -9 ", $1, " > /dev/null 2>&1" }\' | sh\n'
            )
            # kill rviz awk '{ print \"kill -9\", $2 }'
            generated_cmd += 'pgrep rviz | awk \'{ print "kill -9 $(pgrep -P ", $1, ") > /dev/null 2>&1" }\' | sh\n'
            generated_cmd += (
                'pgrep rviz | awk \'{ print "kill -9 ", $1, " > /dev/null 2>&1" }\' | sh\n'
            )
            # sleep 1 sec
            generated_cmd += "sleep 1\n"
        # create shell script file
        with open(self.script_path, "w") as f:
            f.write(generated_cmd)
        return True

    def __parse_scenario(self, dataset_path: Path) -> str | None:
        scenario_output_dir = self.__output_directory.joinpath(dataset_path.name)
        scenario_output_dir.mkdir()
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
                return self.__create_launch_command_with_t4_dataset(
                    dataset_path,
                    scenario_path.name,
                    scenario_output_dir,
                    scenario_yaml_obj,
                )
            return self.__create_launch_command(
                dataset_path,
                scenario_path.name,
                scenario_output_dir,
                scenario_yaml_obj,
            )

    def __create_launch_command(
        self,
        scenario_root: Path,
        scenario_name: str,
        scenario_output_dir: Path,
        scenario_yaml_obj: dict,
    ) -> str | None:
        map_path = Path(scenario_yaml_obj.get("LocalMapPath", "/dlr_not_exist_dir"))
        if not map_path.exists():
            termcolor.cprint(
                f"map: {map_path.as_posix()} used in scenario: {scenario_root.as_posix()} does not exist ",
                "red",
            )
            return None
        input_bag = scenario_root.joinpath("input_bag")
        if not input_bag.exists():
            termcolor.cprint(f"input_bag file {input_bag.as_posix()} does not exist.", "red")
            return None
        use_case_name = scenario_yaml_obj["Evaluation"]["UseCaseName"]
        launch_base_command = f"ros2 launch driving_log_replayer {use_case_name}.launch.py"
        # evaluation component args
        launch_args = " scenario_path:=" + os.path.join(scenario_root, scenario_name)
        launch_args += " result_json_path:=" + os.path.join(scenario_output_dir, "result.json")

        # ros2 bag play args
        launch_args += " play_rate:=" + str(self.__rate)
        launch_args += " play_delay:=" + str(self.__delay)
        launch_args += " input_bag:=" + input_bag.as_posix()

        # ros2 bag record args
        result_bag_path = os.path.join(scenario_output_dir, "result_bag")
        launch_args += " result_bag_path:=" + result_bag_path

        # logging_simulator.launch args
        launch_args += " map_path:=" + map_path
        launch_args += " vehicle_model:=" + scenario_yaml_obj["VehicleModel"]
        launch_args += " sensor_model:=" + scenario_yaml_obj["SensorModel"]
        launch_args += " vehicle_id:=" + scenario_yaml_obj["VehicleId"]
        launch_args += " rviz:=true"
        # diag launch localization
        if scenario_yaml_obj["Evaluation"]["UseCaseName"] == "performance_diag":
            launch_args += " localization:=" + str(
                scenario_yaml_obj["Evaluation"]["LaunchLocalization"],
            )
        if scenario_yaml_obj["Evaluation"]["UseCaseName"] == "obstacle_detection":
            launch_planning = scenario_yaml_obj["Evaluation"].get("LaunchPlanning", False)
            launch_args += " planning:=" + str(launch_planning)
            target_pointcloud = scenario_yaml_obj["Evaluation"]["Conditions"].get(
                "TargetPointCloud",
                "/perception/obstacle_segmentation/pointcloud",
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
        print("launch command generated! => " + launch_command)  # noqa
        return launch_command

    def __create_launch_command_with_t4_dataset(  # noqa TODO: fix logic
        self,
        scenario_root: str,
        scenario_name: str,
        scenario_output_dir: str,
        scenario_yaml_obj: dict,
    ) -> str | None:
        launch_command_for_all_dataset = ""
        scenario_path = os.path.join(scenario_root, scenario_name)
        t4_dataset_base_path = os.path.join(scenario_root, "t4_dataset")
        t4_datasets = scenario_yaml_obj["Evaluation"]["Datasets"]
        is_database_evaluation = bool(len(t4_datasets) > 1)
        for dataset in t4_datasets:
            # get dataset_id
            key = next(iter(dataset))
            # create sub directory for the dataset
            output_dir_per_dataset = os.path.join(scenario_output_dir, key)
            os.makedirs(output_dir_per_dataset)
            vehicle_id = dataset[key].get("VehicleId", "")
            map_path = os.path.expandvars(dataset[key].get("LocalMapPath", ""))
            t4_dataset_path = os.path.join(t4_dataset_base_path, key)
            launch_sensing = str(dataset[key].get("LaunchSensing", True))

            if vehicle_id == "":
                termcolor.cprint(
                    f"vehicle_id is not defined in dataset: {key}",
                    "red",
                )
                continue

            if map_path != "":
                if not os.path.exists(map_path):
                    termcolor.cprint(
                        "map: " + map_path + " used in dataset: " + key + " does not exist ",
                        "red",
                    )
                    continue
            else:
                termcolor.cprint(f"LocalMapPath is not described in {key}.", "red")
                continue

            if not os.path.exists(t4_dataset_path):
                termcolor.cprint(
                    f"t4_dataset: {key} does not exist",
                    "red",
                )
                continue

            use_case_name = scenario_yaml_obj["Evaluation"]["UseCaseName"]
            launch_base_command = f"ros2 launch driving_log_replayer {use_case_name}.launch.py"
            # evaluation component args

            launch_args = " scenario_path:=" + scenario_path
            launch_args += " result_json_path:=" + os.path.join(
                output_dir_per_dataset,
                "result.json",
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
            if scenario_yaml_obj.get("PerceptionMode") is not None:
                perception_mode: str = scenario_yaml_obj["PerceptionMode"]
                assert perception_mode in self.PERCEPTION_MODES, (
                    f"perception_mode must be chosen from {self.PERCEPTION_MODES}, "
                    f"but got {perception_mode}"
                )
                launch_args += " perception_mode:=" + perception_mode

            # t4_dataset
            launch_args += " t4_dataset_path:=" + t4_dataset_path
            launch_args += " result_archive_path:=" + os.path.join(
                output_dir_per_dataset,
                "result_archive",
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
        print("launch command generated! => " + launch_command_for_all_dataset)  # noqa
        return launch_command_for_all_dataset
