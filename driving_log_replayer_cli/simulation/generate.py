def _cmd_use_t4_dataset(
    scenario_path: Path,
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
        launch_args = _create_common_arg(
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
        launch_args += f" result_archive_path:={output_dir_per_dataset.joinpath('result_archive')}"
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
