from os.path import expandvars
import subprocess

import termcolor

from driving_log_replayer_cli.simulation.generate import TestScriptGenerator
from driving_log_replayer_cli.simulation.result import convert
from driving_log_replayer_cli.simulation.result import display


class DrivingLogReplayerTestRunner:
    def __init__(
        self,
        data_directory: str,
        output_directory: str,
        autoware_path: str,
        rate: float,
        delay: float,
        perception_mode: str,
        override_record_topics: bool,  # noqa
        override_topics_regex: str,
        output_json: bool,  # noqa
    ) -> None:
        self.__data_directory = expandvars(data_directory)
        self.__output_directory = expandvars(output_directory)
        self.__autoware_path = expandvars(autoware_path)
        termcolor.cprint("<< generating launch command >>", "green")
        generator = TestScriptGenerator(
            self.__data_directory,
            self.__output_directory,
            self.__autoware_path,
            rate,
            delay,
            perception_mode,
            override_record_topics,
            override_topics_regex,
        )
        is_executable = generator.run()
        if not is_executable:
            print("aborted.")  # noqa
            return
        cmd = "/bin/bash " + generator.script_path.as_posix()
        subprocess.run(cmd, shell=True)
        if output_json:
            convert(self.__output_directory)
        termcolor.cprint("<< show test result >>", "green")
        display(self.__output_directory)
