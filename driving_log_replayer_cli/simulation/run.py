import os
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
        output_json: bool,  # noqa
    ) -> None:
        self.__data_directory = data_directory
        self.__output_directory = output_directory
        self.__autoware_path = autoware_path
        self.__rate = rate
        self.__delay = delay
        self.__output_json = output_json

        termcolor.cprint("<< generating launch command >>", "green")
        generator = TestScriptGenerator(
            self.__data_directory,
            self.__output_directory,
            self.__autoware_path,
            self.__rate,
            self.__delay,
        )
        is_executable = generator.run()
        if is_executable:
            cmd = "/bin/bash " + generator.script_path
            subprocess.run(cmd, shell=True)
            if self.__output_json:
                convert(self.__output_directory)
            termcolor.cprint("<< show test result >>", "green")
            display(self.__output_directory)
        else:
            print("aborted.")  # noqa


def run(
    data_directory: str,
    output_directory: str,
    autoware_path: str,
    rate: float,
    delay: float,
    output_json: bool,  # noqa
) -> None:
    DrivingLogReplayerTestRunner(
        os.path.expandvars(data_directory),
        os.path.expandvars(output_directory),
        os.path.expandvars(autoware_path),
        rate,
        delay,
        output_json=output_json,
    )
