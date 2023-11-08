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
        perception_mode: str,
    ) -> None:
        termcolor.cprint("<< generating launch command >>", "green")
        generator = TestScriptGenerator(
            data_directory,
            output_directory,
            autoware_path,
            rate,
            delay,
            perception_mode,
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
