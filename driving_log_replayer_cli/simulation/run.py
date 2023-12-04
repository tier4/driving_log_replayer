from os.path import expandvars
from pathlib import Path
import subprocess
import sys

import termcolor

from driving_log_replayer_cli.simulation.generate import TestScriptGenerator
from driving_log_replayer_cli.simulation.result import convert
from driving_log_replayer_cli.simulation.result import display


def run_with_log(cmd: list, log_path: Path) -> None:
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    f = log_path.open("w", encoding="utf-8")

    while True:
        line = proc.stdout.readline().decode("utf-8")
        sys.stdout.write(line)
        f.write(line)
        if not line and proc.poll() is not None:
            break
    f.close()


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
        cmd = ["/bin/bash", generator.script_path.as_posix()]
        run_with_log(cmd, Path(self.__output_directory, "console.log"))
        if output_json:
            convert(self.__output_directory)
        termcolor.cprint("<< show test result >>", "green")
        display(self.__output_directory)
