import contextlib
from os.path import expandvars
from pathlib import Path

import simplejson as json
import termcolor


class DrivingLogReplayerResultViewer:
    def __init__(self, result_path: str) -> None:
        self.__result_path = Path(result_path)

    def output(self) -> None:
        print("--------------------------------------------------")  # noqa
        self.__result_json_dict = {}
        with self.__result_path.open() as jsonl_file:
            last_line = jsonl_file.readlines()[-1]
            with contextlib.suppress(json.JSONDecodeError):
                self.__result_json_dict = json.loads(last_line)
        if self.__result_json_dict:
            # dict is not empty
            if self.__result_json_dict["Result"]["Success"]:
                result = "TestResult: Passed"
                color = "green"
            else:
                result = "TestResult: Failed"
                color = "red"
            termcolor.cprint(result, color)
            termcolor.cprint(self.__result_json_dict["Result"]["Summary"], color)


class DrivingLogReplayerResultConverter:
    def __init__(self, result_path: str) -> None:
        self.__result_path = Path(result_path)

    def convert(self) -> None:
        output_file_path = self.__result_path.parent.joinpath("result.json")
        if output_file_path.exists():
            return
        with self.__result_path.open() as jsonl_file:
            result_dict = [json.loads(line) for line in jsonl_file]
            with output_file_path.open("w") as out_file:
                json.dump(result_dict, out_file)


def display(output_directory: str) -> None:
    result_paths = Path(expandvars(output_directory)).glob("**/result.jsonl")
    number = 1
    total = len(result_paths)
    for result_path in result_paths:
        print(f"test case {number} / {total} : use case: {result_path.parent.name}")  # noqa
        viewer = DrivingLogReplayerResultViewer(result_path.as_posix())
        viewer.output()
        number = number + 1


def convert(output_directory: str) -> None:
    result_paths = Path(expandvars(output_directory)).glob("**/result.jsonl")
    for result_path in result_paths:
        converter = DrivingLogReplayerResultConverter(result_path.as_posix())
        converter.convert()
