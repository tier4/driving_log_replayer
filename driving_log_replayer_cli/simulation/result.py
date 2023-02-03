import glob
import os
from pathlib import Path

from natsort import natsorted
import simplejson as json
import termcolor


class DrivingLogReplayerResultViewer:
    def __init__(self, result_path):
        self.__result_path = result_path

    def output(self):
        print("--------------------------------------------------")
        self.__result_json_dict = {}
        with open(self.__result_path, "r") as jsonl_file:
            last_line = jsonl_file.readlines()[-1]
            try:
                self.__result_json_dict = json.loads(last_line)
            except json.JSONDecodeError:
                pass
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
    def __init__(self, result_path):
        self.__result_path = result_path

    def convert(self):
        output_file_path = Path(self.__result_path).parent.joinpath("result.json")
        if output_file_path.exists():
            # termcolor.cprint("A json file already exists. Skip convert", "yellow")
            return
        with open(self.__result_path, "r") as jsonl_file:
            result_dict = [json.loads(line) for line in jsonl_file]
            with open(output_file_path, "w") as out_file:
                json.dump(result_dict, out_file)


def display(output_directory: str):
    regex = os.path.join(os.path.expandvars(output_directory), "**", "result.jsonl")
    result_paths = glob.glob(regex, recursive=True)
    number = 1
    total = len(result_paths)
    for result_path in natsorted(result_paths):
        result_path_obj = Path(result_path)
        log_directory = result_path_obj.parent
        print(f"test case {number} / {total} : use case: {log_directory.name}")
        viewer = DrivingLogReplayerResultViewer(result_path_obj.as_posix())
        viewer.output()
        number = number + 1


def convert(output_directory: str):
    regex = os.path.join(os.path.expandvars(output_directory), "**", "result.jsonl")
    result_paths = glob.glob(regex, recursive=True)
    for result_path in natsorted(result_paths):
        result_path_obj = Path(result_path)
        converter = DrivingLogReplayerResultConverter(result_path_obj.as_posix())
        converter.convert()
