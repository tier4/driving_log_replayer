import contextlib
from pathlib import Path

import simplejson as json
import termcolor


def display(result_path: Path) -> None:
    print("--------------------------------------------------")  # noqa
    result_json_dict = {}
    with result_path.open() as jsonl_file:
        last_line = jsonl_file.readlines()[-1]
        with contextlib.suppress(json.JSONDecodeError):
            result_json_dict = json.loads(last_line)
    if result_json_dict:
        # dict is not empty
        if result_json_dict["Result"]["Success"]:
            result = "TestResult: Passed"
            color = "green"
        else:
            result = "TestResult: Failed"
            color = "red"
        termcolor.cprint(result, color)
        termcolor.cprint(result_json_dict["Result"]["Summary"], color)


def convert(result_path: Path) -> None:
    output_file_path = result_path.parent.joinpath("result.json")
    if output_file_path.exists():
        return
    with result_path.open() as jsonl_file:
        result_dict = [json.loads(line) for line in jsonl_file]
    with output_file_path.open("w") as out_file:
        json.dump(result_dict, out_file)


def display_all(output_directory: Path) -> None:
    result_paths = output_directory.glob("**/result.jsonl")
    for result_path in result_paths:
        print(f"scenario: {result_path.parent.name}")  # noqa
        display(result_path)


def convert_all(output_directory: Path) -> None:
    result_paths = output_directory.glob("**/result.jsonl")
    for result_path in result_paths:
        convert(result_path)
