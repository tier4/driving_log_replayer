from pathlib import Path

import simplejson as json
import termcolor


def load_result(result_path: Path) -> list[dict]:
    if not result_path.exists():
        return []
    with result_path.open() as jsonl_file:
        try:
            return [json.loads(line) for line in jsonl_file]
        except json.JSONDecodeError:
            return []


def load_last_result(result_path: Path) -> dict:
    result_list = load_result(result_path)
    if result_list == []:
        return {}
    return result_list[-1]


def load_final_metrics(result_path: Path) -> dict:
    return load_last_result(result_path).get("Frame", {}).get("FinalMetrics", {})


def display(result_path: Path) -> None:
    print("--------------------------------------------------")  # noqa
    last_result = load_last_result(result_path)
    if last_result:
        # dict is not empty
        if last_result["Result"]["Success"]:
            result = "TestResult: Passed"
            color = "green"
        else:
            result = "TestResult: Failed"
            color = "red"
        termcolor.cprint(result, color)
        termcolor.cprint(last_result["Result"]["Summary"], color)


def display_all(output_directory: Path) -> None:
    result_paths = output_directory.glob("**/result.jsonl")
    for result_path in result_paths:
        print(f"scenario: {result_path.parent.name}")  # noqa
        display(result_path)
