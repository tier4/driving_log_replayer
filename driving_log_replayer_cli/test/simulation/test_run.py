from driving_log_replayer_cli.simulation.run import args_to_dict
from driving_log_replayer_cli.simulation.run import launch_dict_to_str


def test_extract_single_arg() -> None:
    args = args_to_dict(["rate:=0.5"])
    assert launch_dict_to_str(args) == " rate:=0.5"


def test_extract_multiple_args() -> None:
    args = args_to_dict(["rate:=0.5", "delay:=2.0"])
    assert launch_dict_to_str(args) == " rate:=0.5 delay:=2.0"


def test_extract_no_arg() -> None:
    args = args_to_dict("not_arg")
    assert launch_dict_to_str(args) == ""


def test_extract_dict_string() -> None:
    # annotationless perception argument
    args = args_to_dict(
        [
            'annotationless_pass_range:={"CAR":{"min":"0.0-1.1","max":"0.0-1.2","mean":"0.5-1.3"},"BUS":{"min":"0.0-1.1","max":"0.0-1.2","mean":"0.5-1.3"}}',
            "annotationless_threshold_file:=/home/autoware/result.jsonl",
        ],
    )
    assert (
        launch_dict_to_str(args)
        == ' \'annotationless_pass_range:={"CAR":{"min":"0.0-1.1","max":"0.0-1.2","mean":"0.5-1.3"},"BUS":{"min":"0.0-1.1","max":"0.0-1.2","mean":"0.5-1.3"}}\' annotationless_threshold_file:=/home/autoware/result.jsonl'
    )
    # confirm json load
    json_str = '{"CAR":{"min":"0.0-1.1","max":"0.0-1.2","mean":"0.5-1.3"},"BUS":{"min":"0.0-1.1","max":"0.0-1.2","mean":"0.5-1.3"}}'
    import json

    json.loads(json_str)  # check valid json string


def test_extract_array_string() -> None:
    # pose_initializer argument
    args = args_to_dict(["initial_pose:=[89571.150,42301.188,-3.159,-0.009,-0.002,0.288,0.958]"])
    assert (
        launch_dict_to_str(args)
        == " 'initial_pose:=[89571.150,42301.188,-3.159,-0.009,-0.002,0.288,0.958]'"
    )


def test_override_arg() -> None:
    scenario_arg = {"vehicle_model": "sample_vehicle"}
    args = args_to_dict(["vehicle_model:=lexus"])
    scenario_arg.update(args)
    assert launch_dict_to_str(scenario_arg) == " vehicle_model:=lexus"
