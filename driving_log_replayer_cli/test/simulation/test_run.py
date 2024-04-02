from driving_log_replayer_cli.simulation.run import extract_arg


def test_extract_single_arg() -> None:
    args = extract_arg(["rate:=0.5"])
    assert args == " rate:=0.5"


def test_extract_multiple_args() -> None:
    args = extract_arg(["rate:=0.5", "delay:=2.0"])
    assert args == " rate:=0.5 delay:=2.0"


def test_extract_no_arg() -> None:
    args = extract_arg("not_arg")
    assert args == ""


def test_extract_dict_string() -> None:
    # annotationless perception argument
    args = extract_arg(
        [
            'annotationless_pass_range:={"CAR":{"min":"0.0-1.1","max":"0.0-1.2","mean":"0.5-1.3"},"BUS":{"min":"0.0-1.1","max":"0.0-1.2","mean":"0.5-1.3"}}',
            "annotationless_threshold_file:=/home/autoware/result.jsonl",
        ],
    )
    assert (
        args
        == ' \'annotationless_pass_range:={"CAR":{"min":"0.0-1.1","max":"0.0-1.2","mean":"0.5-1.3"},"BUS":{"min":"0.0-1.1","max":"0.0-1.2","mean":"0.5-1.3"}}\' annotationless_threshold_file:=/home/autoware/result.jsonl'
    )
    # confirm json load
    json_str = '{"CAR":{"min":"0.0-1.1","max":"0.0-1.2","mean":"0.5-1.3"},"BUS":{"min":"0.0-1.1","max":"0.0-1.2","mean":"0.5-1.3"}}'
    import json

    json.loads(json_str)  # check valid json string


def test_extract_array_string() -> None:
    # pose_initializer argument
    args = extract_arg(["initial_pose:=[89571.150,42301.188,-3.159,-0.009,-0.002,0.288,0.958]"])
    assert args == " 'initial_pose:=[89571.150,42301.188,-3.159,-0.009,-0.002,0.288,0.958]'"
