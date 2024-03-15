from driving_log_replayer_cli.simulation.run import extract_arg


def test_extract_single_arg() -> None:
    args = extract_arg("rate:=0.5")
    assert args == " rate:=0.5"


def test_extract_multiple_args() -> None:
    args = extract_arg("rate:=0.5,delay:=2.0")
    assert args == " rate:=0.5 delay:=2.0"


def test_extract_no_arg() -> None:
    args = extract_arg("not_arg")
    assert args == ""


def test_extract_dict() -> None:
    args = extract_arg(
        'annotationless_threshold_file:=/home/autoware/result.jsonl,annotationless_pass_range:={"CAR":"0.0-1.1","BUS":"0.2-1.2"}',
    )
    assert (
        args
        == ' annotationless_threshold_file:=/home/autoware/result.jsonl annotationless_pass_range:=\'{"CAR":"0.0-1.1","BUS":"0.2-1.2"}\''
    )
