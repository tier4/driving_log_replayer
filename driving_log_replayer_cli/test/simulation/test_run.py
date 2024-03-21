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
