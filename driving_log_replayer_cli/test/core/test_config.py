import pytest

from driving_log_replayer_cli.core.config import Config
from driving_log_replayer_cli.core.config import load_config
from driving_log_replayer_cli.core.config import save_config
from driving_log_replayer_cli.core.exception import UserError


def test_config_create_output_dir() -> None:
    conf = Config(
        data_directory="$HOME/data/awf",
        output_directory="$HOME/out/unittest",
        autoware_path="$HOME/ros_ws/awf",
    )
    assert conf.autoware_path.as_posix() == "/home/hyt/ros_ws/awf"


def test_config_not_exit_dir() -> None:
    with pytest.raises(UserError) as e:
        Config(
            data_directory="$HOME/not_exit_data",
            output_directory="$HOME/out/unittest",
            autoware_path="$HOME/ros_ws/awf",
        )
    assert str(e.value) == "$HOME/not_exit_data is not valid path"


def test_load_config_from_file() -> None:
    config = load_config("default")
    assert config.autoware_path.as_posix() == "/home/hyt/ros_ws/awf"


def test_save_config_as_file() -> None:
    config = load_config("default")
    save_config(config, "copy_default")
