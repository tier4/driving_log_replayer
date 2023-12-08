from pathlib import Path

import pytest

from driving_log_replayer_cli.core.config import Config
from driving_log_replayer_cli.core.config import load_config
from driving_log_replayer_cli.core.config import save_config
from driving_log_replayer_cli.core.exception import UserError


def test_config_create_output_dir() -> None:
    conf: Config = Config(
        data_directory=".",
        output_directory="$HOME/dlr_out_unittest",
        autoware_path=".",
    )
    assert conf.output_directory.exists()
    conf.output_directory.rmdir()


def test_config_not_exit_dir() -> None:
    with pytest.raises(UserError) as e:
        Config(
            data_directory="$HOME/not_exit_data",
            output_directory=".",
            autoware_path=".",
        )
    assert str(e.value) == "$HOME/not_exit_data is not valid path"


def test_save_config_as_file() -> None:
    config = load_config(
        "default",
        Path(__file__).parent.joinpath("sample.driving_log_replayer.config.toml"),
    )
    save_config(config, "copy_default", Path(__file__).parent.joinpath("unittest.config.toml"))
