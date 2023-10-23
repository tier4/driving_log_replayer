from pathlib import Path
from typing import Any
from typing import NamedTuple
from typing import overload

import toml

DEFAULT_CONFIG_FILE = ".driving_log_replayer.config.toml"


class Config(NamedTuple):
    data_directory: str
    output_directory: str
    autoware_path: str

    def as_dict(self) -> dict[str, Any]:
        return self._asdict()


@overload
def load_config(profile: str, filepath: str) -> Config:
    ...


@overload
def load_config(profile: str, filepath: None = None) -> Config:
    ...


def load_config(profile: str, filepath: str | None = None) -> Config:
    if filepath is None:
        filepath = _default_filepath()

    config_data = _load_from_file(filepath)

    if profile not in config_data:
        from driving_log_replayer_cli.core.exception import UserError

        error_msg = f"Not found profile: {profile}"
        raise UserError(error_msg)  # EM102

    return Config(**config_data[profile])


@overload
def save_config(config: Config, profile: str, filepath: str) -> None:
    ...


@overload
def save_config(config: Config, profile: str, filepath: None = None) -> None:
    ...


def save_config(config: Config, profile: str, filepath: str | None = None):
    if filepath is None:
        filepath = _default_filepath()

    config_data = {}

    if Path(filepath).exists():
        with Path(filepath).open() as fp:
            config_data = toml.load(fp)

    config_data[profile] = config.as_dict()

    _save_as_file(config_data, filepath)


@overload
def remove_config(profile: str, filepath: str) -> Config:
    ...


@overload
def remove_config(profile: str, filepath: None = None) -> Config:
    ...


def remove_config(profile: str, filepath: str | None = None) -> Config:
    if filepath is None:
        filepath = _default_filepath()

    config_data = _load_from_file(filepath)

    if profile not in config_data:
        from driving_log_replayer_cli.core.exception import UserError

        error_msg = f"Not found profile: {profile}"
        raise UserError(error_msg)  # EM102

    config = config_data[profile]
    del config_data[profile]

    _save_as_file(config_data, filepath)

    return Config(**config)


def _load_from_file(filepath: str) -> dict[str, dict]:
    if not Path(filepath).exists():
        from driving_log_replayer_cli.core.exception import UserError

        error_msg = "Configuration file is not found."
        raise UserError(error_msg)  # EM101

    with Path(filepath).open() as fp:
        return toml.load(fp)


def _save_as_file(data: dict[str, dict], filepath: str) -> None:
    Path(filepath).parent.mkdir(exist_ok=True)

    with Path(filepath).open("w") as fp:
        toml.dump(data, fp)

    Path(filepath).chmod(0o600)


def _default_filepath() -> str:
    return Path("~").expanduser().joinpath(DEFAULT_CONFIG_FILE)
