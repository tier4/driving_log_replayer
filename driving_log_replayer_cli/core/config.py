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

    from os.path import exists

    if exists(filepath):
        with open(filepath) as fp:
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
    from os.path import exists

    if not exists(filepath):
        from driving_log_replayer_cli.core.exception import UserError

        error_msg = "Configuration file is not found."
        raise UserError(error_msg)  # EM101

    with open(filepath) as fp:
        return toml.load(fp)


def _save_as_file(data: dict[str, dict], filepath: str) -> None:
    from os import makedirs
    from os.path import dirname

    makedirs(dirname(filepath), exist_ok=True)

    with open(filepath, mode="w") as fp:
        toml.dump(data, fp)

    from os import chmod
    from stat import S_IREAD
    from stat import S_IWRITE

    chmod(filepath, S_IREAD | S_IWRITE)


def _default_filepath() -> str:
    from os.path import expanduser
    from os.path import join

    return join(expanduser("~"), DEFAULT_CONFIG_FILE)
