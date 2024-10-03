from os.path import expandvars
from pathlib import Path

from pydantic import BaseModel
from pydantic import field_serializer
from pydantic import field_validator
import toml

from driving_log_replayer_cli.core.exception import UserError

DEFAULT_CONFIG_FILE = ".driving_log_replayer.config.toml"


class Config(BaseModel):
    data_directory: Path
    output_directory: Path
    autoware_path: Path

    @field_validator("data_directory", "autoware_path", mode="before")
    @classmethod
    def validate_path(cls, v: str) -> Path:
        normal_path = Path(expandvars(v))
        if normal_path.exists():
            return normal_path
        err_msg = f"{v} is not valid path"
        raise UserError(err_msg)

    @field_validator("output_directory", mode="before")
    @classmethod
    def validate_out_dir(cls, v: str) -> Path:
        normal_path = Path(expandvars(v))
        normal_path.mkdir(parents=True, exist_ok=True)
        if normal_path.exists():
            return normal_path
        err_msg = f"{v} is not valid path"
        raise UserError(err_msg)

    @field_serializer("data_directory", "output_directory", "autoware_path")
    def serialize_path(self, v: Path) -> None:
        return v.as_posix()


def load_config(profile: str, filepath: Path | None = None) -> Config:
    if filepath is None:
        filepath = _default_filepath()

    config_data = _load_from_file(filepath)

    if profile not in config_data:
        error_msg = f"Not found profile: {profile}"
        raise UserError(error_msg)

    return Config(**config_data[profile])


def save_config(config: Config, profile: str, filepath: Path | None = None) -> None:
    if filepath is None:
        filepath = _default_filepath()

    config_data = {}

    if filepath.exists():
        with filepath.open() as fp:
            config_data = toml.load(fp)
    config_data[profile] = config.model_dump()
    _save_as_file(config_data, filepath)


def remove_config(profile: str, filepath: Path | None = None) -> Config:
    if filepath is None:
        filepath = _default_filepath()

    config_data = _load_from_file(filepath)

    if profile not in config_data:
        error_msg = f"Not found profile: {profile}"
        raise UserError(error_msg)

    config = config_data[profile]
    del config_data[profile]

    _save_as_file(config_data, filepath)

    return Config(**config)


def _load_from_file(filepath: Path) -> dict[str, dict]:
    if not filepath.exists():
        error_msg = f"Configuration file {filepath} was not found."
        raise UserError(error_msg)
    with filepath.open() as fp:
        return toml.load(fp)


def _save_as_file(data: dict[str, dict], filepath: Path) -> None:
    filepath.parent.mkdir(exist_ok=True)

    with filepath.open("w") as fp:
        toml.dump(data, fp)

    filepath.chmod(0o600)


def _default_filepath() -> Path:
    return Path("~", DEFAULT_CONFIG_FILE).expanduser()
