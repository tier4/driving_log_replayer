from datetime import datetime
from os.path import expandvars
from pathlib import Path
import shutil
from typing import Literal

from pydantic import BaseModel
from pydantic import field_serializer
from pydantic import field_validator
import yaml

from driving_log_replayer_cli.core.exception import UserError


class Scenario(BaseModel):
    ScenarioFormatVersion: Literal["2.2.0", "3.0.0"]
    ScenarioName: str
    ScenarioDescription: str
    SensorModel: str
    VehicleModel: str
    VehicleId: str | None = None
    LocalMapPath: Path | None = None
    Evaluation: dict

    @field_validator("LocalMapPath", mode="before")
    @classmethod
    def validate_local_path(cls, v: str | None) -> Path | None:
        if v is None:
            return None
        normal_path = Path(expandvars(v))
        if normal_path.exists():
            return normal_path
        err_msg = f"{v} is not valid path"
        raise UserError(err_msg)

    @field_serializer("LocalMapPath")
    def serialize_path(self, v: Path) -> None:
        return v.as_posix()

    def dump(self, save_path: Path) -> None:
        with save_path.open("w") as file:
            yaml.safe_dump(self.model_dump(), file, sort_keys=False)


def load_scenario(scenario_path: Path) -> Scenario:
    if scenario_path.is_symlink():
        scenario_path = scenario_path.resolve()
    with scenario_path.open() as scenario_file:
        return Scenario(**yaml.safe_load(scenario_file))


def backup_scenario_file(scenario_path: Path) -> None:
    bak_name = scenario_path.name + f".{datetime.now().strftime('%Y%m%d%H%M%S')}.bak"  # noqa
    backup_file_path = scenario_path.parent.joinpath(bak_name)
    shutil.copy(scenario_path, backup_file_path)
    return backup_file_path


class Dataset(BaseModel):
    VehicleId: str
    LocalMapPath: Path
    LaunchSensing: bool | None = None

    @field_validator("LocalMapPath", mode="before")
    @classmethod
    def validate_local_path(cls, v: str | None) -> Path | None:
        if v is None:
            return None
        normal_path = Path(expandvars(v))
        if normal_path.exists():
            return normal_path
        err_msg = f"{v} is not valid path"
        raise UserError(err_msg)


class Datasets(BaseModel):
    Datasets: list[dict[str, Dataset]]
