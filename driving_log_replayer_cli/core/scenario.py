from os.path import expandvars
from pathlib import Path

from pydantic import BaseModel
from pydantic import field_validator
from typing_extensions import Literal
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

    @field_validator("LocalMapPath")
    def validate_local_path(cls, v: str | None) -> Path | None:  # noqa
        if v is None:
            return None
        normal_path = Path(expandvars(v))
        if normal_path.exists():
            return normal_path
        err_msg = f"{v} is not valid path"
        raise UserError(err_msg)


def load_scenario(scenario_path: Path) -> Scenario:
    if scenario_path.is_symlink():
        scenario_path = scenario_path.resolve()
    with scenario_path.open() as scenario_file:
        return Scenario(**yaml.safe_load(scenario_file))


class Dataset(BaseModel):
    VehicleId: str
    LocalMapPath: Path
    LaunchSensing: bool | None = None

    @field_validator("LocalMapPath")
    def validate_local_path(cls, v: str | None) -> Path | None:  # noqa
        if v is None:
            return None
        normal_path = Path(expandvars(v))
        if normal_path.exists():
            return normal_path
        err_msg = f"{v} is not valid path"
        raise UserError(err_msg)


class Datasets(BaseModel):
    Datasets: list[dict[str, Dataset]]
