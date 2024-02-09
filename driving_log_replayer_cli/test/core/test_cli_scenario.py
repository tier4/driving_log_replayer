from pathlib import Path

from driving_log_replayer_cli.core.scenario import Datasets
from driving_log_replayer_cli.core.scenario import load_scenario


def test_datasets() -> None:
    scenario = load_scenario(Path(__file__).parent.joinpath("sample.scenario.yaml"))
    datasets = Datasets(Datasets=scenario.Evaluation["Datasets"])
    assert datasets.Datasets[0]["sample_dataset"].LocalMapPath.as_posix() == "."
