import datetime
from pathlib import Path

import click

from driving_log_replayer_cli.core.config import Config
from driving_log_replayer_cli.core.config import load_config
from driving_log_replayer_cli.simulation.result import convert
from driving_log_replayer_cli.simulation.result import display
from driving_log_replayer_cli.simulation.run import run as sim_run

CONTEXT_SETTINGS = {"help_option_names": ["-h", "--help"]}
PERCEPTION_MODES = (
    "camera_lidar_radar_fusion",
    "camera_lidar_fusion",
    "lidar_radar_fusion",
    "lidar",
    "radar",
)


@click.group(context_settings=CONTEXT_SETTINGS)
def simulation() -> None:
    """Run simulation and check simulation log."""


@simulation.command(context_settings=CONTEXT_SETTINGS)
@click.option("--profile", "-p", type=str, default="default")
@click.option("--rate", "-r", default=1.0)
@click.option("--delay", "-d", default=10.0)
@click.option("--no-json", is_flag=True, help="Do not convert jsonl files to json")
@click.option(
    "--perception-mode",
    "-m",
    type=click.Choice(PERCEPTION_MODES, case_sensitive=False),
)
@click.option("--override_record_topics", "-o", default=False, type=bool)
@click.option("--override_topics_regex", "-x", default="/override_unused", type=str)
def run(
    profile: str,
    rate: float,
    delay: float,
    no_json: bool,  # noqa
    override_record_topics: bool,  # noqa
    override_topics_regex: str,
    perception_mode: str | None = None,
) -> None:
    config: Config = load_config(profile)
    output_dir_by_time = config.output_directory.joinpath(
        datetime.datetime.now().strftime("%Y-%m%d-%H%M%S")  # noqa
    )
    print(output_dir_by_time)  # noqa
    sim_run(
        config.data_directory,
        output_dir_by_time,
        config.autoware_path,
        rate,
        delay,
        perception_mode,
        override_record_topics,
        override_topics_regex,
        not no_json,
    )


@simulation.command(context_settings=CONTEXT_SETTINGS)
@click.argument("output_directory", type=click.Path(exists=True, file_okay=False))
def show_result(output_directory: str) -> None:
    """Show summary of simulation results in output_directory."""
    display(Path(output_directory).resolve())  # support latest dir(symlink)


@simulation.command(context_settings=CONTEXT_SETTINGS)
@click.argument("output_directory", type=click.Path(exists=True, file_okay=False))
def convert_result(output_directory: str) -> None:
    """Convert result.jsonl to result.json in output_directory."""
    convert(Path(output_directory).resolve())  # support latest dir(symlink)
