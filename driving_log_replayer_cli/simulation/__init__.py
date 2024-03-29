from pathlib import Path

import click

from driving_log_replayer_cli.core.config import Config
from driving_log_replayer_cli.core.config import load_config
from driving_log_replayer_cli.core.result import convert_all
from driving_log_replayer_cli.core.result import display_all
from driving_log_replayer_cli.simulation.run import run as sim_run

CONTEXT_SETTINGS = {"help_option_names": ["-h", "--help"]}


@click.group(context_settings=CONTEXT_SETTINGS)
def simulation() -> None:
    """Run simulation and check simulation log."""


@simulation.command(context_settings=CONTEXT_SETTINGS)
@click.option("--profile", "-p", type=str, default="default")
@click.option("--launch_args", "-l", type=str, default="")
def run(
    profile: str,
    launch_args: str,
) -> None:
    config: Config = load_config(profile)
    sim_run(
        config,
        launch_args,
    )


@simulation.command(context_settings=CONTEXT_SETTINGS)
@click.argument(
    "output_directory",
    type=click.Path(exists=True, file_okay=False, resolve_path=True, path_type=Path),
)
def show_result(output_directory: Path) -> None:
    """Show summary of simulation results in output_directory."""
    display_all(output_directory)


@simulation.command(context_settings=CONTEXT_SETTINGS)
@click.argument(
    "output_directory",
    type=click.Path(exists=True, file_okay=False, resolve_path=True, path_type=Path),
)
def convert_result(output_directory: Path) -> None:
    """Convert result.jsonl to result.json in output_directory."""
    convert_all(output_directory)
