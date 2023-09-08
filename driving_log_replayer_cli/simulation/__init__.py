import datetime
import os

import click

from driving_log_replayer_cli.core.config import load_config
from driving_log_replayer_cli.simulation.result import convert
from driving_log_replayer_cli.simulation.result import display
from driving_log_replayer_cli.simulation.run import run as sim_run

CONTEXT_SETTINGS = {"help_option_names": ["-h", "--help"]}


@click.group(context_settings=CONTEXT_SETTINGS)
def simulation() -> None:
    """Run simulation and check simulation log."""


@simulation.command(context_settings=CONTEXT_SETTINGS)
@click.option("--profile", "-p", type=str, default="default")
@click.option("--rate", "-r", default=1.0)
@click.option("--delay", "-d", default=10.0)
@click.option("--no-json", is_flag=True, help="Do not convert jsonl files to json")
def run(profile: str, rate: float, delay: float, no_json: bool) -> None:
    """Run simulation using data_directory and show result."""
    config = load_config(profile)
    output_dir_by_time = os.path.join(
        config.output_directory, datetime.datetime.now().strftime("%Y-%m%d-%H%M%S")
    )
    print(output_dir_by_time)
    sim_run(
        config.data_directory, output_dir_by_time, config.autoware_path, rate, delay, not no_json
    )


@simulation.command(context_settings=CONTEXT_SETTINGS)
@click.argument("output_directory", type=str)
def show_result(output_directory: str) -> None:
    """Show summary of simulation results in output_directory."""
    display(output_directory)


@simulation.command(context_settings=CONTEXT_SETTINGS)
@click.argument("output_directory", type=str)
def convert_result(output_directory: str) -> None:
    """Convert result.jsonl to result.json in output_directory."""
    convert(output_directory)
