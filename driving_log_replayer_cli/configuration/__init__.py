from pathlib import Path

import click

from driving_log_replayer_cli.core.config import Config
from driving_log_replayer_cli.core.config import save_config

CONTEXT_SETTINGS = {"help_option_names": ["-h", "--help"]}


@click.group(context_settings=CONTEXT_SETTINGS)
def configuration() -> None:
    """Command to setup driving_log_replayer config file."""


@configuration.command(context_settings=CONTEXT_SETTINGS)
@click.option(
    "--data_directory",
    "-d",
    required=True,
    type=click.Path(exists=True, file_okay=False),
)
@click.option(
    "--output_directory",
    "-o",
    required=True,
    type=click.Path(exists=False, file_okay=False),
)
@click.option("--autoware_path", "-a", required=True, type=click.Path(exists=True, file_okay=False))
@click.option(
    "--profile_name",
    "-p",
    type=str,
    default="default",
    help="profile name in config file default value is default",
)
def register(
    data_directory: Path,
    output_directory: Path,
    autoware_path: Path,
    profile_name: str,
) -> None:
    """Create profile in driving_log_replayer config file."""
    config = Config(
        data_directory=data_directory,
        output_directory=output_directory,
        autoware_path=autoware_path,
    )
    save_config(config, profile_name)
