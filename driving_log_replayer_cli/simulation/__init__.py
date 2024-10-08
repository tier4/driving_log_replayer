from pathlib import Path

import click

from driving_log_replayer_cli.core.config import Config
from driving_log_replayer_cli.core.config import load_config
from driving_log_replayer_cli.core.result import display_all
from driving_log_replayer_cli.simulation.run import dry_run as sim_dry_run
from driving_log_replayer_cli.simulation.run import run as sim_run
from driving_log_replayer_cli.simulation.update import update_annotationless_scenario_condition

CONTEXT_SETTINGS = {"help_option_names": ["-h", "--help"]}


@click.group(context_settings=CONTEXT_SETTINGS)
def simulation() -> None:
    """Run simulation and check simulation log."""


@simulation.command(context_settings=CONTEXT_SETTINGS)
@click.option("--profile", "-p", type=str, default="default")
@click.option("--launch_args", "-l", multiple=True, default=[])
@click.option(
    "--update_scenario",
    "-u",
    type=click.Choice(["keep", "existing", "all"]),
    default="keep",
    help="Automatically update scenario conditions after simulation runs. keep do nothing. Valid only with annotationless_perception. Otherwise this option is ignored.",
)
def run(
    profile: str,
    launch_args: list[str],
    update_scenario: str,
) -> None:
    config: Config = load_config(profile)
    sim_run(
        config,
        launch_args,
        update_scenario,
    )


@simulation.command(context_settings=CONTEXT_SETTINGS)
@click.option("--profile", "-p", type=str, default="default")
@click.option(
    "--use-case",
    "-u",
    type=click.Choice(["annotationless_perception"]),
    default="annotationless_perception",
)
@click.option("--launch_args", "-l", multiple=True, default=[])
def dry_run(
    profile: str,
    use_case: str,
    launch_args: list[str],
) -> None:
    config: Config = load_config(profile)
    sim_dry_run(
        config,
        use_case,
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
@click.option(
    "--scenario",
    "-s",
    required=True,
    type=click.Path(exists=True, resolve_path=True, path_type=Path),
    help="Path to the scenario YAML file",
)
@click.option(
    "--result",
    "-r",
    required=True,
    type=click.Path(exists=True, resolve_path=True, path_type=Path),
    help="Path to the result JSONL file",
)
@click.option(
    "--update_method",
    "-u",
    type=click.Choice(["existing", "all"]),
    default="existing",
    help="choice update method. EXISTING updates only the keys listed in the scenario. all updates all keys",
)
def update_condition(scenario: Path, result: Path, update_method: str) -> None:
    """Update annotationless_perception scenario file conditions with data from JSONL results."""
    update_annotationless_scenario_condition(scenario, result, update_method)
