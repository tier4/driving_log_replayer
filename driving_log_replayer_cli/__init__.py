import importlib.metadata

import click
from driving_log_replayer_cli.configuration import configuration
from driving_log_replayer_cli.simulation import simulation

__version__ = importlib.metadata.version("driving_log_replayer_cli")

CONTEXT_SETTINGS = {"help_option_names": ["-h", "--help"]}


def main():
    cmd.add_command(configuration)
    cmd.add_command(simulation)
    cmd()


def print_version(ctx, param, value):
    if not value or ctx.resilient_parsing:
        return
    click.echo(__version__)
    ctx.exit()


@click.group(context_settings=CONTEXT_SETTINGS)
@click.option(
    "--version", "-v", is_flag=True, callback=print_version, expose_value=False, is_eager=True
)
def cmd():
    """Command line tool to use driving_log_replayer.

    https://github.com/tier4/driving_log_replayer
    """
    pass
