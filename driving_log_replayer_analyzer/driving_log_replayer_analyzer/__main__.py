# Copyright (c) 2022 TierIV.inc
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from importlib.metadata import version

import click
from driving_log_replayer_analyzer.analysis import analysis

CONTEXT_SETTINGS = {"help_option_names": ["-h", "--help"]}

try:
    __version__ = version(__package__)
except Exception:
    __version__ = "0.0.0"


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


cmd.add_command(analysis)
cmd()
