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

import os
from pathlib import Path
from typing import Optional

import click
from driving_log_replayer_analyzer.analysis.obstacle_segmentation import visualize as os_vis

CONTEXT_SETTINGS = {"help_option_names": ["-h", "--help"]}


@click.group(context_settings=CONTEXT_SETTINGS)
def analysis():
    """Run analysis of the use case."""
    pass


@analysis.command(context_settings=CONTEXT_SETTINGS)
@click.argument("input_jsonl", type=str)
@click.option("--output_dir", "-o", type=str)
@click.option("--config_yaml", "-c", type=str)
def obstacle_segmentation(input_jsonl: str, output_dir: Optional[str], config_yaml: Optional[str]):
    """Run obstacle_segmentation analysis."""
    p_input_jsonl = Path(os.path.expandvars(input_jsonl))
    if output_dir is None:
        p_output_dir = p_input_jsonl.parent
    else:
        p_output_dir = Path(os.path.expandvars(output_dir))
    if config_yaml is None:
        p_config = Path.home().joinpath("config.yaml")
    else:
        p_config = Path(os.path.expandvars(config_yaml))
    os_vis(p_input_jsonl, p_output_dir, p_config)
