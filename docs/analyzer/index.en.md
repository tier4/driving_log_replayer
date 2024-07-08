# Driving Log Replayer Analyzer

Package to analyze the result files of tests performed by Driving Log Replayer.

## Directory Structure

The directory structure is as follows.

```shell
log_evaluator_analyzer
├── __init__.py
├── __main__.py    # Entry point for CLI
├── analysis       # CLI analysis command
├── config         # Configuration file and module to read configuration
├── data           # Module to read data from jsonl
└── plot           # Module to plot data
```

Although this package is ROS-independent, it is also imported into ROS nodes as a library, so it is also installed as a ROS package.

The roles of the modules are shown in Fig.

[architecture](./images/architecture.drawio.svg)

## Caution

Currently only analysis of result.jsonl of obstacle_segmentation is possible.
If necessary, add analysis modules for each use case.
Add use_case_name.py files to analysis, config, and data.

## How to install

- Installed with log_evaluator_cli
- Installed with log_evaluator as a ros package

## Usage

```shell
dlr-analyzer analysis ${use-case-name} ${result.jsonl_path} [-c ${config_path}]
```
