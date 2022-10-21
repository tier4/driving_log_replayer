# Driving Log Replayer Result File Format

This section describes the format of the result file output by driving_log_replayer.

It is in jsonl format, with each line containing a string in json format.

## Format

Each line is output in the following format.
The actual output is a single line of text, but it is formatted for ease of reading.

```json
{
  "Result": {
    "Success": "true or false",
    "Summary": "summary of result"
  },
  "Stamp": {
    "System": "system time",
    "ROS": "simulation time"
  },
  "Frame": {
    "Ego": { "TransformStamped": "transform_stamped from map to base_link" },
    "Different configurations for each use case": "..."
  }
}
```

- Result: Result of the evaluation of the executed scenario
- Stamp: The time of the evaluation
- Frame: Evaluation results for one received frame (topic) and attached information such as values used for judgment.
  - For more information on Frame, see the evaluation results file format for each use case.

## Output json file

jsonl is used because it can append lines and is easy to handle programmatically.
However, when a person visually checks the result, it is easier to understand if it is indented in json format.
Therefore, when simulations are run using driving_log_replayer_cli, the json file is also output by default.

On the other hand, if the simulation is run locally with wasim or in the cloud, only a jsonl file is created.
If you want to convert a jsonl file to json, you can do so with the following command.

```shell
# Conversion of result files, converting result.jsonl under output_directory to result.json
driving_log_replayer simulation convert-result ${output_directory}
```

## Analyze the result files

As mentioned in the json output section, json files are output only when driving_log_replayer_cli is used locally.
Therefore, when analyzing the result files, such as graphing, jsonl should be used as the analysis target.

If you use python, [you can use pandas to read jsonl](https://qiita.com/meshidenn/items/3ff72396fe85044bc74f).
