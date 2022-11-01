# Driving Log Replayer Result File Format

It is in JSONL format, with each line containing a string in JSON format.

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

Each evaluation output consists of the following attributes:

- Result: Result of the evaluation of the executed scenario
- Stamp: The time of the evaluation
- Frame: Evaluation results for one received frame (topic) and attached information such as values used for judgment.

For more information on Frame, see the evaluation results file format for each use case.

## Output JSON file

JSONL is used because it is appendable, and one result can be appended per topic subscription.
However, it is easier to read a JSON file format when investigating the files manually.
Therefore, when simulations are run using driving_log_replayer_cli, the JSON file is also output by default.

On the other hand, if the simulation is run locally with [wasim](https://docs.web.auto/en/developers-guides/wasim/use-cases/run-simulations-locally/) or in [the cloud](https://docs.web.auto/en/user-manuals/evaluator/introduction), only a JSONL file is created.
If you want to convert a JSONL file to JSON, you can do so with the following command.

```shell
# Conversion of result files, converting result.jsonl under output_directory to result.json
driving_log_replayer simulation convert-result ${output_directory}
```

## Analyze the result files

As mentioned in the json output section, json files are output only when driving_log_replayer_cli is used locally.
Therefore, when analyzing the result files, such as graphing, jsonl should be used as the analysis target.

If you use python, you can read jsonl [by setting lines=True in pandas.read_json](https://pandas.pydata.org/docs/reference/api/pandas.read_json.html).
