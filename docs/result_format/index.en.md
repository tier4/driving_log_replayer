# log_evaluator Result File Format

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

## Analyze the result files

With vscode's JSONL Converter, you can easily convert jsonl <-> json with the push of a button

<https://marketplace.visualstudio.com/items?itemName=F-loat.jsonl-converter>

If you use python, you can read jsonl [by setting lines=True in pandas.read_json](https://pandas.pydata.org/docs/reference/api/pandas.read_json.html).
