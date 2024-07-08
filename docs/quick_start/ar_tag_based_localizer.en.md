# ArTagBasedLocalizer Evaluation

## Preparation

1. Copy sample scenario

   ```shell
   mkdir -p ~/log_evaluator_data/ar_tag_based_localizer/sample
   cp -r ~/autoware/src/simulator/log_evaluator/sample/ar_tag_based_localizer/scenario.yaml ~/log_evaluator_data/ar_tag_based_localizer/sample
   ```

2. Copy sample bag and map

   ```shell
   cp -r ~/log_evaluator_data/sample_bag/ar_tag_based_localizer/input_bag ~/log_evaluator_data/ar_tag_based_localizer/sample
   cp -r ~/log_evaluator_data/sample_bag/ar_tag_based_localizer/map ~/log_evaluator_data/ar_tag_based_localizer/sample
   ```

## How to run

1. Run the simulation

   ```shell
   dlr simulation run -p ar_tag_based_localizer -l play_rate:=0.5
   ```

2. Check the results

   Results are displayed in the terminal like below.

   ```shell
   test case 1 / 1 : use case: sample
   --------------------------------------------------
   TestResult: Passed
   Passed: ArTagBasedLocalizer Availability (Success): Detected 1 AR tags
   ```
