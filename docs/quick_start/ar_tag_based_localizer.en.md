# ArTagBasedLocalizer Evaluation

## Preparation

1. Copy sample scenario

   ```shell
   mkdir -p ~/driving_log_replayer_data/ar_tag_based_localizer/sample
   cp -r ~/autoware/src/simulator/driving_log_replayer/sample/ar_tag_based_localizer/scenario.yaml ~/driving_log_replayer_data/ar_tag_based_localizer/sample
   ```

2. Copy sample bag and map

   ```shell
   cp -r ~/driving_log_replayer_data/sample_bag/ar_tag_based_localizer/input_bag ~/driving_log_replayer_data/ar_tag_based_localizer/sample
   cp -r ~/driving_log_replayer_data/sample_bag/ar_tag_based_localizer/map ~/driving_log_replayer_data/ar_tag_based_localizer/sample
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
