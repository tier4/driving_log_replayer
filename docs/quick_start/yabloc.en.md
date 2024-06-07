# YabLoc Evaluation

## Preparation

1. Copy sample scenario

   ```shell
   mkdir -p ~/driving_log_replayer_data/yabloc/sample
   cp -r ~/autoware/src/simulator/driving_log_replayer/sample/yabloc/scenario.yaml ~/driving_log_replayer_data/yabloc/sample
   ```

2. Download bag file

   ```shell
   cp -r ~/driving_log_replayer_data/sample_bag/yabloc/input_bag ~/driving_log_replayer_data/yabloc/sample
   ```

## How to run

1. Run the simulation

   ```shell
   dlr simulation run -p yabloc -l play_rate:=0.5
   ```

2. Check the results

   Results are displayed in the terminal like below.

   ```shell
   test case 1 / 1 : use case: sample
   --------------------------------------------------
   TestResult: Passed
   Passed: YabLoc Availability (Passed): OK
   ```
