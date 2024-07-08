# YabLoc Evaluation

## Preparation

1. Copy sample scenario

   ```shell
   mkdir -p ~/log_evaluator_data/yabloc/sample
   cp -r ~/autoware/src/simulator/log_evaluator/sample/yabloc/scenario.yaml ~/log_evaluator_data/yabloc/sample
   ```

2. Download bag file

   ```shell
   cp -r ~/log_evaluator_data/sample_bag/yabloc/input_bag ~/log_evaluator_data/yabloc/sample
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
