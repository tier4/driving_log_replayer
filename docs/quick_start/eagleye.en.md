# Eagleye Evaluation

## Preparation

1. Copy sample scenario

   ```bash
   mkdir -p ~/log_evaluator_data/eagleye/sample
   cp -r ~/autoware/src/simulator/log_evaluator/sample/eagleye/scenario.yaml ~/log_evaluator_data/eagleye/sample
   ```

2. Copy sample bag

   ```shell
   cp -r ~/log_evaluator_data/sample_bag/eagleye/input_bag ~/log_evaluator_data/eagleye/sample
   ```

## How to run

1. Run the simulation

   ```bash
   dlr simulation run -p eagleye  -l play_rate:=0.5
   ```

2. Check the results

   Results are displayed in the terminal like below.

   ```bash
   test case 1 / 1 : use case: sample
   --------------------------------------------------
   TestResult: Passed
   Passed: Eagleye Availability (Passed): OK
   ```
