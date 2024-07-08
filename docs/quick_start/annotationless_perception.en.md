# Annotationless Perception

## Preparation

1. Copy sample scenario

   ```shell
   mkdir -p ~/log_evaluator_data/annotationless_perception/sample
   cp -r ~/autoware/src/simulator/log_evaluator/sample/annotationless_perception/scenario.yaml ~/log_evaluator_data/annotationless_perception/sample
   ```

2. Copy bag file from dataset

   ```shell
   cp -r ~/log_evaluator_data/sample_dataset/input_bag ~/log_evaluator_data/annotationless_perception/sample
   ```

## How to run

1. Run the simulation

   ```shell
   dlr simulation run -p annotationless_perception -l play_rate:=0.5
   ```

2. Check the results

   Results are displayed in the terminal like below.

   ```shell
   scenario: sample
   --------------------------------------------------
   TestResult: Passed
   Passed:
   CAR (Success)
   BUS (Success)
   PEDESTRIAN (Success)
   BICYCLE (Success)
   MOTORCYCLE (Success)
   TRAILER (Success)
   UNKNOWN (Success)
   TRUCK (Success)
   ```
