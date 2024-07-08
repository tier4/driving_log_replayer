# Obstacle Segmentation Evaluation

## Preparation

1. Copy sample scenario

   ```shell
   mkdir -p ~/log_evaluator_data/obstacle_segmentation/sample
   cp -r ~/autoware/src/simulator/log_evaluator/sample/obstacle_segmentation/scenario.yaml ~/log_evaluator_data/obstacle_segmentation/sample
   ```

2. Copy bag file from dataset

   ```shell
   mkdir -p ~/log_evaluator_data/obstacle_segmentation/sample/t4_dataset
   cp -r ~/log_evaluator_data/sample_dataset ~/log_evaluator_data/obstacle_segmentation/sample/t4_dataset
   ```

## How to run

1. Run the simulation

   ```shell
   dlr simulation run -p obstacle_segmentation  -l play_rate:=0.5
   ```

   ![obstacle_segmentation](images/obstacle_segmentation.png)

2. Check the results

   Results are displayed in the terminal like below.
   The number of tests will vary slightly depending on PC performance and CPU load conditions, so slight differences are not a problem.

   ```shell
    test case 1 / 1 : use case: sample_dataset
    --------------------------------------------------
    TestResult: Failed
    Detection Failed: detection: 557 / 681 -> 81.79% detection_warn: 0 non_detection: 681 / 681 -> 100.00%
   ```
