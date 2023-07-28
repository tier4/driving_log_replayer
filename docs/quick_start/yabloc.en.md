# YabLoc Evaluation

## Preparation

1. Copy sample scenario

   ```bash
   mkdir -p ~/driving_log_replayer_data/yabloc/sample
   cp -r ~/autoware/src/simulator/driving_log_replayer/sample/yabloc/scenario.yaml ~/driving_log_replayer_data/yabloc/sample
   ```

2. Copy bag file from dataset
   # TODO!!!!!!!!!!!!!!!!!!!!!!!!
   ```bash
   cp -r ~/driving_log_replayer_data/sample_dataset/input_bag ~/driving_log_replayer_data/yabloc/sample
   ```

3. Filter and slice bag

   ```bash
   source ~/autoware/install/setup.bash
   cd ~/driving_log_replayer_data/yabloc/sample
   rm -rf input_bag
   rm -rf filtered_bag
   mv sliced_bag input_bag
   ```

## How to run

1. Run the simulation

   ```bash
   driving_log_replayer simulation run -p yabloc --rate 0.5
   ```

   ![yabloc](images/yabloc.png)

2. Check the results

   Results are displayed in the terminal like below.
   The number of tests will vary slightly depending on PC performance and CPU load conditions, so slight differences are not a problem.

   # TODO!!!!!!!!!!!!!!