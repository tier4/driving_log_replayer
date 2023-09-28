# YabLoc Evaluation

## Preparation

1. Copy sample scenario

   ```bash
   mkdir -p ~/driving_log_replayer_data/yabloc/sample
   cp -r ~/autoware/src/simulator/driving_log_replayer/sample/yabloc/scenario.yaml ~/driving_log_replayer_data/yabloc/sample
   ```

2. Download bag file
   Download a rosbag from [Google Drive Link](https://drive.google.com/file/d/1zKTGRH4lD-wptpOdNCgpiPfGRDP0XrUm/view).
   After that, execute following commands:

   ```bash
   unzstd yabloc_autoware_test_made_in_awsim_0.db3.zst
   mkdir input_bag
   mv yabloc_autoware_test_made_in_awsim_0.db3 input_bag
   ros2 bag reindex input_bag -s sqlite3
   mv input_bag ~/driving_log_replayer_data/yabloc/sample
   ```

3. Filter and slice bag

   ```bash
   source ~/autoware/install/setup.bash
   cd ~/driving_log_replayer_data/yabloc/sample
   ros2 bag filter input_bag -o filtered_bag -x "/clock"
   ros2 bag slice filtered_bag -o sliced_bag -e 580
   rm -rf input_bag
   rm -rf filtered_bag
   mv sliced_bag input_bag
   ```

## How to run

1. Run the simulation

   ```bash
   driving_log_replayer simulation run -p yabloc --rate 0.5
   ```

2. Check the results

   Results are displayed in the terminal like below.

   ```bash
   test case 1 / 1 : use case: sample
   --------------------------------------------------
   TestResult: Passed
   Passed: YabLoc Availability (Passed): OK
   ```
