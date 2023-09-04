# Eagleye Evaluation

## Preparation

1. Copy sample scenario

   ```bash
   mkdir -p ~/driving_log_replayer_data/eagleye/sample
   cp -r ~/autoware/src/simulator/driving_log_replayer/sample/eagleye/scenario.yaml ~/driving_log_replayer_data/eagleye/sample
   ```

2. Download map files

   Access [AWSIM Quick Start Demo page](https://tier4.github.io/AWSIM/GettingStarted/QuickStartDemo/) and download map files (.pcd and .osm) in "Launching Autoware" section.
   Store it in `$HOME/autoware_map/awsim-shinjuku` as defined in the sample scenario.yaml.

3. Download bag file
   Download a rosbag from [Google Drive Link](https://drive.google.com/file/d/1Zgv9eP0j2hAgTj7pW8n-YaECPQGGQjO2/view).
   After that, execute following commands:

   ```bash
   unzip eagleye_awsim_test_bag.zip
   ```

## How to run

1. Run the simulation

   ```bash
   driving_log_replayer simulation run -p eagleye --rate 0.5
   ```

2. Check the results

   Results are displayed in the terminal like below.

   ```bash
   test case 1 / 1 : use case: sample
   --------------------------------------------------
   TestResult: Passed
   Passed: Eagleye Availability (Passed): OK
   ```
