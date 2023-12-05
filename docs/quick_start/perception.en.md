# Perception Evaluation

## Preparation

1. Copy sample scenario

   ```shell
   mkdir -p ~/driving_log_replayer_data/perception/sample
   cp -r ~/autoware/src/simulator/driving_log_replayer/sample/perception/scenario.yaml ~/driving_log_replayer_data/perception/sample
   ```

2. Copy bag file from dataset

   ```shell
   mkdir -p ~/driving_log_replayer_data/perception/sample/t4_dataset
   cp -r ~/driving_log_replayer_data/sample_dataset ~/driving_log_replayer_data/perception/sample/t4_dataset
   ```

3. Transform machine learning trained models

   ```shell
   source ~/autoware/install/setup
   ros2 launch autoware_launch logging_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
   # Wait until the following file is created in ~/autoware/install/lidar_centerpoint/share/lidar_centerpoint/data
   # - pts_backbone_neck_head_centerpoint_tiny.engine
   # - pts_voxel_encoder_centerpoint_tiny.engine
   # When the file is output, press Ctrl+C to stop launch.
   ```

## How to run

1. Run the simulation

   ```shell
   dlr simulation run -p perception --rate 0.5
   ```

   ![perception](images/perception.png)

2. Check the results

   Results are displayed in the terminal like below.
   The number of tests will vary slightly depending on PC performance and CPU load conditions, so slight differences are not a problem.

   ```shell
    test case 1 / 1 : use case: sample
    --------------------------------------------------
    TestResult: Passed
    Passed: 682 / 682 -> 100.00%
   ```
