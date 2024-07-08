# Setup

!!! note

    Running the log_evaluator requires some additional steps on top of building and installing Autoware, so make sure that [log_evaluator installation](installation.md) has been completed first before proceeding.

    Sample map: Copyright 2020 TIER IV, Inc.

    Sample Dataset: Copyright 2022 TIER IV, Inc.

## Set up resources

1. Dataset and Map Setup (annotationless_perception, localization, obstacle_segmentation, perception)

   ```shell
   mkdir -p ~/log_evaluator
   gdown -O ~/log_evaluator/sample_dataset_v2.tar.zst 'https://docs.google.com/uc?export=download&id=1iCoykBBETI_rGfKEFYYb7LFydF-RJVkC'
   tar -I zstd -xvf ~/log_evaluator/sample_dataset_v2.tar.zst -C ~/log_evaluator/
   gdown -O ~/log_evaluator/sample-map-planning.zip 'https://docs.google.com/uc?export=download&id=1499_nsbUbIeturZaDj7jhUownh5fvXHd'
   unzip -d ~/log_evaluator/ ~/log_evaluator/sample-map-planning.zip
   mv ~/log_evaluator/sample-map-planning ~/log_evaluator/sample_dataset/map
   ```

   You can also download manually.
   [データセット](https://drive.google.com/file/d/1iCoykBBETI_rGfKEFYYb7LFydF-RJVkC/view)
   [sample-map-planning](https://drive.google.com/file/d/1499_nsbUbIeturZaDj7jhUownh5fvXHd/view)

2. Dataset and Map Setup(yabloc, eagleye, ar_tag_based_localizer)

   ```shell
   gdown -O ~/log_evaluator/sample_bag.tar.zst 'https://docs.google.com/uc?export=download&id=17ppdMKi4IC8J_2-_9nyYv-LAfW0M1re5'
   tar -I zstd -xvf ~/log_evaluator/sample_bag.tar.zst -C ~/log_evaluator/
   mv ~/log_evaluator/sample_bag/*  ~/log_evaluator/
   rmdir ~/log_evaluator/sample_bag
   cp -r ~/log_evaluator/ar_tag_based_localizer/map ~/log_evaluator/eagleye/
   cp -r ~/log_evaluator/ar_tag_based_localizer/map ~/log_evaluator/yabloc/
   ```

   You can also download manually.
   [bag](https://drive.google.com/file/d/17ppdMKi4IC8J_2-_9nyYv-LAfW0M1re5/view)

3. Copy the sample scenario to the dataset directory

   ```shell
   # Specify the directory where autoware is installed. Change according to your environment.
   AUTOWARE_PATH=$HOME/ros_ws/awf
   # SAMPLE_ROOT=${AUTOWARE_PATH}/src/simulator/log_evaluator/sample
   SAMPLE_ROOT=${AUTOWARE_PATH}/src/simulator/driving_log_replayer/sample # Temporary old name until repository split
   cp ${SAMPLE_ROOT}/annotationless_perception/scenario.yaml ~/log_evaluator/annotationless_perception.yaml
   cp ${SAMPLE_ROOT}/ar_tag_based_localizer/scenario.yaml ~/log_evaluator/ar_tag_based_localizer.yaml
   cp ${SAMPLE_ROOT}/eagleye/scenario.yaml ~/log_evaluator/eagleye.yaml
   cp ${SAMPLE_ROOT}/localization/scenario.yaml ~/log_evaluator/localization.yaml
   cp ${SAMPLE_ROOT}/obstacle_segmentation/scenario.yaml ~/log_evaluator/obstacle_segmentation.yaml
   cp ${SAMPLE_ROOT}/perception/scenario.yaml ~/log_evaluator/perception.yaml
   cp ${SAMPLE_ROOT}/yabloc/scenario.yaml ~/log_evaluator/yabloc.yaml
   ```

4. Transform machine learning trained models

   ```shell
   source ~/autoware/install/setup
   ros2 launch autoware_launch logging_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit
   # Wait until the following file is created in ~/autoware/install/lidar_centerpoint/share/lidar_centerpoint/data
   # - pts_backbone_neck_head_centerpoint_tiny.engine
   # - pts_voxel_encoder_centerpoint_tiny.engine
   # When the file is output, press Ctrl+C to stop launch.
   ```

