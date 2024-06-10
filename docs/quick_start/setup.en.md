# Setup

!!! note

    Running the Driving Log Replayer requires some additional steps on top of building and installing Autoware, so make sure that [Driving Log Replayer installation](installation.md) has been completed first before proceeding.

    Sample map: Copyright 2020 TIER IV, Inc.

    Sample Dataset: Copyright 2022 TIER IV, Inc.

## Set up resources

1. Download and unpack sample maps.

   ```shell
   # annotationless_perception, localization, obstacle_segmentation, perception
   mkdir -p ~/autoware_map
   gdown -O ~/autoware_map/sample-map-planning.zip 'https://docs.google.com/uc?export=download&id=1499_nsbUbIeturZaDj7jhUownh5fvXHd'
   unzip -d ~/autoware_map ~/autoware_map/sample-map-planning.zip

   # yabloc, eagleye
   wget -O ~/autoware_map/nishishinjuku_autoware_map.zip https://github.com/tier4/AWSIM/releases/download/v1.1.0/nishishinjuku_autoware_map.zip
   unzip -d ~/autoware_map ~/autoware_map/nishishinjuku_autoware_map.zip
   ```

   You can also download manually.
   [sample-map-planning](https://drive.google.com/file/d/1499_nsbUbIeturZaDj7jhUownh5fvXHd/view)
   [nishishinjuku_autoware_map](https://github.com/tier4/AWSIM/releases/tag/v1.1.0)

2. Download and unpack datasets.

   ```shell
   # annotationless_perception, localization, obstacle_segmentation, perception
   mkdir -p ~/driving_log_replayer_data
   gdown -O ~/driving_log_replayer_data/sample_dataset_v2.tar.zst 'https://docs.google.com/uc?export=download&id=1iCoykBBETI_rGfKEFYYb7LFydF-RJVkC'
   tar -I zstd -xvf ~/driving_log_replayer_data/sample_dataset_v2.tar.zst -C ~/driving_log_replayer_data/

   # yabloc, eagleye, artag
   gdown -O ~/driving_log_replayer_data/sample_bag.tar.zst 'https://docs.google.com/uc?export=download&id=17ppdMKi4IC8J_2-_9nyYv-LAfW0M1re5'
   tar -I zstd -xvf ~/driving_log_replayer_data/sample_bag.tar.zst -C ~/driving_log_replayer_data/
   ```

   You can also download manually.
   [dataset](https://drive.google.com/file/d/1iCoykBBETI_rGfKEFYYb7LFydF-RJVkC/view)
   [bag](https://drive.google.com/file/d/17ppdMKi4IC8J_2-_9nyYv-LAfW0M1re5/view)

3. Copy sample setting

   ```shell
   # assuming that autoware is placed under ~/autoware directory
   cp ~/autoware/src/simulator/driving_log_replayer/sample/.driving_log_replayer.config.toml ~/
   ```
