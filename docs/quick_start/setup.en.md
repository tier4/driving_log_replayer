# Setup

!!! note

    Running the Driving Log Replayer requires some additional steps on top of building and installing Autoware, so make sure that [Driving Log Replayer installation](installation.md) has been completed first before proceeding.

    Sample map: Copyright 2020 TIER IV, Inc.

    Sample Dataset: Copyright 2022 TIER IV, Inc.

## Set up resources

1. Download and unpack a sample map.

   ```shell
   mkdir -p ~/autoware_map
   gdown -O ~/autoware_map/sample-map-planning.zip 'https://docs.google.com/uc?export=download&id=1499_nsbUbIeturZaDj7jhUownh5fvXHd'
   unzip -d ~/autoware_map ~/autoware_map/sample-map-planning.zip
   ```

   You can also download [the map](https://drive.google.com/file/d/1499_nsbUbIeturZaDj7jhUownh5fvXHd/view) manually.

2. Download and unpack a dataset.

   ```shell
   mkdir -p ~/driving_log_replayer_data
   gdown -O ~/driving_log_replayer_data/sample_dataset_v2.tar.zst 'https://docs.google.com/uc?export=download&id=1iCoykBBETI_rGfKEFYYb7LFydF-RJVkC'
   tar -I zstd -xvf ~/driving_log_replayer_data/sample_dataset_v2.tar.zst -C ~/driving_log_replayer_data/
   ```

   You can also download [the dataset](https://drive.google.com/file/d/1iCoykBBETI_rGfKEFYYb7LFydF-RJVkC/view) manually.

3. Copy sample setting

   ```shell
   # assuming that autoware is placed under ~/autoware directory
   cp ~/autoware/src/simulator/driving_log_replayer/sample/.driving_log_replayer.config.toml ~/
   ```
