# 設定

!!! note

    Driving Log Replayerを実行するには、Autowareのビルドとインストールに加えて、[Driving Log Replayerのインストール](installation.md) が完了している必要があります。

    Sample map: Copyright 2020 TIER IV, Inc.

    Sample Dataset: Copyright 2022 TIER IV, Inc.

## リソースのセットアップ

1. 地図のダウンロードと解凍

   ```shell
   # annotationless_perception, localization, obstacle_segmentation, perception
   mkdir -p ~/autoware_map
   gdown -O ~/autoware_map/sample-map-planning.zip 'https://docs.google.com/uc?export=download&id=1499_nsbUbIeturZaDj7jhUownh5fvXHd'
   unzip -d ~/autoware_map ~/autoware_map/sample-map-planning.zip

   # yabloc, eagleye
   wget -O ~/autoware_map/nishishinjuku_autoware_map.zip https://github.com/tier4/AWSIM/releases/download/v1.1.0/nishishinjuku_autoware_map.zip
   unzip -d ~/autoware_map ~/autoware_map/nishishinjuku_autoware_map.zip
   ```

   ブラウザから手動でダウンロードすることも可能です。
   [sample-map-planning](https://drive.google.com/file/d/1499_nsbUbIeturZaDj7jhUownh5fvXHd/view)
   [nishishinjuku_autoware_map](https://github.com/tier4/AWSIM/releases/tag/v1.1.0)

2. データセットのダウロードと解凍

   ```shell
   # annotationless_perception, localization, obstacle_segmentation, perception
   mkdir -p ~/driving_log_replayer_data
   gdown -O ~/driving_log_replayer_data/sample_dataset_v2.tar.zst 'https://docs.google.com/uc?export=download&id=1iCoykBBETI_rGfKEFYYb7LFydF-RJVkC'
   tar -I zstd -xvf ~/driving_log_replayer_data/sample_dataset_v2.tar.zst -C ~/driving_log_replayer_data/

   # yabloc, eagleye, artag
   gdown -O ~/driving_log_replayer_data/sample_bag.tar.zst 'https://docs.google.com/uc?export=download&id=17ppdMKi4IC8J_2-_9nyYv-LAfW0M1re5'
   tar -I zstd -xvf ~/driving_log_replayer_data/sample_bag.tar.zst -C ~/driving_log_replayer_data/
   ```

   ブラウザから手動でダウンロードすることも可能です。
   [データセット](https://drive.google.com/file/d/1iCoykBBETI_rGfKEFYYb7LFydF-RJVkC/view)
   [bag](https://drive.google.com/file/d/17ppdMKi4IC8J_2-_9nyYv-LAfW0M1re5/view)

3. サンプル設定をコピーする

   ```shell
   # assuming that autoware is placed under ~/autoware directory
   cp ~/autoware/src/simulator/driving_log_replayer/sample/.driving_log_replayer.config.toml ~/
   ```
