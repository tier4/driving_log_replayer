# 設定

!!! note

    Driving Log Replayerを実行するには、Autowareのビルドとインストールに加えて、[インストール](installation.md) が完了している必要があります。

    Sample map: Copyright 2020 TIER IV, Inc.

    Sample Dataset: Copyright 2022 TIER IV, Inc.

## リソースのセットアップ

1. 地図のダウンロードと解凍

   planning-simulation で使用する地図と同じファイルを使用します。既にダウンロード済みの場合はこのステップは不要です。

   ```bash
   mkdir -p ~/autoware_map
   gdown -O ~/autoware_map/sample-map-planning.zip 'https://docs.google.com/uc?export=download&id=1499_nsbUbIeturZaDj7jhUownh5fvXHd'
   unzip -d ~/autoware_map ~/autoware_map/sample-map-planning.zip
   ```

   You can also download [the map](https://drive.google.com/file/d/1499_nsbUbIeturZaDj7jhUownh5fvXHd/view) manually.

2. データセットのダウロードと解凍

   The driving_log_replayer tutorial uses the same dataset, so this step is not necessary if you have already downloaded it from other evaluation.

   ```bash
   mkdir -p ~/driving_log_replayer_data
   gdown -O ~/driving_log_replayer_data/sample_dataset.tar.zst 'https://docs.google.com/uc?export=download&id=1UjMWZj5Yc55O7BZiGHa0ikZGhwmcfPiS'
   tar -I zstd -xvf ~/driving_log_replayer_data/sample_dataset.tar.zst -C ~/driving_log_replayer_data/
   ```

   You can also download [the dataset](https://drive.google.com/file/d/1UjMWZj5Yc55O7BZiGHa0ikZGhwmcfPiS/view) manually.

3. Copy sample setting

   The driving_log_replayer tutorial uses the same setting, so this step is not necessary if you have already copied it from other evaluation.

   ```bash
   # assuming that autoware is placed under ~/autoware directory
   cp ~/autoware/src/simulator/driving_log_replayer/sample/.driving_log_replayer.config.toml ~/
   ```
