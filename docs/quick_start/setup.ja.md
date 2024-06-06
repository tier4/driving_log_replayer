# 設定

!!! note

    Driving Log Replayerを実行するには、Autowareのビルドとインストールに加えて、[Driving Log Replayerのインストール](installation.md) が完了している必要があります。

    Sample map: Copyright 2020 TIER IV, Inc.

    Sample Dataset: Copyright 2022 TIER IV, Inc.

## リソースのセットアップ

1. 地図のダウンロードと解凍

   ```shell
   mkdir -p ~/autoware_map
   gdown -O ~/autoware_map/sample-map-planning.zip 'https://docs.google.com/uc?export=download&id=1499_nsbUbIeturZaDj7jhUownh5fvXHd'
   unzip -d ~/autoware_map ~/autoware_map/sample-map-planning.zip
   ```

   ブラウザから手動で[地図](https://drive.google.com/file/d/1499_nsbUbIeturZaDj7jhUownh5fvXHd/view)をダウンロードすることも可能です。

2. データセットのダウロードと解凍

   ```shell
   mkdir -p ~/driving_log_replayer_data
   gdown -O ~/driving_log_replayer_data/sample_dataset_v2.tar.zst 'https://docs.google.com/uc?export=download&id=1iCoykBBETI_rGfKEFYYb7LFydF-RJVkC'
   tar -I zstd -xvf ~/driving_log_replayer_data/sample_dataset_v2.tar.zst -C ~/driving_log_replayer_data/
   ```

   ブラウザから手動で[データセット](https://drive.google.com/file/d/1iCoykBBETI_rGfKEFYYb7LFydF-RJVkC/view)をダウンロードすることも可能です。

3. サンプル設定をコピーする

   ```shell
   # assuming that autoware is placed under ~/autoware directory
   cp ~/autoware/src/simulator/driving_log_replayer/sample/.driving_log_replayer.config.toml ~/
   ```
