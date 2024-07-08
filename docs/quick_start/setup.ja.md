# 設定

!!! note

    log_evaluatorを実行するには、Autowareのビルドとインストールに加えて、[log_evaluatorのインストール](installation.md) が完了している必要があります。

    Sample map: Copyright 2020 TIER IV, Inc.

    Sample Dataset: Copyright 2022 TIER IV, Inc.

## リソースのセットアップ

1. データセットと地図のセットアップ(annotationless_perception, localization, obstacle_segmentation, perception)

   ```shell
   mkdir -p ~/dlr_data
   gdown -O ~/dlr_data/sample_dataset_v2.tar.zst 'https://docs.google.com/uc?export=download&id=1iCoykBBETI_rGfKEFYYb7LFydF-RJVkC'
   tar -I zstd -xvf ~/dlr_data/sample_dataset_v2.tar.zst -C ~/dlr_data/
   gdown -O ~/dlr_data/sample-map-planning.zip 'https://docs.google.com/uc?export=download&id=1499_nsbUbIeturZaDj7jhUownh5fvXHd'
   unzip -d ~/dlr_data/ ~/dlr_data/sample-map-planning.zip
   mv ~/dlr_data/sample-map-planning ~/dlr_data/sample_dataset/map
   ```

   ブラウザから手動でダウンロードすることも可能です。
   [データセット](https://drive.google.com/file/d/1iCoykBBETI_rGfKEFYYb7LFydF-RJVkC/view)
   [sample-map-planning](https://drive.google.com/file/d/1499_nsbUbIeturZaDj7jhUownh5fvXHd/view)

2. データセットと地図のセットアップ(yabloc, eagleye, ar_tag_based_localizer)

   ```shell
   gdown -O ~/dlr_data/sample_bag.tar.zst 'https://docs.google.com/uc?export=download&id=17ppdMKi4IC8J_2-_9nyYv-LAfW0M1re5'
   tar -I zstd -xvf ~/dlr_data/sample_bag.tar.zst -C ~/dlr_data/
   cp -r ~/dlr_data/sample_bag/ar_tag_based_localizer/map ~/dlr_data/sample_bag/eagleye/
   cp -r ~/dlr_data/sample_bag/ar_tag_based_localizer/map ~/dlr_data/sample_bag/yabloc/
   ```

   ブラウザから手動でダウンロードすることも可能です。
   [bag](https://drive.google.com/file/d/17ppdMKi4IC8J_2-_9nyYv-LAfW0M1re5/view)
