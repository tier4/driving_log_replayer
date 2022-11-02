# インストール

[AWF Autoware Core/Universe](https://github.com/autowarefoundation/autoware)を`driving_log_replayer`と一緒にビルドする方法を解説します。

## Requirements

- CPU amd64
- Ubuntu 20.04 / 22.04
- ROS galactic / humble
- Python 3.8 / 3.10
- NVIDIA GPU (required if running perception)
- [pipx](https://pypa.github.io/pipx/)
- [zstd](https://github.com/facebook/zstd)
  - sudo apt install zstd

## ビルド方法

1. Autoware workspace に移動する:

   ```shell
   cd autoware
   ```

2. 依存パッケージを追加する:

   ```shell
   nano simulator.repos
   # 以下の内容を追加する
   ```

   ```shell
    simulator/perception_eval:
      type: git
      url: https://github.com/tier4/autoware_perception_evaluation.git
      version: main
    simulator/driving_log_replayer:
      type: git
      url: https://github.com/tier4/driving_log_replayer.git
      version: main
    simulator/vendor/ros2_numpy:
      type: git
      url: https://github.com/Box-Robotics/ros2_numpy.git
      # galacticで使用する場合でもバージョン指定はhumbleで良い
      version: humble
    simulator/vendor/ros2bag_extensions:
      type: git
      url: https://github.com/tier4/ros2bag_extensions.git
      version: main
   ```

3. simulatorの依存パッケージをインポートする:

   ```shell
   vcs import src < simulator.repos
   ```

4. 依存解決のためにrosdepを更新する:

   ```shell
   rosdep update
   ```

5. rosdepで依存のパッケージをインストールする:

   ```shell
   rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
   ```

6. ワークスペースをビルドする:

   ```shell
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

7. driving_log_replayer_cliをインストールする:

   ```shell
   pipx install git+https://github.com/tier4/driving_log_replayer.git
   ```
