# インストール

[AWF Autoware Core/Universe](https://github.com/autowarefoundation/autoware)を`log_evaluator`と一緒にビルドする方法を解説します。

## Requirements

- CPU amd64
- Ubuntu 22.04
- ROS humble
- Python 3.10
- NVIDIA GPU (required if running perception)
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
     simulator/log_evaluator:
       type: git
       url: https://github.com/tier4/log_evaluator.git
       version: main
     simulator/vendor/ros2_numpy:
       type: git
       url: https://github.com/Box-Robotics/ros2_numpy.git
       version: humble
     simulator/vendor/ros2bag_extensions:
       type: git
       url: https://github.com/tier4/ros2bag_extensions.git
       version: main
   ```

3. simulator の依存パッケージをインポートする:

   ```shell
   vcs import src < simulator.repos
   ```

4. 依存解決のために rosdep を更新する:

   ```shell
   rosdep update
   ```

5. rosdep で依存のパッケージをインストールする:

   ```shell
   rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
   ```

6. ワークスペースをビルドする:

   ```shell
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```
