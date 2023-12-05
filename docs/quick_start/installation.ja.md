# インストール

[AWF Autoware Core/Universe](https://github.com/autowarefoundation/autoware)を`driving_log_replayer`と一緒にビルドする方法を解説します。

## Requirements

- CPU amd64
- Ubuntu 20.04 / 22.04
- ROS humble
- Python 3.10
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

7. driving_log_replayer_cli をインストールする:

   ```shell
   pipx install git+https://github.com/tier4/driving_log_replayer.git
   ```

8. shellの補完スクリプトをインストールする:

   ```shell
   # bash
   _DLR_COMPLETE=bash_source dlr > $HOME/.dlr-complete.bash
   _DLR_COMPLETE=bash_source dlr > $HOME/.dlr-analyzer-complete.bash

   echo "source $HOME/.dlr-complete.bash" >> ~/.bashrc
   echo "source $HOME/.dlr-analyzer-complete.bash" >> ~/.bashrc
   ```

   ```shell
   # fish
   _DLR_COMPLETE=fish_source dlr > $HOME/.config/fish/completions/dlr.fish
   _DLR_ANALYZER_COMPLETE=fish_source dlr-analyzer > $HOME/.config/fish/completions/dlr-analyzer.fish
   ```
