# はじめに

Driving Log Replayer は、log(rosbag2)を用いて Autoware の open loop simulation を実行し、その結果を評価するパッケージである。
Sensing, Localization, Perception の性能確認と、ソフトウェアのリグレッションテストに使用する。

## アーキテクチャ

図に示すように、Autoware の標準機能に Driving Log Replayer 固有のシミュレーションの実行機能と評価機能を付加した構成となっている。

![architecture](images/architecture.png)

## 関連ドキュメント

1. [AutowareDocumentation](https://autowarefoundation.github.io/autoware-documentation/main/)
2. [WebAutoDocumentation](https://docs.web.auto/)

## 関連リポジトリ

1. [ros2bag_extensions](https://github.com/tier4/ros2bag_extensions)
2. [perception_eval](https://github.com/tier4/autoware_perception_evaluation)
