# はじめに

Driving Log Replayer は、log(rosbag2)を用いて Autoware の open loop simulation を実行し、Autoware が出力するトピックを評価するパッケージです。
Sensing, Localization, Perception の性能確認と、ソフトウェアのリグレッションテストに使用します。

## アーキテクチャ

Driving Log Replayer は、Autowareの評価ノードをAutowareの標準機能に付加した構成となっている。
アーキテクチャ図を以下に示す。

![architecture](images/architecture.png)

## 関連ドキュメント

1. [AutowareDocumentation](https://autowarefoundation.github.io/autoware-documentation/main/)
2. [WebAutoDocumentation](https://docs.web.auto/)

## 関連リポジトリ

1. [ros2bag_extensions](https://github.com/tier4/ros2bag_extensions)
2. [perception_eval](https://github.com/tier4/autoware_perception_evaluation)
