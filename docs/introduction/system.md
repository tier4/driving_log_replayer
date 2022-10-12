# システム概要

<<img-overview>>にシステムの概要を示す。
[[img-overview]]
.システム概要
image::images/overview.drawio.svg[システム概要]

## 開発環境

autoware.universeが動作する環境で以下の通りとする。

* Ubuntu 20.04 / 22.04
* ROS galactic / humble
* Python 3.8 / 3.10

## 実行環境

開発環境と同じ。driving_log_replayerは、ROSのパッケージとして提供され、autowareのリポジトリでvcs importすることで、src/simulator以下にインストールされる。
