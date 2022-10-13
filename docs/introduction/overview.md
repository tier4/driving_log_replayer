# パッケージ概要

Driving Log Replayerは、評価の条件が記載されたシナリオをパッケージが読み取り、autowareを起動し、評価結果をjsonlファイル形式で出力するという動作を行うパッケージになっている。
概要を図に示す。

![overview](images/overview.drawio.svg)

## 開発環境

autoware.universeが動作する環境で以下の通りとする。

- Ubuntu 20.04 / 22.04
- ROS galactic / humble
- Python 3.8 / 3.10

## 実行環境

開発環境と同じ。driving_log_replayerは、ROSのパッケージとして提供され、autowareのリポジトリでvcs importすることで、src/simulator以下にインストールされる。
