# パッケージ概要

Driving Log Replayer は、評価の条件が記載されたシナリオをパッケージが読み取り、autoware を起動し、評価結果を jsonl ファイル形式で出力するという動作を行うパッケージになっている。
概要を図に示す。

![overview](images/overview.drawio.svg)

## 開発環境

autoware.universe が動作する環境で以下の通りとする。

- Ubuntu 20.04 / 22.04
- ROS galactic / humble
- Python 3.8 / 3.10

## 実行環境

開発環境と同じ。

## 利用フロー

1. 評価用の bag を実車で取得する
2. 取得した bag を必要な時間、topic だけ残るようにフィルタする
   1. ROS2 の場合は galactic でも未だに filter コマンドが実装されていないので tier4 で開発した ros2bag_extensions を使用する
   2. 収録時に autoware が出力したトピックを落としてセンサートピックだけを残す
   3. 走行前や走行後の評価に不要な時間をカットする(ただし、初期位置位置合わせに車両が静止している時間が 3 秒以上必要なので走行開始前の 10 秒程度は残しておく)
3. シナリオを作成する
   1. sample フォルダ内にシナリオの例あり
   2. 記述方法はフォーマット定義を参照
4. ユースケースが obstacle_segmentation, perception の場合、t4_dataset への変換に対応したアノテーションツールでアノテーションを実施する。
   1. t4_dataset 変換ツールは公開準備中
5. 評価を実行する。
