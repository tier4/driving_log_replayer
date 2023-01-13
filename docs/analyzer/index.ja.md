# Driving Log Replayer Analyzer

Driving Log Replayer で行ったテストの結果ファイルを分析するパッケージ。

## 構成

以下のようなフォルダ構成を取る。

```shell
driving_log_replayer_analyzer
├── __init__.py
├── __main__.py      # CLIのエントリーポイント
├── analysis         # CLIの解析コマンド
├── config           # 設定ファイルと設定を読み込むモジュール
├── data             # jsonlからデータを読み込むモジュール
└── plot             # データを描画するモジュール
```

図に示すように使用される。
ROSに依存しないパッケージではあるが、ROSのノードにもライブラリとしてimportされるので、ROSパッケージとしてもインストールされる。

![architecture](./images/architecture.drawio.svg)

## 注意

現状では obstacle_segmentation の result.jsonl の分析のみ可能
必要に応じて、各use caseに対応した分析モジュールを追加する。
analysis, config, dataにuse_case名.pyファイルを追加する。

## インストール方法

- driving_log_replayer_cli と一緒にインストールされる
- driving_log_replayer と一緒に ros のパッケージとしてインストールされる

## 使い方

```shell
driving_log_replayer_analyzer analysis ${use-case-name} ${result.jsonl_path} [-c ${config_path}]
```
