# 起動方法

driving_log_replayer を起動するには、autoware のワークスペースのビルド、cli のインストール、及び、評価するデータの準備を事前に済ませている必要がある。

## コマンド

driving_log_replayer_cli のコマンドについて解説する。

driving_log_replayer_cli をインストールすると、ターミナルで driving_log_replayer というコマンドが実行できるようになる。
driving_log_replayer コマンドは、サブコマンドを持っている。
各コマンドに必要な引数は--help オプションを指定すると表示できるようになっている。

```shell
# driving_log_replayer top level help
driving_log_replayer --help

# show version
driving_log_replayer --version

# show subcommand help
driving_log_replayer subcommand --help

# show subsubcommand help
driving_log_replayer subcommand subsubcommand --help
```

## cli サブコマンド

サブコマンドとして以下が存在する

- configure
- simulation

### driving_log_replayer configure

設定ファイル.driving_log_replayer.config.toml を操作するコマンド。

```shell
# -pで指定したprofile名デフォルト値defaultにdata_directory、output_directory、autoware_pathを設定する
driving_log_replayer configure register -d ${data_directory} -o ${output_directory} -a ${autoware_path} [-p ${profile}]
```

### driving_log_replayer simulation

simulation 実行に利用する。

```shell
# simulation 実行、jsonlとjsonの両方の結果ファイルが出力される
driving_log_replayer simulation run -p ${profile}

# simulation 実行、jsonlをjsonに変換しない
driving_log_replayer simulation run -p ${profile} --no-json

# 結果の確認、output_directory以下の結果ファイルのサマリーを表示する
driving_log_replayer simulation show-result ${output_directory}

# 結果ファイルをjson変換、クラウドで実行したjsonlやno-jsonで実行した結果ファイルを変換する
driving_log_replayer simulation convert-result ${output_directory}
```

## wasim による driving_log_replayer 実行

TIER IV が提供している[Autoware Evaluator](https://docs.web.auto/user-manuals/evaluator/introduction "Autoware Evaluator")へ
アクセス権がある場合は[wasim](https://docs.web.auto/developers-guides/wasim/introduction "wasim")を利用することもできる。

使い方は[ドキュメントサイト](https://docs.web.auto/developers-guides/wasim/use-cases/run-simulations-locally/ "ドキュメントサイト")を参照。

wasim は Autoware Evaluator に登録済みのシナリオをダウンロードして実行するので、クラウド環境に登録済みのシナリオしか実行出来ない。
クラウドに登録してないシナリオは driving_log_replayer_cli を使用する。
