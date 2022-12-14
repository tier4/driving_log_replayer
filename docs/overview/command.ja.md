# コマンド

driving_log_replayer_cli をインストールすると、ターミナルで `driving_log_replayer` というコマンドが実行できるようになります。
`driving_log_replayer` コマンドは、サブコマンドを持っています。
各コマンドに必要な引数は `--help` オプションを指定すると表示できるます。

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

設定ファイル `.driving_log_replayer.config.toml` を操作するコマンド。

```shell
# -pで指定したprofile名にdata_directory、output_directory、autoware_pathを設定する。
# -pを省略した場合はprofile名にdefaultが指定される
driving_log_replayer configure register -d ${data_directory} -o ${output_directory} -a ${autoware_path} [-p ${profile}]
```

### driving_log_replayer simulation

simulation を実行するコマンド

```shell
# simulation 実行、jsonlとjsonの両方の結果ファイルが出力される
driving_log_replayer simulation run -p ${profile}

# simulation 実行、jsonlをjsonに変換しない
driving_log_replayer simulation run -p ${profile} --no-json

# 結果の確認、output_directory以下の結果ファイルのサマリーを表示する
driving_log_replayer simulation show-result ${output_directory}

# 結果ファイルをjsonに変換する
driving_log_replayer simulation convert-result ${output_directory}
```

## wasim による driving_log_replayer 実行

TIER IV が提供している[Autoware Evaluator](https://docs.web.auto/user-manuals/evaluator/introduction)へ
アクセス権がある場合は[wasim](https://docs.web.auto/developers-guides/wasim/introduction)を利用することもできる。

使い方は[ドキュメントサイト](https://docs.web.auto/developers-guides/wasim/use-cases/run-simulations-locally/)を参照。

wasim は Autoware Evaluator からシナリオをダウンロードして実行するので、クラウド環境に登録済みのシナリオしか実行出来ない。
クラウドに登録してないシナリオは driving_log_replayer_cli を使用する。
