# コマンド

driving_log_replayer_cli をインストールすると、ターミナルで `dlr` というコマンドが実行できるようになります。
`dlr` コマンドは、サブコマンドを持っています。
各コマンドに必要な引数は `--help` オプションを指定すると表示できるます。

```shell
# driving_log_replayer top level help
dlr --help

# show version
dlr --version

# show subcommand help
dlr subcommand --help

# show subsubcommand help
dlr subcommand subsubcommand --help
```

## cli サブコマンド

サブコマンドとして以下が存在する

- configure
- simulation

### dlr configure

設定ファイル `.driving_log_replayer.config.toml` を操作するコマンド。

```shell
# -pで指定したprofile名にdata_directory、output_directory、autoware_pathを設定する。
# -pを省略した場合はprofile名にdefaultが指定される
dlr configure register -d ${data_directory} -o ${output_directory} -a ${autoware_path} [-p ${profile}]
```

### dlr simulation

simulation を実行するコマンド

```shell
# simulation 実行、jsonlとjsonの両方の結果ファイルが出力される
dlr simulation run -p ${profile}

# simulation 実行、jsonlをjsonに変換しない
dlr simulation run -p ${profile} --no-json

# 結果の確認、output_directory以下の結果ファイルのサマリーを表示する
dlr simulation show-result ${output_directory}

# 結果ファイルをjsonに変換する
dlr simulation convert-result ${output_directory}
```

## wasim による driving_log_replayer 実行

TIER IV が提供している[Autoware Evaluator](https://docs.web.auto/user-manuals/evaluator/introduction)へ
アクセス権がある場合は[wasim](https://docs.web.auto/developers-guides/wasim/introduction)を利用することもできる。

使い方は[ドキュメントサイト](https://docs.web.auto/developers-guides/wasim/use-cases/run-simulations-locally/)を参照。

wasim は Autoware Evaluator からシナリオをダウンロードして実行するので、クラウド環境に登録済みのシナリオしか実行出来ない。
クラウドに登録してないシナリオは driving_log_replayer_cli を使用する。
