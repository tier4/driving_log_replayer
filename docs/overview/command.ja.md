# コマンド

log_evaluatorはシナリオのパスと、評価結果の出力ディレクトリを指定することで起動できる。

```shell
ros2 launch log_evaluator dlr.launch.py scenario_path:=${scenario_path} output_dir:=${output_dir} [resource_dir:=${resource_dir}]
```

## CLI

version 2までで利用してたlog_evaluator_cliは廃止

## wasim による log_evaluator 実行

TIER IV が提供している[Autoware Evaluator](https://docs.web.auto/user-manuals/evaluator/introduction)へ
アクセス権がある場合は[wasim](https://docs.web.auto/developers-guides/wasim/introduction)を利用することもできる。

使い方は[ドキュメントサイト](https://docs.web.auto/developers-guides/wasim/use-cases/run-simulations-locally/)を参照。

wasim は Autoware Evaluator からシナリオをダウンロードして実行するので、クラウド環境に登録済みのシナリオしか実行出来ない。
クラウドに登録してないシナリオは log_evaluator_cli を使用する。
