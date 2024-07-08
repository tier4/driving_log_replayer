# コマンド

log_evaluatorはシナリオのパスを指定することで起動できる。

```shell
ros2 launch log_evaluator log_evaluator.launch.py scenario_path:=${scenario_path} [output_dir:=${output_dir} resource_dir:=${resource_dir}]
```

## wasim による log_evaluator 実行

TIER IV が提供している[Autoware Evaluator](https://docs.web.auto/user-manuals/evaluator/introduction)へ
アクセス権がある場合は[wasim](https://docs.web.auto/developers-guides/wasim/introduction)を利用することもできる。

使い方は[ドキュメントサイト](https://docs.web.auto/developers-guides/wasim/use-cases/run-simulations-locally/)を参照。

wasim は Autoware Evaluator からシナリオをダウンロードして実行するので、クラウド環境に登録済みのシナリオしか実行出来ない。
