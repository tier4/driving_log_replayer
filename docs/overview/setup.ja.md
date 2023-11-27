# 設定

driving_log_replayer を利用するのに必要な設定を述べる。

## cli の設定

driving_log_replayer_cli では、cli に渡す引数を少なくするために引数に指定するディレクトリを設定ファイルに記載し設定ファイルから読み込む形式を取る。

よって cli を使う前に以下の形式で$HOME/.driving_log_replayer.config.toml ファイルを作成しておく。
手動で作成、もしくは driving_log_replayer configure コマンドで作成する。

```shell
# 手動で作成
nano $HOME/.driving_log_replayer.config.toml

# cliで作成
dlr configure register -d ${data_directory} -o ${output_directory} -a ${autoware_path} [-p ${profile}]
```

profile は最低 1 つ必要で、1 つは `default` という名前である必要がある。

以降で説明するコマンドで-p ${profile}で profile 名を指定すると、プロファイルに指定した設定が読み込まれる。
複数の autoware を切り替えて利用することが出来、プロファイルを何も指定しない場合は default が使用される。

```toml
[profile_name]
data_directory = "シミュレーションの入力に使うデータフォルダのパス"
output_directory = "シミュレーションの結果を出力するフォルダのパス"
autoware_path = "autowareのワークスペースのパス"
```

設定例

```toml
# defaultは必ず必要、profile名を省略したときに選択される
[default]
data_directory = "$HOME/driving_log_replayer_data/default"
output_directory = "$HOME/driving_log_replayer_output/default"
autoware_path = "$HOME/autoware"

[localization]
data_directory = "$HOME/driving_log_replayer_data/localization"
output_directory = "$HOME/driving_log_replayer_output/localization"
autoware_path = "$HOME/autoware"

[yabloc]
data_directory = "$HOME/driving_log_replayer_data/yabloc"
output_directory = "$HOME/driving_log_replayer_output/yabloc"
autoware_path = "$HOME/autoware"

[ar_tag_based_localizer]
data_directory = "$HOME/driving_log_replayer_data/ar_tag_based_localizer"
output_directory = "$HOME/driving_log_replayer_output/ar_tag_based_localizer"
autoware_path = "$HOME/autoware"

[obstacle_segmentation]
data_directory = "$HOME/driving_log_replayer_data/obstacle_segmentation"
output_directory = "$HOME/driving_log_replayer_output/obstacle_segmentation"
autoware_path = "$HOME/autoware"

[perception]
data_directory = "$HOME/driving_log_replayer_data/perception"
output_directory = "$HOME/driving_log_replayer_output/perception"
autoware_path = "$HOME/autoware"
```

## フォルダ構成、ファイル命名規則

driving_log_replayer が期待するフォルダ構成、ファイル命名規則について解説する。

driving_log_replayer では、フォルダ構成、ファイル名などを固定にすることで、シナリオに記述するパスや、コマンドに渡す引数を少なくしている。
また、`data_directory` に複数のフォルダを置くことで、複数のテストを連続で実行できるようになっている。

### データフォルダ

シミュレーションで使用するリソースを保存しておくフォルダ。

テストケース毎に、シナリオと、bag、dataset を配置する。

### localization, performance_diag のデータフォルダ構成

```shell
driving_log_replayer_data             // .driving_log_replayer.config の data_directory
│
├── TC001                          // テストケースディレクトリ 任意の名称をつけてよい
│   ├── scenario.yaml             // シナリオ
│   └── input_bag                 // センサーデータが収録された入力用のbag
│       ├── input_bag_0.db3       // bagのバイナリファイル
│       └── metadata.yaml         // bagのmetadata
│
├── TC002                           // テストケースディレクトリ S001と構成は同じ
...

```

### obstacle_segmentation, perception のデータフォルダ構成

```shell
driving_log_replayer_data              // .driving_log_replayer.config の data_directory
│
├── TC001                           // シナリオディレクトリ 任意の名称をつけてよい
│   ├── scenario.yaml              // シナリオ
│   └── t4_dataset
│       ├── T4D001                 // t4_datasetディレクトリ、sensingの場合は1個
│       │   ├── annotation
│       │   ├── data
│       │   │   └── LIDAR_CONCAT
│       │   └── input_bag
│       └── T4D002                 // t4_datasetディレクトリ、perceptionの場合は複数個持てる
│           ├── annotation
│           ├── data
│           │   └── LIDAR_CONCAT
│           └── input_bag
│          ...
│
├── TC002                           // シナリオディレクトリ S001と構成は同じ
...

```

### マップフォルダ

シミュレーションで使用する地図を保存しておくフォルダ。

```shell
autoware_map
│
├── LocalMapPath1            // シナリオのLocalMapPathで指定するパス
│   ├── lanelet2_map.osm    // laneletファイル
│   └── pointcloud_map.pcd  // pcdファイル
│
├── LocalMapPath2            // シナリオのLocalMapPathで指定するパス
...

```
