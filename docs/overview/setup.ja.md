# 設定

driving_log_replayer を利用するのに必要な設定を述べる。

## フォルダ構成、ファイル命名規則

driving_log_replayer が期待するフォルダ構成、ファイル命名規則について解説する。

driving_log_replayer では、フォルダ構成、ファイル名などを固定にすることで、シナリオに記述するパスや、コマンドに渡す引数を少なくしている。
また、`data_directory` に複数のフォルダを置くことで、複数のテストを連続で実行できるようになっている。

### データフォルダ

シミュレーションで使用するリソースを保存しておくフォルダ。

テストケース毎に、シナリオと、bag、dataset を配置する。

### t4_datasetを使用しないユースケースのデータフォルダ構成

```shell
profile_data_directory                // .driving_log_replayer.config の profile 毎のdata_directory
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

### t4_datasetを使用するユースケースのデータフォルダ構成

```shell
profile_data_directory                 // .driving_log_replayer.config の data_directory
│
├── TC001                           // テストケースディレクトリ 任意の名称をつけてよい
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
├── TC002                           // テストケースディレクトリ S001と構成は同じ
...

```

### profileで共通のシナリオファイルを使いまわす場合

同じ地図、車両、センサー構成で取得したbagファイルを一つの共通シナリオで評価したい場合に使用する。
annotationless_perceptionで閾値を決定するためにメトリクスを求めるだけのシミュレーションを実行したい場合に利用できる。

例えば、以下の場合は、TC001はbase_scenario.yamlで評価されて、TC002はディレクトリにあるscenario.yamlが使われる

```shell
profile_data_directory                // .driving_log_replayer.config の data_directory
├── base_scenario.yaml　　　　　　 // ディレクトリにシナリオファイルが存在しないときに使われる共通シナリオ
│
├── TC001                          // テストケースディレクトリ 任意の名称をつけてよい
│   └── input_bag                 // センサーデータが収録された入力用のbag
│       ├── input_bag_0.db3       // bagのバイナリファイル
│       └── metadata.yaml         // bagのmetadata
│
├── TC002                          // テストケースディレクトリ 任意の名称をつけてよい
│   ├── scenario.yaml             // ディレクトリ内のシナリオファイルが使用される。base_scenario.yamlは無視される
│   └── input_bag                 // センサーデータが収録された入力用のbag
│       ├── input_bag_0.db3       // bagのバイナリファイル
│       └── metadata.yaml         // bagのmetadata
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
