# log_evaluator シナリオフォーマット定義

log_evaluator で用いるシナリオのフォーマットについて述べる。

## フォーマットに関する注意事項

- キーは CamelCase にて定義する。
- 座標系に関しては、 `map` 座標系を使用する
- 単位系に関しては、特に指定がなければ以下を使用する。

```shell
距離: m
速度: m/s
加速度: m/s^2
時間: s
```

## サンプル

シナリオのサンプルを[sample](https://github.com/tier4/log_evaluator/tree/develop/sample) フォルダに置いている。

## フォーマット

基本構造は以下の通り。各キーの詳細は以下で記述する。

### 3.x.x フォーマット

```yaml
ScenarioFormatVersion: 3.x.x
ScenarioName: String
ScenarioDescription: String
SensorModel: String
VehicleModel: String
Evaluation:
  UseCaseName: String
  UseCaseFormatVersion: String
  Conditions: Dictionary # refer use case
  Datasets:
    - DatasetName:
        VehicleId: String
```

### ScenarioFormatVersion

シナリオフォーマットのバージョン情報を記述する。セマンティックバージョンを用いる。

現在は、3.0.0

フォーマットの更新の度にマイナーバージョンを更新する。

### ScenarioName

シナリオの名前を記述する。Autoware Evaluator 上でシナリオの表示名として使用される。

### ScenarioDescription

シナリオの説明を記述する。Autoware Evaluator 上でシナリオの説明として使用される。

### SensorModel

autoware_launch/launch/logging_simulator.launch.xml の引数の sensor_model を指定する

### VehicleModel

autoware_launch/launch/logging_simulator.launch.xml の引数の vehicle_model を指定する

### Evaluation

シミュレーションの評価条件を定義する。

#### UseCaseName

評価プログラムを指定する。

ここで指定された名前と同じ名前の評価ノードを呼び出すことで評価が実行される。

#### UseCaseFormatVersion

ユースケースのフォーマットのバージョン情報を記述する。セマンティックバージョンを用いる。
メジャーバージョンが 1 になるまでは、フォーマットの更新の度にマイナーバージョンを更新する。初期バージョンは 0.1.0。

#### Conditions

ユースケース毎に設定できる条件を指定する。

指定可能な条件は各ユースケースを参照。

#### Datasets

複数個のDatasetを記述することが可能であるが、複数個のDatasetに対して、同じ評価条件で使用する場合のみ利用できる。
複数個のDatasetを記述した場合は、利用したいdatasetのindexをlaunchの起動引数に渡す必要がある。
indexは0番から始まる。
データセットが1個の場合はdataset_index:=0としてもよい。

```shell
# シナリオに記述したdataset数が1個の場合。dataset_index:=0は省略可能
ros2 launch log_evaluator dlr.launch.py scenario_path:=${scenario_path} [dataset_index:=0]

# シナリオに記述したdataset数が複数の場合
ros2 launch log_evaluator dlr.launch.py scenario_path:=${scenario_path} dataset_index:=${index_number}
```

#### DatasetName

t4_datasetのデータセット名

#### VehicleId

autoware_launch/launch/logging_simulator.launch.xml の引数の vehicle_id を指定する。

車両 ID が不明な場合は、`default` を設定する。
