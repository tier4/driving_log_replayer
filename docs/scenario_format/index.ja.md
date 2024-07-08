# Driving Log Replayer シナリオフォーマット定義

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
        LocalMapPath: String # Optional
```

### ScenarioFormatVersion

シナリオフォーマットのバージョン情報を記述する。セマンティックバージョンを用いる。

現在は、3.0.0固定

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

ここで指定された名前と同じ名前の launch ファイルを呼び出すことで評価が実行される。
log_evaluator/launch に指定した名称と同じ名称の launch.py ファイルが存在している必要がある。

#### UseCaseFormatVersion

ユースケースのフォーマットのバージョン情報を記述する。セマンティックバージョンを用いる。
メジャーバージョンが 1 になるまでは、フォーマットの更新の度にマイナーバージョンを更新する。初期バージョンは 0.1.0。

#### Conditions

ユースケース毎に設定できる条件を指定する。

指定可能な条件は各ユースケースを参照。

#### Datasets

複数個のDatasetを記述することが可能であるが、複数個のDatasetに対して、同じ評価条件で使用する場合のみ利用できる。
複数個のDatasetを記述した場合は、利用したいdatasetのindexをlaunchの起動引数に渡す必要がある。渡さない場合は0番が暗黙で使われる。

```shell
# シナリオ記述した2番目のdatasetを使用して評価したい場合。
ros2 launch log_evaluator dlr.launch.py scenario_path:=${perception_database_dataset_scenario} output_dir:=${output_dir} database_index:=1

# 未指定は0
ros2 launch log_evaluator dlr.launch.py scenario_path:=${perception_database_dataset_scenario} output_dir:=${output_dir}
```

#### DatasetName

t4_datasetのデータセット名のみ(シナリオからの相対パス)、または、絶対パスを指定する。

#### VehicleId

autoware_launch/launch/logging_simulator.launch.xml の引数の vehicle_id を指定する。

車両 ID が不明な場合は、`default` を設定する。

#### LocalMapPath

ローカル環境で使用する地図のフォルダのパスを記述する。
log_evaluatorの過去のデータセットと互換性をもたせるために存在する項目
LocalMapPathがない場合は、Dataset配下の`map`ディレクトリが自動で使われる。
