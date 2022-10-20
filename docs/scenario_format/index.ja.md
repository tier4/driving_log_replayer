# Driving Log Replayer シナリオフォーマット定義

driving_log_replayer で用いるシナリオのフォーマットについて述べる。

## フォーマットに関する注意事項

- キーは CamelCase にて定義する。
- 座標系に関しては、特に指定なければ Lanelet2 と同じ座標系にて記述。マップ座標系。
- 単位系に関しては、特に指定がなければ以下を使用する。

```shell
距離: m
速度: m/s
加速度: m/s^2
時間: s
時刻: UNIX TIME
```

## サンプル

シナリオのサンプルを docs/sample フォルダに格納している。

## フォーマット

基本構造は以下の通り。各キーの詳細は以下で記述する。

### 2.x.x フォーマット

localizationとperformance_diagで使用する。

```yaml
ScenarioFormatVersion: 2.x.x
ScenarioName: String
ScenarioDescription: String
SensorModel: String
VehicleModel: String
VehicleId: String
LocalMapPath: String
Evaluation:
  UseCaseName: String
  UseCaseFormatVersion: String
  Conditions: Dictionary # refer use case
```

### 3.x.x フォーマット

perceptionとobstacle_segmentationで使用する。
VehicleIdとLocalMapPathがt4_datasetのid毎に設定するように変更されている。

```yaml
ScenarioFormatVersion: 3.x.x
ScenarioName: String
ScenarioDescription: String
SensorModel: String
VehicleModel: String
Evaluation:
  UseCaseName: String
  UseCaseFormatVersion: String
  Datasets:
    - DatasetName:
        VehicleId: String
        LocalMapPath: String
  Conditions: Dictionary # refer use case
```

### ScenarioFormatVersion

シナリオフォーマットのバージョン情報を記述する。セマンティックバージョンを用いる。

localizationとperformance_diagは2.x.x系を使用する。2.x.xの最新バージョンは 2.2.0
perceptionとobstacle_segmentationは3.x.x系を使用する。3.x.xの最新バージョンは3.0.0

フォーマットの更新の度にマイナーバージョンを更新する。

### ScenarioName

シナリオの名前を記述する。Autoware Evaluator 上でシナリオの表示名として使用される。

### ScenarioDescription

シナリオの説明を記述する。Autoware Evaluator 上でシナリオの説明として使用される。

### SensorModel

autoware_launch/launch/logging_simulator.launch.xml の引数の sensor_model を指定する

### VehicleModel

autoware_launch/launch/logging_simulator.launch.xml の引数の vehicle_model を指定する

### VehicleId

autoware_launch/launch/logging_simulator.launch.xml の引数の vehicle_id を指定する。

車両 ID の指定がない場合は、default を指定する。

### LocalMapPath

ローカル環境で使用する地図のフォルダのパスを記述する。

`$HOME`のような環境変数を使用することが出来る。

### Evaluation

シミュレーションの評価条件を定義する。

#### UseCaseName

評価プログラムを指定する。

ここで指定された名前と同じ名前の launch ファイルを呼び出すことで評価が実行される。
driving_log_replayer/launch に指定した名称と同じ名称の launch.py ファイルが存在している必要がある。

#### UseCaseFormatVersion

ユースケースのフォーマットのバージョン情報を記述する。セマンティックバージョンを用いる。
メジャーバージョンが 1 になるまでは、フォーマットの更新の度にマイナーバージョンを更新する。初期バージョンは 0.1.0。

#### Conditions

ユースケース毎に設定できる条件を指定する。

指定可能な条件は各ユースケースを参照。
