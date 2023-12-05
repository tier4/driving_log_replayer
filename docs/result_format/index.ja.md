# Driving Log Replayer 結果ファイルフォーマット

1 行毎に json 形式の文字列が入っている jsonl 形式となっている。

## フォーマット

各行以下の形式のフォーマットで出力される。
実際には一行の文字列だがみやすさのためにフォーマットしている。

```json
{
  "Result": {
    "Success": "true or false",
    "Summary": "評価した結果の要約"
  },
  "Stamp": {
    "System": "PCの時刻",
    "ROS": "シミュレーションの時刻"
  },
  "Frame": {
    "Ego": { "TransformStamped": "mapからbase_linkへのtransform_stamped" },
    "ユースケース毎に構成が異なる": "..."
  }
}
```

結果出力は以下の属性から構成される。

- Result: 実行したシナリオの評価結果
- Stamp: 評価した時刻
- Frame: 受け取った frame(topic) 1 回分の評価結果と、判定に使用した値などの付属情報

Frame の詳細については、各ユースケースの評価結果ファイルフォーマットを参照。

## json ファイルの出力

jsonl は追記可能で、1 回の topic 受信に対して 1 回分の結果を追記することができるため採用している。
しかし、人が目で見て結果を確認する場合は、json 形式でインデントされているほうが理解しやすい。
そのため、driving_log_replayer_cli を用いてシミュレーション実行した場合はデフォルトで json ファイルも出力される。

一方、ローカルで wasim で実行した場合や、クラウドで実行した場合は jsonl ファイルしか作られない。
jsonl ファイルを json に変換したい場合は、以下のコマンドで変換することができる。

```shell
# 結果ファイルの変換、output_directory以下のresult.jsonlをresult.jsonに変換する
dlr simulation convert-result ${output_directory}
```

## 結果ファイルの分析

json 出力の項目でも述べた通り、json ファイルの出力はローカルで driving_log_replayer_cli を用いた場合のみ出力される。
なので、結果ファイルをグラフ化するなどの解析作業を行う場合は、jsonl を解析対象とすること。

python を用いる場合は、[pandas.read_json で lines=True とすると jsonl を読み込むことができる](https://pandas.pydata.org/docs/reference/api/pandas.read_json.html)
