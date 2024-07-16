# log_evaluator 結果ファイルフォーマット

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

## 結果ファイルの分析

vscodeのJSONL Converterを用いるとボタンを押すだけで容易にjsonl <-> jsonの変換ができる

<https://marketplace.visualstudio.com/items?itemName=F-loat.jsonl-converter>

python を用いる場合は、[pandas.read_json で lines=True とすると jsonl を読み込むことができる](https://pandas.pydata.org/docs/reference/api/pandas.read_json.html)
