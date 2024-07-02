# refactor

- [x] t4_datasetを前提とした作りに直す。
- [x] database datasetに合わせに合わせる。読み出すdatasetのindexを指定できるようにする
- [x] 少なくとものこの段階でcliは消す
- [x] ros2 launchの引数をこの段階で減らす。UseCaseのlaunchで同じでなければならない
- [x] シナリオ移行用スクリプト、dlrのdata_directoryにあるシナリオにDatasetsの項目追加した上で、input_bagのディレクトリを変更する処理ができるようにする
- [ ] 上記をやるときに、dry_runモードを用意して、挙動が良さそう確認できるようにする。もしくは、コピーとって実行できるようにする
- [ ] annotationless_perceptionのauto_evaluatorの実行を直す必要がある
- [ ] webauto cliを使って従来と同じように実行できるかの確認をする。強制上書きさせられればそのように動くはず
