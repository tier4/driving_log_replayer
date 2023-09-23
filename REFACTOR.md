# refactor

- [] frameをsetしてからwriteするまで更新される可能性がある。コピーを渡すほうがいいかも
- [] frameを渡したら、set_frameを自動でやって書き込みまで自動にしたい。resultにwriterをもたせる。
- [] evaluation_targetのdictかlistをResultが持てばupdateは共通処理にできる
- [] localizationではframeのResultが単発のフレームの成否ではなく、累計の成否が出ている
- [] perception系では単発の結果の成否が入っている。
- [] Infoに[{}]となっているが、配列である意味がない
