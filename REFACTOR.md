# refactor

- [] frameをsetしてからwriteするまで更新される可能性がある。コピーを渡すほうがいいかも
- [] frameを渡したら、set_frameを自動でやって書き込みまで自動にしたい。resultにwriterをもたせる。
- [] gtestでobstacle_segmentationのcppノードもテストする
- [] result.jsonlもpydantic.BaseModelでフォーマットガチガチにしてmodel_dump()->dict->json_dump->strの流れのほうがいいかも
