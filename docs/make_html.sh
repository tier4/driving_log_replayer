#!/bin/bash

# ファイル名を設定(拡張子があったら取る)
FILE_NAME=$(basename "${1}" .adoc)

# htmlに変換
asciidoctorj "${FILE_NAME}.adoc" -o "${FILE_NAME}.html" -r asciidoctor-diagram -a allow-uri-read -a data-uri -a toc=left

# docker で一時ファイルをパーミッション644で作成されるとコンテナ外で消せないので、自動的にパーミッションを変更
if test -e "/.dockerenv"; then # .dockerenvはあるか。docker環境かを確認する
    find . -user root -perm 644 -type f -exec chmod 666 {} \;
    find . -user root -perm 755 -exec chmod 777 {} \; # フォルダも同様
fi
