# ELP燃焼器制御プログラム

## はじめに
本リポジトリは，Earth Light Projectの燃焼器制御プログラムのソースコード管理用リポジトリである．

## 使い方
コマンドプロンプト(Windows)またはターミナル(Mac)を開き，お好みのディレクトリに移動したのち，次のコードを実行する．

```
$ git clone https://github.com/nashinokagaku/ELP
```

すると，現在のディレクトリに新しく"ELP"というディレクトリが追加される．
ELPディレクトリ内の任意のプログラムをArduino IDEで開き，マイコンボードに書き込む．

## ブランチの概要
- main

  安定動作する本番用の総合プログラム

- development

  mainの開発用

- flow_sensor_test

  流量系統のテスト用プログラム

- Legacy_Programs

  6/26日の打上で使用した旧プログラム

- GREENHOUSE_LoRa_Legacy

  6/26日の打上げで使用したLoRa用のプログラム