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

## プログラムの概要
- flow_sensor_test

  流量制御の試験用プログラム（旧）
- flow_sensor_test_pid

  flow_sensor_testの改良版．流量制御の試験をするならこれ
- flow_sensor_test_dob

  flow_sensor_test_pidに外乱オブザーバを追加したプログラム
- integration_for_Arduino_Mega4

  本番用プログラム（旧）．流量，燃焼器内圧，点火制御および通信をおこなう
- MegaFire_pid

  integration_for_Arduino_Mega4の改良版
- MegaFire_dob

  MegaFire_pidに外乱オブザーバを追加したプログラム

- LoRa_Serial_Master_Slave.ino

  6/26の打上げ時に使ったLoRaのプログラム