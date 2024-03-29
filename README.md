# ELP燃焼器制御プログラム

## はじめに
本リポジトリは，Earth Light Projectの燃焼器制御プログラムのソースコード管理用リポジトリである．

## 使い方
コマンドプロンプト(Windows)またはターミナル(Mac)を開き，お好みのディレクトリに移動したのち，次のコードを実行する．

```
$ git clone https://github.com/EarthLightProject/MegaFire.git
```

すると，現在のディレクトリに新しく"ELP"というディレクトリが追加される．
ELPディレクトリ内の任意のプログラムをArduino IDEで開き，マイコンボードに書き込む．

## ブランチの概要
- main

  安定動作する本番用の総合プログラム

- development

  mainの開発用←基本これを使っている

- flow_sensor_test

  流量系統のテスト用プログラム

- Legacy_Programs

  6/26日の打上で使用した旧プログラム

- GREENHOUSE_LoRa_Legacy

  6/26日の打上げで使用したLoRa用のプログラム

## コマンド一覧 
### 一般コマンド
- REIG　点火、消火  
- RESET　通信用Megaリセット  
- LOG　ログのスタートストップ  
- STA　ログの状態(１のとき取得中)、スピコンのON・OFF(1のときON)、LPGを流すか(1のとき流す)  
- BME　気密センサハウジングの値(気温、湿度、気圧)  
- BMEOUT　外側BMEの値(気温、湿度、気圧)(繋いでいる場合のみ)  
- FLOW　流量の瞬間的な値と操作量。それぞれ空気,LPGの順  
- LPGCUT　REIGを入れた時にLPGを流さなくするコマンド。初期設定では流す。コマンド入れるごとに反転    
- SENS　管内圧、空気流量、ガス流量、熱電対の順で表示。点火の確認用  
- TIME　マイコン起動からの時間をmin単位で表示  
- MKFL:xxx　SDカードにxxxという名前のファイルを新しく作製。作製後はそのファイルにログを保存。リセットされるとデフォルトのファイルに戻る。ファイル名は16文字以内。拡張子は不要  
### パラメータ設定コマンド
以下のコマンドは若干応答が遅い可能性あり
- $r_a=0.xx　xxの値(単位はL/min)に空気の流量を設定。0.99以上を入力するとErr。=以下なしの場合現在の値を表示    
- $r_g=0.xx　xxの値(単位はL/min)にLPGの流量を設定。0.20以上を入力するとErr。=以下なしの場合現在の値を表示   
- $Kp_g=xxxx　LPGのPゲイン設定。  =以下なしの場合現在の値を表示  
- $Kp_a=xxxx　空気のPゲイン設定。  =以下なしの場合現在の値を表示   
- $Ki_g=xxxx　LPGのIゲイン設定。  =以下なしの場合現在の値を表示   
- $Ki_a=xxxx　空気のIゲイン設定。  =以下なしの場合現在の値を表示  
- $OffSet_g=xxxx　LPGのオフセット設定。 =以下なしの場合現在の値を表示。消火の度に前回の操作量がoffsetに反映されるため、点火の度に変更が必要。  
- $OffSet_a=xxxx　空気のオフセット設定。 =以下なしの場合現在の値を表示。消火の度に前回の操作量がoffsetに反映されるため、点火の度に変更が必要。  
- $LOAD　保存されたパラメータを読み出す。新品のマイコンは空の値になっている。  
- $SAVE　現在のパラメータを保存。電源を切っても記憶される。offsetは点火の度に代わっているので注意。次回起動時からはこの値が読みだされる。  
- $DEFAULT　デフォルトの値を読み出す。値はMegaFire_pid.inoの制御定数定義を書き換えてプログラムすることで設定。  


### 燃焼用LoRa地上局の設定
configuration setting is below.
-  -------------------------------------
-  Node                        : Coordinator
-  Band Width                  : 62.5kHz
-  Spreading Factor            : 10
-  Effective Bitrate           : 489bps
-  Channel                     : 15
-  PAN ID                      : 0001
-  Own Node ID                 : 0001
-  Destination ID              : 0000
-  Acknowledge                 : ON
-  Retry count                 : 3
-  Transfer Mode               : Payload
-  Receive Node ID information : OFF
-  RSSI information            : ON
- Config/Operation            : Configuration
-  UART baudrate               : 115200
-  Sleep Mode                  : No Sleep
-  Sleep Time                  : 50
-  Output Power                : 13dBm
-  Format                      : ASCII
-  Send Time                   : 0
-  Send Data                   :
-  AES Key                     : 00000000000000000000000000000000