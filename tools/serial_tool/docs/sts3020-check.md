# STS3020 / M5Stack Core2 bring-up

この手順は `CoreXY_UFO` の `main` firmware 用。ホーンを外し、XYモータの電源を遮断した状態で開始する。

## Core2 Port A → 半二重TTLアダプタ

| Core2 Port A | Waveshare Bus Servo Adapter (A) UART端子 |
| --- | --- |
| GPIO32 (TX) | TX (同名接続。アダプタのUART mode Aを選択) |
| GPIO33 (RX) | RX |
| GND | GND |
| 5V | 接続しない |

- STS3020本体は3ピンの GND/VCC/Signal をバスサーボアダプタへ接続する。ピン位置は変換基板の表示に従い通電前に確認する。
- サーボ電源は別系統の7.4V、1台でも3A以上を目安とする。Core2 5V/3.3Vから**給電しない**。
- 共通GND。UART信号は3.3V MCUに適合する変換基板を使い、サーボ信号線をESP32へ直接接続しない。
- TMC2209は従来どおりSerial2（GPIO14 TX / GPIO13 RX）を使い、STS3020専用はSerial1。
- GPIO33のNeoPixelは切断する（firmwareはGPIO出力禁止）。旧LEDチェックCSVは使用しない。
- ID 1、1Mbps、ポジションモード前提。変更されている個体は先にFEETECHの設定ツールで確認する。
- 初回はホーンを外し、指を挟まないよう可動部をフリーにする。

## Build / run

```sh
pio run -e m5stack-core2
pio run -e m5stack-core2 --target upload
python tools/serial_tool/serial_send.py --port /dev/cu.usbserial-XXXX --csv tools/serial_tool/examples/sts3020_check.csv --startup-delay 4 --timeout 15 --echo
```

ホスト用CSVは実機を動かす。最初は `CONFIG` と `PEN_STATUS` のみを手動で実行し、通信成立後に次へ進む。

## Commands and expected behavior

1. `CONFIG`: UART TX=32 RX=33、baud=1000000、ID=1、NeoPixel=DISABLED。
2. `PEN_STATUS`: `PEN_STATUS OK id=1 position=...`。これは移動なしの現在位置読出し。
3. `SELFTEST`: CoreXY計算のチェックのみ。サーボ移動なし。
4. `PENUP`: 60度相当の目標値を送信し、`PEN UP` を返す。ホーンなしで動作を確認する。
5. `PEN_STATUS`: 充分に待って実位置が変化したか見る。ファームウェアは2秒以内に位置誤差35カウント以内の到達を待つ。ペン機構の移動完了と一致するか目視確認する。
6. `PENDOWN`: 80度相当の目標値へ指令し、実機の動きと位置を観察する。
7. `M3` / `M5`: G-code経由でPENDOWN/PENUPと同じ経路を使用する。通常描画時に実施。
8. サーボ未給電/通信断のとき `PEN_STATUS` およびペン指令は `ERROR:`。ジョブ中の送信失敗は継続させない。

角度から目標位置は `round(angle_deg * 4095 / 360)` で計算する。60度=約683、80度=約910。サーボゼロ点と実機の取り付け角度は要調整。

## Troubleshooting

- `NO_REPLY`: 外部電源、GND、TX/RX、アダプタのUART mode、サーボID、通信速度、ピン配置を確認する。
- `PEN_STATUS`はOKなのに動かない: ホーンの干渉、モード、電流制限/過負荷、電源電圧降下などを確認する。
- `CONFIG`でLED disabled: 仕様どおり。GPIO33はNeoPixelに使用できない。
- `SIMULATION_MODE=1`では外部UARTを初期化しない。位置読み出しは計算上の値になる。
- `PEN UP` / `PEN DOWN` はサーボ内蔵エンコーダで目標誤差35カウント以内（最大2秒）を確認できたときに返す。ホーン・ペンの実機移動や干渉の不在は保証しないので、目視で確認する。

## Unverified on hardware

PlatformIOビルド・Core2フラッシュ・STS3020実応答・電源負荷・制御速度/加速度・干渉・停電復帰位置は実機により確認する。
