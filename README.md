# CoreXY Plotter Firmware

M5Stack Core2でCoreXYペンプロッタを制御するためのPlatformIOファームウェアです。
初期版は、安全なbring-upと将来のplanner拡張に必要な責務分離を優先しています。

## Build

```bash
pio run
```

## Host WebUI

PCやRaspberry PiからUSB Serial経由で状態確認、手動操作、G-code送信を行う
Host WebUIがあります。

起動方法、`.venv`の作り方、シリアルポート設定は
[tools/webui/README.md](tools/webui/README.md)を参照してください。

## Simulation mode

`include/PlotterConfig.h`の`SIMULATION_MODE`は初期確認時に`1`へ切り替えて使えます。この状態では
`XY`、`TEST_A`、`TEST_B`コマンドは予定動作をログ表示しますが、モータ出力APIを
呼びません。実機確認前に`SELFTEST`とsimulation用コマンドを実行してください。

## Safety

- 初回起動はモータ電源を切った状態で行ってください。
- GPIO35/36のリミット入力には外付けpull-upが必要です。
- STS3020には外部7.4V電源（1台あたり3A以上を目安）と、1Mbps半二重TTL UART変換基板（例: Waveshare Bus Servo Adapter (A)）が必要です。Core2の5V/3.3Vから給電しないでください。
- TMC2209 UART TX側には1kΩ直列抵抗を入れてください。
- Port AのGPIO32=STS3020 UART TX、GPIO33=RX。既存のGPIO33 NeoPixelは競合するため**無効化**しています。物理的なNeoPixel配線も外してください。
- 起動時はSTS3020への位置指令を送信しません。ホーンを外して`PEN_STATUS`で疎通確認してから`PENUP`/`PENDOWN`を試してください。
- TMC2209 A/BのENはGND固定で常時activeです。電気的に停止するには外部スイッチ
  またはE-stop回路が必要です。

## Bring-up commands

```text
PEN_STATUS
PENUP
PENDOWN
TMC_INIT
MELODY
G28
G21
G90
G1 X10 Y10 F600
G91
G1 X5 Y0 F600
M3
M5
M114
```

`MELODY`は診断専用です。起動時には自動再生せず、通常motionがidleでTMC UARTが
readyの場合だけ明示実行します。終了または中断時には通常TMC profileへ戻します。

## Text to G-code

日本語文字列を描画する場合は、ホスト側CLIの`tools/text_tool/kst32b_to_gcode.py`で
KST32BストロークフォントデータからG-codeを生成します。InkscapeやHershey Textには
依存しません。詳細は[tools/text_tool/README.md](tools/text_tool/README.md)を参照してください。

詳細: [STS3020接続・単体確認](tools/serial_tool/docs/sts3020-check.md)。サーボの初期ID=1、初期通信速度=1Mbps、0..360度と0..4095の角度換算を想定します。位置指令後に最大2秒間、実位置の誤差35カウント以内を確認します。ただしサーボ内部エンコーダの位置確認であり、ペン機構の接触・干渉がないことは保証しません。

## STS3020 servo ID setup

FEETECH STS3020を同じシリアルバスに直列接続する場合、各サーボのIDを重複させないでください。
新品は通常ID=1で出荷されるため、2台目以降は接続前にIDを変更します。
IDが重複すると、同じ指令に複数台が同時に応答して通信が壊れます
(PINGの不規則な失敗、`reply from unexpected ID`、SCANで何も見つからない等)。

ID割り付け:

| ID | 用途 |
|---:|---|
| 1 | Z軸(アーム上下、紐とドラム) |
| 2 | アーム |

ID設定には単体テストファームウェア`m5stack-core2-sts3020-test`を使います。
コマンドはUSB Serial(115200bps)から入力します。

```bash
pio run -e m5stack-core2-sts3020-test -t upload
pio device monitor -b 115200
```

手順:

1. **IDを変更するサーボ1台だけ**をバスに接続し、外部サーボ電源を入れる
2. `SCAN`で、そのサーボだけが応答することを確認する(例: `ID=1: found`、`1 servo(s) found`)
3. 必要なら`ID <現在のID>`で操作対象を選ぶ(起動時はID=1)
4. `SET_ID <新しいID>`を実行し、`SET_ID OK: servo now answers ID=<新しいID>`を確認する
5. `SCAN`と`STATUS`で新しいIDで応答することを確認する
6. 全サーボを直列に戻し、`SCAN`で全IDが応答し、`bad reply`がないことを確認する

`SET_ID`の安全条件:

- 選択中IDへのPINGが10回連続で成功し、broadcast PINGがそのIDだけを返すこと
  (複数台接続中は拒否される)
- 新しいIDに応答する機器がいないこと
- 書込み前にtorqueをOFFにし、EEPROMを一時的にunlockしてIDを書き、その後lockする

IDはサーボのEEPROMに保存され、電源を切っても保持されます。
テストファームウェアは起動時にID=1を選択するので、他のサーボを操作するときは`ID <n>`で切り替えます。
詳細は[tools/serial_tool/docs/sts3020-test-check.md](tools/serial_tool/docs/sts3020-test-check.md)を参照してください。

## Known limitations

- TMC2209のMS1/MS2またはジャンパで、A/Bアドレスをそれぞれ`0`と`1`に設定する必要があります。
- `TMC_STATUS`の診断値とメロディ用1200mA profileは実機条件に合わせた確認が必要です。
- `TEST_A`/`TEST_B`の独立A/B moveはbring-up用で、厳密なXY線形補間を保証しません。
- G-codeは最小対応です。`G0/G1/G4/G20/G21/G28/G90/G91/M3/M5/M114`のみ対応し、arc、Z、checksum検証、完全なGRBL互換は未実装です。`G4`は`P`ミリ秒指定のみ対応します。
- look-aheadとjunction deviationは実装済みですが、実機調整は未完了です。
# CoreXY_Plotter
# CoreXY_Plotter
