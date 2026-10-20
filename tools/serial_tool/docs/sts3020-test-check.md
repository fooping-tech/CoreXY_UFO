# STS3020 Standalone Test Check

FEETECH STS3020シリアルバスサーボ1台を、CoreXYファームウェアとは別の専用ファームウェア
`m5stack-core2-sts3020-test`で通信確認・微小動作確認するチェックです。
CoreXY、TMC2209、NeoPixel、LCD UIタスクは起動しません。

## Wiring (必須確認)

| 項目 | 内容 |
|---|---|
| Port | Core2 Port A (赤): GPIO32 = UART1 TX、GPIO33 = UART1 RX |
| 半二重変換 | バスサーボアダプタ(Waveshare Bus Servo Adapter (A)、FE-URT-1等)経由。ESP32 TX/RXをサーボ信号線へ直結しない |
| サーボ電源 | 外部電源からアダプタ経由。**サーボEEPROMの入力電圧範囲内**(`STATUS`の`Min voltage`..`Max voltage`)であること |
| GND | 外部電源GNDとCore2 GNDを共通にする |
| 負荷 | 初回はホーン・負荷なし |
| UART | 1,000,000bps、8N1、ID=1 (`src/sts3020test/Sts3020TestConfig.h`) |

## Build / Upload

```bash
pio run -e m5stack-core2-sts3020-test
pio run -e m5stack-core2-sts3020-test -t upload
pio device monitor -b 115200
```

## CSV

`tools/serial_tool/examples/sts3020_test_check.csv`

```bash
python tools/serial_tool/serial_send.py \
  --port /dev/cu.usbserial-XXXX \
  --csv tools/serial_tool/examples/sts3020_test_check.csv \
  --startup-delay 4 \
  --timeout 5 \
  --echo
```

PING/READ/STATUSが失敗、または`Servo error flags`が`none`以外の場合は、JOG以降を実行しないこと。
ファームウェア側もJOG時に同じ条件を検査し、満たさない場合は動かさない。

## Commands

| Command | 動作 |
|---|---|
| `HELP` | コマンド一覧 |
| `PING` | ID=1へPing。失敗時はbroadcast PingでID相違を判定し、原因候補を表示 |
| `READ` | 現在位置(0..4095 counts) |
| `STATUS` | ID、model、mode、torque、位置、goal、角度制限、速度、負荷、電圧、電圧制限、温度、電流、moving |
| `JOG <+/-n>` | 現在位置からn counts移動(\|n\|<=100) |
| `OFF` | トルクOFF。**非常停止ではない**。危険時はサーボ電源を遮断する |
| `SCAN` | ID 0..20へPINGし応答IDを一覧。壊れた応答はID重複の可能性として表示 |
| `ID <n>` | 以降の全コマンドの対象IDを切替(起動時ID=1)。LCDにも表示 |
| `SET_ID <n>` | 選択中サーボのIDを変更(EEPROM保存)。**対象1台だけ接続**した状態で実行。複数台応答時は拒否 |
| `MT_ON` / `MT_JOG <+/-n>` / `MT_OFF` | 多回転位置制御の試験(負荷なし専用)。詳細はSPEC.md |

ID設定手順は[README.md](../../../README.md)の「STS3020 servo ID setup」を参照。

## Touch UI

LCDに位置、Torque、Link、電圧、servo error flags、直前の結果(`Last:`)を表示し、1秒周期で更新する。

| 画面ボタン | 動作 |
|---|---|
| `-100` / `-30` / `+30` / `+100` | `JOG`と同じ検査付き経路で1回だけ移動 |
| `READ` | 位置読出し |
| `OFF` | トルクOFF(非常停止ではない) |

画面下のタッチ領域: BtnA=`-30`、BtnB=`OFF`、BtnC=`+30`。

- 1タップ=1回。長押しで連続動作しない
- 実行中のボタンはオレンジ表示。実行中にタップしても破棄される
- タッチ操作もSerialへ`[TOUCH] <label>`とJOGログを出力する

| Step | What to confirm |
|---|---|
| 起動直後 | 画面表示のみでサーボが動かない。`Link: OK`、`Flags: none` |
| `+30`タップ | 約2.6度回り`Last: JOG +30 PASS`、Posが+30付近 |
| `-30`タップ | 元へ戻り`Last: JOG -30 PASS` |
| 範囲外(Pos>3965で`+30`) | `Last: JOG +30 REJECT: range`、動かない |
| `OFF`タップ | `Torque: OFF`、`Last: OFF OK` |
| BtnA/BtnB/BtnC | `-30` / `OFF` / `+30`と同じ動作 |

## JOG Sequence

1. Ping、servo error flags=0、mode=0(position)を確認
2. 現在位置を読む。失敗なら動かさない
3. 目標 = 現在 + n
4. 目標が100..3995外、またはサーボ角度制限外なら拒否
5. goal=現在位置を書いてからtorque ON(古いgoalへの飛びを防止)、その後 speed 200 counts/s、ACC 10で目標を指令
6. 20ms周期で位置を読み、±5 counts以内かつmoving=0で到達
7. `Current` / `Target` / `Write command` / `Actual` / `Result: PASS|TIMEOUT`を表示。2秒で到達しなければTIMEOUT

## Checks

| Step | What to confirm |
|---|---|
| 起動ログ | `TX: GPIO32`、`RX: GPIO33`、`Baud: 1000000`、`Servo detected: ID=1`。起動時に動かない |
| `PING` | `Servo detected: ID=1`、`Servo error flags: 0x00 (none)` |
| `READ` | `Position:`が0..4095。JOG±30には100+30..3995-30付近が必要 |
| `STATUS` | `Mode: 0 (position)`、`Voltage`が`Min voltage`..`Max voltage`内、温度が常温 |
| `JOG +30` | 約2.6度回り`Result: PASS` |
| `JOG -30` | 元位置へ戻り`Result: PASS` |
| `OFF` | `Torque disabled`、手で軸が回る |

## Troubleshooting

- `Servo error flags: 0x01 VOLTAGE`: 電源電圧がEEPROMの`Max voltage`/`Min voltage`外。電源電圧をサーボ定格に合わせる。EEPROM制限値を書き換えて回避しない
- `Position`が100未満または3995超: トルクOFFの状態で出力軸を手で中央(約2048)付近へ回してから再試行。硬くて回せない場合は、範囲内へ戻る向きに`JOG -100`(4095付近)または`JOG +100`(0付近)を1回実行してから±30を行う
- 実機確認済み電源は約8.0V。12VではVOLTAGE flag(EEPROM上限10.0V超)
- `PING FAILED`: 表示される原因候補(ID、baud、TX/RX、アダプタ電源、サーボ電源、共通GND)を確認

## Multi-turn Check (負荷なし専用)

```text
STATUS            # Multi-turn: OFF、Max limit 4095、EEPROM lock 1、flags none
MT_ON             # Limits now: 0..0、Tracking zero
MT_JOG +30        # Hold check: OK、Result: PASS
MT_JOG +4096      # 右1回転 PASS
MT_JOG -4096
MT_JOG +12288     # 右3回転 PASS(約16s)
MT_JOG -12288
MT_OFF            # Limits now: 0..4095
OFF
```

- `+`が時計回り(出力軸側から見て右)
- Present Positionは1回転ごとに0..4095へ折り返す。ファームウェアが折り返しを補正し`abs`として表示する
- `MT_ON`が`REFUSED: goal register ... != position`の場合はサーボ電源を再投入してから再実行
- `Hold check`で`ABORTED`または`RUNAWAY`が出たら即torque OFF済み。サーボ電源を再投入する
- 多回転中は通常`JOG`/タッチJOGは拒否される。MT中の`OFF`で追跡は無効になり、`MT_OFF`→`MT_ON`が必要
- 角度制限0/0は保存されない。サーボ電源再投入で0/4095に戻る
