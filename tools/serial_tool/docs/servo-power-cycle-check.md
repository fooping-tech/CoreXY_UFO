# Servo Power Cycle Check

STS3020 だけ電源を入れ直したあと(Core2 は動いたまま)の、`SERVO_STATUS` と JOG の確認手順です。
依頼元: CoreXY_UFO_Game `docs/HANDOFF_FIRMWARE_Z_SERVO.md`(ゲーム側 Issue #62)、このリポジトリの Issue #11。

## 何が起きるか

- STS3020 の多回転モード(角度制限 `0/0`)は EEPROM のロックを掛けたまま書いているので、サーボの電源を切ると消え、電源投入時の `0/4095`(1回転)に戻る。
- ファームウェアは追跡中(`tracking=YES`)のままになる。サーボの回転数(絶対位置 `abs`)はもう失われている。

## 応答

| 行 | 意味 |
|---|---|
| `SERVO_STATUS id=N result=OK ... limits=0/4095 tracking=LOST abs=..` | 追跡中なのにサーボが1回転モードに戻っている。`abs` は古い値 |
| `JOG servo id=N begin_ms=.. (hold check; XY waits) tracking=KEPT` | 追跡を続けたまま JOG を始めた(`abs` は連続) |
| `JOG servo id=N begin_ms=.. (hold check; XY waits) tracking=NEW` | 追跡していなかったので、ここで多回転にした(起動後の初回など) |
| `JOG servo id=N begin_ms=.. (hold check; XY waits) tracking=RESET` | 多回転が失われていたので設定し直した。`abs` は今の1回転内の位置から振り直し |
| `JOG servo id=N disabled reason=.. multi_turn_lost=YES` | 設定し直しに失敗した(例: `goal_mismatch`)。このセッションではそのサーボを動かさない |

JOG の終了時は、サーボが保持点で止まるまで(最大 600 ms)追跡を続ける。終了直後に次の JOG を始めても `idle_drift` にならない。

## 実機結果(2026-11-08)

- `--id 1 --servo-turn`: 9/9 PASS。raw 189→3318(0 をまたいで -966 counts)
- `--id 2 --rate 400`: 7/7 PASS。raw 3666→87→595(4095 をまたぐ)

`SERVO_TURN`/`SERVO_ANGLE` は従来どおり、1回目を `result=REJECTED reason=multi_turn_lost (retry)` で拒否し、2回目で多回転に戻して動かす。

## 自動チェック(サーボが動く)

サーボの電源だけを切れるようにしておく(サーボ用電源のスイッチやコネクタ)。XY は動かない。
現在の機体は ID 1/2 の電源が共通なので、入れ直すと両方が `tracking=LOST` になる(対象でない方はそのまま)。
スクリプトはポートを開くと Core2 がリセットされるため、1回の実行の中で電源を入れ直す(実行の合間ではない)。
Z ドラムに荷重がある場合は、`--rate` の向き(符号)と大きさを確かめてから実行する。

```bash
cd tools/serial_tool
../../.venv/bin/python servo_power_cycle_check.py --port /dev/cu.usbserial-XXXX --id 1
../../.venv/bin/python servo_power_cycle_check.py --port /dev/cu.usbserial-XXXX --id 2 --rate 400
# SERVO_TURN の従来動作も見る(2回目の電源入れ直しを求められる)
../../.venv/bin/python servo_power_cycle_check.py --port /dev/cu.usbserial-XXXX --id 1 --servo-turn
```

1. JOG で追跡を始め、`tracking=YES limits=0/0` を確認する。
2. 画面の指示でサーボだけ電源を入れ直す → `SERVO_STATUS` が `tracking=LOST`。
3. JOG → `tracking=RESET` で動き出し、その後 `tracking=YES limits=0/0`。
4. もう一度 JOG → `tracking=KEPT`、`abs` が JOG の量だけ変わる(1回転の端で止まらない)。
5. (`--servo-turn`)もう一度入れ直し → `SERVO_TURN` が1回目 `REJECTED multi_turn_lost`、2回目 `PASS`。

## 手動チェック

`examples/servo_power_cycle_check.csv` で、電源の入れ直しの前後の `SERVO_STATUS 1/2` を見る。

```bash
../../.venv/bin/python serial_send.py --port /dev/cu.usbserial-XXXX --csv examples/servo_power_cycle_check.csv
```

ジョイスティック(Atom)の Z/アームのジョグも同じ `beginJog()` を通るので、入れ直し後の最初の押下で動くことを目で確かめる。
