# Servo Turn Limit Check

`SERVO_TURN` の 1 回の上限の確認手順です(Issue #16)。

## 仕様

- 1 回の `SERVO_TURN` で動かせる量は ID ごとに決まる(`stsMaxTurns(id)`、`PlotterConfig.h`)。
  - ID 1(Z ドラム): 6 回転(`STS_Z_MAX_TURNS`)。2026-11-20 に 3 から変更。
  - ID 2(アーム/爪): 3 回転(`STS_TEST_MAX_TURNS`)。変更なし。
- 上限を超えると、動かさずに `result=REJECTED reason=turns_over max=<上限>` を返す。
- 追跡位置の上限(|abs| <= 28000 counts)は変わらない。超える移動は `reason=abs_out_of_range`。
- `CONFIG` に `SERVO_LIMITS max_turns id1=6.0 id2=3.0 abs_limit_counts=28000 speed=2400` が出る。ホストはこの行で上限を知る。この行がないファームウェアは、どの ID も 3 回転まで。
- 速度(2400 counts/s)、hold check、runaway guard、読出し欠落の検出、`ABORT` での hold 停止は変わらない。6 回転は約 10.7 秒、timeout は 12.2 秒。

## 動かさない確認

サーボは動かない(上限超過は動作前に断る)。

```bash
python tools/serial_tool/serial_send.py \
  --port /dev/cu.usbserial-XXXX \
  --csv tools/serial_tool/examples/servo_turn_limit_check.csv \
  --startup-delay 6 --echo
```

| 確認 | 期待 |
|---|---|
| `CONFIG` | `SERVO_LIMITS max_turns id1=6.0 id2=3.0 abs_limit_counts=28000` |
| `SERVO_TURN 1 6.01` | `result=REJECTED reason=turns_over max=6.0` |
| `SERVO_TURN 2 3.01` | `result=REJECTED reason=turns_over max=3.0` |

## 動かす確認(Z ドラムが 3.55 回転する)

**安全**

- **ID 1(Z ドラム)が 3.55 回転(14541 counts)下がって、同じだけ上がる。** アームを一番上にして、下に物や手がないことを確かめてから実行する。2026-11-19 の校正の上下の幅(14711 counts)の内側に収まる量にしてある。チェーンの長さを変えた場合は量を見直す。
- 先に `SERVO_STATUS 1` の `abs=` を読む。`abs + 14541` が 28000 を超えると `reason=abs_out_of_range` で断られる。
- 異常時は `ABORT`(その位置で hold)。それでも止まらなければサーボ電源を切る。

```bash
python tools/serial_tool/serial_send.py \
  --port /dev/cu.usbserial-XXXX \
  --csv tools/serial_tool/examples/servo_turn_limit_move_check.csv \
  --startup-delay 6 --echo
```

| 確認 | 期待 |
|---|---|
| `SERVO_TURN 1 3.55` | 途中で止まらずに下がり、`result=PASS`(約 6.5 秒、誤差 +/-5 counts 以内) |
| `SERVO_TURN 1 -3.55` | 同じく 1 回で戻る |

長い移動の途中の中断は、手で確認する。`SERVO_TURN 1 3.55` の 2 秒後に `ABORT` を送り、`result=ABORTED ... hold=YES` でその位置に止まること、止まってから 2 秒以上待って `ALARM_CLEAR` の後に `SERVO_TURN 1 -<動いた分>` で戻せることを見る(hold の直後は減速で行き過ぎて戻るので、すぐ次を送ると `idle_drift` になることがある)。

## 実機結果

- 2026-11-20: 未実施(デバイス未接続)。
