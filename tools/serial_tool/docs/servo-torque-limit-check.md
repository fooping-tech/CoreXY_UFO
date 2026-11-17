# Servo Torque Limit Check

爪(アーム)用 STS3020 ID 2 のトルク制限 50% の確認手順です(Issue #13)。

## 仕様

- `SERVO_STATUS <id>` の行末に `torque_limit=`(RAM、いま効いている値)と `max_torque=`(EEPROM)が出る。単位は 0.1%(1000 = 100%)。
- ID 2 は torque ON の直前に毎回 `torque_limit=500` を書く。ID 1(Z ドラム)は 1000 のまま。
- EEPROM の `max_torque` は書き換えない。サーボの電源を入れ直すと `torque_limit` は 1000 に戻り、次に動かしたときに 500 へ書き直す。
- 起動時は書かないので、電源投入から最初の動作までは `torque_limit=1000` と出る(その間 torque は OFF)。

## 安全

- **ID 2(爪)が 18 度動いて戻る。** 爪の周りに物や手がないことを確かめてから実行する。
- ID 1(Z ドラム)は動かさない。
- 異常時はサーボ電源を切る。`SERVO_OFF` は非常停止ではない。

## 手順

```bash
python tools/serial_tool/serial_send.py \
  --port /dev/cu.usbserial-XXXX \
  --csv tools/serial_tool/examples/servo_torque_limit_check.csv \
  --startup-delay 6 --echo
```

| 確認 | 期待 |
|---|---|
| `SERVO_STATUS 2`(動かした後) | `torque_limit=500 max_torque=1000` |
| `SERVO_STATUS 1` | `torque_limit=1000 max_torque=1000` |

## サーボ電源の入れ直し

1. 上の CSV を 1 回通す(`torque_limit=500`)。
2. Core2 は動かしたまま、サーボの電源だけ切って入れ直す。
3. `SERVO_STATUS 2` → `torque_limit=1000 ... tracking=LOST`(RAM が EEPROM の値に戻っている)。
4. CSV をもう一度通す。`SERVO_TURN` の 1 回目は `result=REJECTED reason=multi_turn_lost (retry)` で止まるので([Servo Power Cycle Check](servo-power-cycle-check.md))、もう一度通す。
5. `SERVO_STATUS 2` → `torque_limit=500`。

## 実機結果

- 2026-11-17: 電源投入直後の読出しは ID 1・ID 2 とも `torque_limit=1000 max_torque=1000`。サーボを動かす確認(上の CSV と電源の入れ直し)は未実施。
