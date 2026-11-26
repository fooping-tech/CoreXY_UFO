# Servo Diag Check

サーボの移動中の電圧・電流・状態の記録(`SDIAG`)の確認手順です(Issue #28、調査用)。

## 仕様

- `SERVO_TURN`(`moveRelative`)の監視ループで、位置を読むたびにサーボの`FeedBack()`も読み、移動ごとに 1 行出す。判定は変えない。
  - `SDIAG id= result= delta= ms= samples= ok= v_min= v_max= i_max= load_max= status_or= fail_noreply= fail_crc= fail_other= first_fail_ms=`
  - 電圧は 0.1 V 単位、電流は生の値(約 6.5 mA/単位)、負荷は 0.1 % 単位で符号付き、`status_or`はサーボの状態バイトの OR(bit0 電圧、bit1 センサー、bit2 温度、bit3 電流、bit5 過負荷)。
- PASS 以外のとき、直近 36 サンプルを`SDIAG_S id= t:pos/電圧V/電流I/負荷L/状態/温度C`で 6 個ずつ。読めなかったサンプルは`t:ERRn`(1 無応答、2 CRC、3 ID 違い、4 長さ)。
- `feedback_lost`のあと、最大 1.5 s Ping して`SDIAG_AFTER`(戻ったか、`restarted=YES|NO`)。
- 行は結果の行(`SERVO_TURN id=...`)の前に出る。

## 確認(爪が 36° 動く)

**安全**: 爪(ID 2)が 0.10 回転閉じて、同じだけ戻る。爪の間に指や物がないこと。異常時は`ABORT`。

```bash
python tools/serial_tool/serial_send.py \
  --port /dev/cu.usbserial-XXXX \
  --csv tools/serial_tool/examples/servo_diag_check.csv \
  --startup-delay 6 --echo
```

| 確認 | 期待 |
|---|---|
| `SERVO_TURN 2 0.10` | 結果の前に`SDIAG id=2 result=PASS`、`ok`と`samples`が同じ、`fail_*=0`、`status_or=0x00` |
| `v_min`/`v_max` | 電源の電圧(7.6 V の電源なら 73〜76 前後) |
| `SERVO_TURN 2 -0.10` | 同じく`SDIAG id=2 result=PASS` |

失敗時の`SDIAG_S`と`SDIAG_AFTER`は、狙って起こせない(受信線に雑音を入れる必要がある)。ゲームで起きたときのログで見る。読み方の例は SPEC.md の 2026-11-26 #28 の項。

## 実機結果

- 2026-11-26: 通常の開閉で`SDIAG`が出た(7.3〜7.6 V、失敗 0)。Dial の上の失敗で`SDIAG_S`(0.65 s 以降 ERR3、1 回の読出しが 2.2 s 戻らない)と`SDIAG_AFTER back=YES back_ms=694 restarted=NO`を確認。原因はミストの雑音。
