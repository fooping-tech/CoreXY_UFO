# Servo Feedback Recover Check

受信に雑音が入ってサーボの返事が途切れても、戻ったら回転の数を取り戻す処理の確認手順です(Issue #30)。

## 仕様

- `SERVO_TURN`の途中で 250 ms 以上読めなかったら、最大 5 s サーボの返事を待つ。返事が戻り、角度リミット 0/0・goal = target・停止中で、いまの角度を持つ位置の候補が 1 つに決まれば、追跡を取り戻して続ける(`SPEC.md`の 2026-11-26 #30 の項)。
  - 取り戻せた: `SERVO id=N feedback recovered after ~Nms raw= goal= abs=`、結果は`result=PASS ... recovered=YES gap_ms=`(手前で止まっていれば`TIMEOUT torque=ON ... recovered=YES`)
  - 取り戻せない: `result=FAILED reason=feedback_lost tracking=INVALID recover=<no_reply|multi_turn_lost|goal_changed|still_moving|off_the_path|ambiguous_turn>`
- 雑音の間も読出しは固まらない(`rFlushSCS`の上限 2 ms)。取り戻しの最中の`ABORT`は`result=ABORTED reason=abort feedback_lost`で、hold は書かない。

## 雑音の作り方(どれか 1 つ)

- **ミスト**(実機で起きたもの): ゲームの`SET_MIST CONTINUOUS`(3 s)を、移動を送った直後にデバイステストのパネルから出す。1 回で約 15 s 返事が途切れるので、**5 s を超えて取り戻せない**(`recover=no_reply`)。取り戻せる場合を見るには、雑音を 1〜4 s で止める必要がある。
- **信号線を一瞬抜く**: サーボの信号線(データ線)だけを 1〜3 s 抜いて戻す。電源と GND は抜かない(抜くとサーボが再起動して`multi_turn_lost`になる)。

## 確認(爪が 36° 動く)

**安全**: 爪(ID 2)が 0.10 回転ずつ動く。爪の間に指や物がないこと。雑音の間は`ABORT`もサーボに届かないので、Z(ID 1)ではやらない。異常時はサーボの電源を切る。

```bash
python tools/serial_tool/serial_send.py \
  --port /dev/cu.usbserial-XXXX \
  --csv tools/serial_tool/examples/servo_feedback_recover_check.csv \
  --startup-delay 6 --echo
```

| 確認 | 期待 |
|---|---|
| 1 回目・2 回目の`SERVO_TURN 2` | 雑音なしでは従来どおり`PASS`(`recovered=`なし) |
| 3 回目(雑音 1〜4 s) | `feedback recovered after ~...ms`と`result=PASS ... recovered=YES gap_ms=` |
| 続く`SERVO_STATUS 2` | `tracking=YES`、`abs=`が target(+/-5) |
| 雑音が 5 s を超えたとき | `FAILED ... recover=no_reply`。そのあと`SERVO_STATUS 2`が`OK`なら、次の`SERVO_TURN`は追跡を再設定して動く |

## 実機結果

- 2026-11-26: ゲームのかんたんモードで、ミストの雑音で約 4.5 s 返事がなかった爪の開きが`recovered=YES gap_ms=4494`で`PASS`。この CSV と、候補の判定(レビュー #31 の修正)は未実施。
