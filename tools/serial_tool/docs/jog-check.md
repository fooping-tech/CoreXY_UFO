# Serial JOG / STOP Check

PC(USBシリアル)からの速度ジョグ`JOG`と、アラームにしない減速停止`STOP`の確認手順です。
依頼元: CoreXY_UFO_Game `docs/HANDOFF_FIRMWARE_JOG.md`(§4 受け入れ条件)。

## コマンド

```text
JOG <vx_mm_s> <vy_mm_s> [<z_counts_s> <arm_counts_s>]
STOP
```

- `JOG`は押している間50ms(20Hz)ごとに送り続ける。最後の`JOG`から`SERIAL_JOG_TIMEOUT_MS`(250ms)来なければ減速停止(デッドマン)。
- `JOG 0 0`で減速停止。`JOG`には`ACK QUEUED`を返さない(ログキュー保護)。
- `TEST_MODE`は不要。シリアルのXYジョグはHOMED必須(Z/アームのみならHOMEDなしで可)。
- `STOP`は実行中の`XY`/ジョグ/ジョブを減速停止し、`STOP`より前にキューに積まれた移動系コマンドを捨てる。アラームにならず、HOMEDも保つ。
- `ABORT`は従来どおり即時停止・アラーム・HOMED解除(非常停止)。

## 応答

| 行 | 意味 |
|---|---|
| `JOG start vx=.. vy=.. z=.. arm=.. src=SERIAL\|JOY HOMED=YES\|NO` | セッション開始(1回) |
| `JOG_POS X=.. Y=.. VX=.. VY=.. Z_ABS=.. ARM_ABS=..` | ジョグ中20Hz。VX/VYは位置変化から算出 [mm/s]。サーボ追跡が無効なら`NA` |
| `JOG soft_limit axis=X X=..` | ソフトリミットで外向き速度が0になった(セッションは継続) |
| `JOG end reason=<..> X=.. Y=.. HOMED=YES\|NO src=..` | 終了(1回)。reason: `zero` `timeout` `idle` `link_lost` `command` `abort` `alarm` `limit_x` `limit_y` |
| `NACK_JOG reason=<not_homed\|alarm\|homing\|job_active\|busy\|bad_args>` | 開始拒否。1回の押下につき1回だけ出す |
| `NACK_<CMD> reason=jog_active` | ジョグ中に移動・設定系コマンドを送った(`XY`は`NACK_XY target=(..) reason=jog_active`) |
| `ACK QUEUED STOP` → `STOP complete X=.. Y=.. HOMED=..` | STOP受付と完了 |
| `NACK_XY target=(..) reason=stopped [X=.. Y=..]` | STOPで止めた/捨てた`XY`。X/Yは止まった位置 |

拒否された押下と、`command`/`alarm`/`limit_*`で終わったセッションの後は、`JOG 0 0`か250ms以上の
送信停止(キーを離す)まで`JOG`を無視する。押しっぱなしのキーで後から勝手に動き出さないため。

ジョグ中もすぐ応答する読み取り系: `POS` `M114` `SERVO_STATUS` `HOME_STATUS` `LIMIT_STATUS`
`TMC_STATUS`(多レジスタ診断は省略) `JOB_STATUS` `CONFIG` `HELP` `PEN_STATUS`。

## 自動チェック(実機が動く)

作業範囲を空け、リミットスイッチと電源断の手段を確認してから実行する。HOMEも実行する。

```bash
python tools/serial_tool/jog_check.py --port /dev/cu.usbserial-XXXX --echo
python tools/serial_tool/jog_check.py --port /dev/cu.usbserial-XXXX --tests 1,2,5 --skip-home
```

| test | 内容 |
|---|---|
| 8 | 起動直後(HOMED=NO)の`JOG 50 0` → `NACK_JOG reason=not_homed` |
| 1 | `JOG 50 0`を2秒 → `JOG 0 0` → X約+100mm、`reason=zero HOMED=YES` |
| 2 | `JOG -50 0`を1秒、送信停止 → 最後のJOGから600ms以内に`reason=timeout` |
| 5 / 5b | ジョグ中の`XY 100 100` → `NACK_XY ... reason=jog_active`、後から実行されない。ジョグ中の`POS`が`MOTION=YES`で即答 |
| 3 | X=380から`JOG 100 0`を1.5秒 → `JOG_POS`のXが389〜390 |
| 6 | `XY 20 y 3000`の途中で`STOP` → `NACK_XY reason=stopped`、`STOP complete HOMED=YES`、`ALARM=NO`、続く`XY`が通る |
| 7 | 20HzのJOGを10秒+`POS` 2Hz → `JOG_POS`約200行、POS全応答、その後の`ACK_XY`を受信 |
| 11 | `JOG 0 0 400 0`を1秒 → TEST_MODE OFFでZサーボが回る(hold check通過) |
| 4 | ジョグ中に`ABORT` → `reason=abort`、`ALARM=YES`、`HOMED=NO`(最後に`ALARM_CLEAR`) |

## Serial CSV(動かない確認)

`examples/jog_check.csv`: CONFIGのJOG設定、HELP、待機中の`STOP`がアラームにならないこと。

```bash
python tools/serial_tool/serial_send.py --port /dev/cu.usbserial-XXXX \
  --csv tools/serial_tool/examples/jog_check.csv --startup-delay 4 --echo
```

## ジョイスティック

Atom JoyStickのジョグも同じセッションを使う(TEST_MODE不要、HOMEDなら保持してソフトリミット手前で停止、
ジョグ中のSerialコマンドで終わらない)。手順は[joystick-check.md](joystick-check.md)。
