# Home Fast Approach Check

HOMEDのまま`HOME`したときに、原点の手前(X・Yそれぞれ原点から20 mm)まで通常の移動で動いてから原点探しをすること(Issue #20)を実機で確認するチェックです。
HOMEDでない最初の`HOME`は今までどおり(手前への移動なし)であることも確かめます。

## Safety

- [Homing Check](homing-check.md)が通っている機体で実行してください(limit方向・極性が正しいこと)。
- 手前への移動は100 mm/s(`HOMING_FAST_APPROACH_FEED_MM_MIN` 6000 mm/min)です。可動範囲に物がないこと、E-stopまたはモータ電源をすぐ切れることを確かめてください。
- CSVは最初に`ZERO`でHOMEDを破棄します。脱調の疑いがあるときも同じく`ZERO`してから`HOME`します。

## CSV

`tools/serial_tool/examples/home_fast_approach_check.csv`

## Run

```bash
python tools/serial_tool/serial_send.py \
  --port /dev/cu.usbserial-023591AC \
  --csv tools/serial_tool/examples/home_fast_approach_check.csv \
  --startup-delay 4 \
  --timeout 95 \
  --echo
```

送信予定だけを確認する場合:

```bash
python tools/serial_tool/serial_send.py \
  --csv tools/serial_tool/examples/home_fast_approach_check.csv \
  --dry-run
```

## Expected Logs

| Step | What to confirm |
|---|---|
| `CONFIG` | `HOMING fast_approach=1 margin=20.000 feed=6000.000` |
| 最初の`HOME`(`ZERO`のあと) | `HOME fast approach skipped: not homed`、そのあと今までどおり`HOME_X`→`HOME_Y`→`HOME complete` |
| 中央からの`HOME` | `HOME fast approach from=(195.000,100.000) to=(20.000,20.000)` → `HOME fast approach complete` → `HOME_X started`…`HOME complete`。かかった時間(以前は18.6 s、見込み7〜8 s) |
| `HOME_X set zero` / `HOME_Y set zero` | `before_zero`がほぼ0(手前への速い移動で脱調していない) |
| X=10からの`HOME` | `to=(10.000,20.000)`: 原点から遠ざかる向きには動かない |

## Failure

| Log | Meaning |
|---|---|
| `HOMING error ... reason=homing fast approach hit a limit (position off)` | 手前への移動中にlimitが入った。HOMEDの位置がずれていた(脱調)。HOMEDは外れる。`ZERO` → `ALARM_CLEAR` → `HOME`で戻す |
| `before_zero`が大きい | 速い移動で脱調した可能性。`HOMING_FAST_APPROACH_FEED_MM_MIN`を下げる |
