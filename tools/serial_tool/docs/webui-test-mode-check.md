# WebUI Test Mode Check

ブラウザアプリ(Host WebUI)の`Test`タブから、ホーミングなしでリミット確認・モータ単独移動・X/Y移動・
シリアルサーボ(ID 1 = Z軸ドラム、ID 2 = アーム)の角度指定と回転を行う確認手順です。
通常ファームウェア(`m5stack-core2`)に組み込んだSerialコマンドを使います。

## 安全条件

- 初回はサーボのホーン・負荷なし、XYは可動範囲の中央付近から始める
- サーボは外部電源(約8V、EEPROM上限10.0V以下)、バスサーボアダプタ経由、GND共通
- `STOP`はABORTを送る。物理的な非常停止の代わりにはならない
- テストモードで動かすと`HOMED`は解除される。ジョブ前に`TEST_MODE OFF`→`HOME`

## 起動

```bash
pio run -e m5stack-core2 -t upload
python tools/webui/server.py
```

ブラウザで`http://127.0.0.1:8787`を開き、`Settings`でポートに接続して`Test`タブを開く。

## Test タブ

| 部分 | 内容 |
|---|---|
| `ENTER / EXIT TEST MODE` | `TEST_MODE ON/OFF`。ジョブ中・homing中は入れない |
| `STOP` | 実行中のテストコマンドを中断して`ABORT`を送る。その後`CLEAR ALARM` |
| Limits | `POS`を1.5秒ごとに取得し、X/Yのdebounced値(ON/OFF)とraw入力を表示 |
| Motors | `TEST_A/TEST_B <±80/400/1600> 600`。A+ = +X+Y、B+ = +X-Y |
| X / Y Move | `TEST_JOG <dx> <dy> 600`。1/5/10/50mm。-X/-Yがリミット方向 |
| Serial Servo ID1/ID2 | `SERVO_STATUS`、`SERVO_ANGLE <id> <deg>`(最短経路)、`SERVO_TURN <id> <turns>`(±3回転まで、+が時計回り)、`SERVO_OFF` |

動作ボタンはテストモードON・接続中・他のテストコマンド非実行中・alarmなしのときだけ有効。

## ファームウェア側の安全機構

| 対象 | 内容 |
|---|---|
| `TEST_JOG` / `TEST_A` / `TEST_B` | 開始時OFFのリミットがON(rawまたはdebounced)になった時点で即停止し`result=STOPPED_LIMIT`。ONのリミットへ向かう移動は`REJECTED` |
| `TEST_JOG` | テストモード必須、1回最大50mm、feed 0〜1200mm/min、TMC ready必須 |
| `SERVO_ANGLE` / `SERVO_TURN` | テストモード必須。初回は角度制限0/0(EEPROM保存なし)にして多回転追跡を開始。毎回hold check(200ms ±10 counts)とrunaway guard(50 counts)。異常時は即torque OFF |
| 多回転追跡の開始条件 | 角度制限0/4095かつ「goal=現在位置」または「電源投入状態(goal=0、torque OFF)」。角度制限0/0の場合は静止保持中でgoalと一致するときだけ復帰。それ以外は`reason=... (power-cycle servo)` |
| `PENUP` / `PENDOWN` | ペン用サーボ未割当のためERROR。G-codeのM3/M5はサーボを動かさずスキップ |
| `JOB_BEGIN` | テストモード中は`REJECT` |

## 手順

1. `Test`タブを開き、Limitsが`OFF / raw OPEN`で表示されることを確認
2. テストモードOFFのまま、ボタンが無効であることを確認
3. `ENTER TEST MODE` → 状態が`ON`
4. X/Yスイッチを手で押し、該当インジケータが`ON`(赤)になることを確認
5. X / Y Move: `1mm`で`+X`、`-X`、`+Y`、`-Y`。最下段の結果が`TEST_JOG result=DONE`
6. `-X`を押しながら途中でXスイッチを手で押す → `result=STOPPED_LIMIT limit=X`
7. Xスイッチを押したまま`-X` → `result=REJECTED reason=x_limit_active_toward_limit`
8. Motors: `80`で`A+`、`B+`、`A-`、`B-`の方向を確認
9. Servo ID1: `STATUS` → `limits=0/4095`、`+1/4` → `SERVO_TURN ... result=PASS`、`-1/4`で戻る
10. Servo ID1: `Turns 3`で`ROTATE`(右3回転、約6s)、`-3`で戻る
11. Servo ID2: `Angle 180`で`MOVE TO ANGLE` → `SERVO_ANGLE ... result=PASS`
12. `STOP`を移動中に押す → 停止、`ALARM`。`CLEAR ALARM`で復帰
13. `EXIT TEST MODE`、`Control`で`HOME`

## Serial CSV

Test タブを使わずに同じコマンドをSerialで確認する場合:

```bash
python tools/serial_tool/serial_send.py \
  --port /dev/cu.usbserial-XXXX \
  --csv tools/serial_tool/examples/webui_test_mode_check.csv \
  --startup-delay 4 \
  --timeout 40 \
  --queue-mode \
  --echo
```

CSVはXYを1mm、モータを80 steps、サーボを1/4回転動かす。実行前にXY可動部とサーボの周囲を確認すること。
