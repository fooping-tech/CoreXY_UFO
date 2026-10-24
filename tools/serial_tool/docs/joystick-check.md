# Atom JoyStick Check

Atom JoyStick(v1.0、AtomS3)から、ESP-NOWでXY・Z(サーボID1)・アーム(サーボID2)を
連続ジョグする簡易操作の確認手順です。本番の指示はPCからUSBシリアルで行います。
2026-10-24からTEST_MODEは不要(Serial `JOG`と共通のセッション。[jog-check.md](jog-check.md))。

## 書き込み

```bash
pio run -e atoms3-joystick -t upload      # Atom JoyStick (USB: /dev/cu.usbmodem*)
pio run -e m5stack-core2 -t upload        # Core2
```

Atomは**スティックを離した状態で電源ON**(中央値の校正)。LCDに`CENTER? reboot`が出たら再起動する。

## 操作

| 操作 | 動作 |
|---|---|
| 左スティック | XY(倒している間スムーズに移動、離すと減速停止) |
| 右スティック 上下 | Z(サーボID1) |
| 右スティック 左右 | アーム(サーボID2) |
| 右ボタン | 非常停止(ABORT。ALARMになりHOMED解除) |
| 左ボタン 1秒長押し | テストモード ON / OFF(ジョグには不要。TEST_JOG/SERVO_TURN等用) |
| 画面ボタン 1秒長押し | ALARM_CLEAR |
| 左スティック押し込み | 速度切替 SLOW(600)→FAST(3000)→VERY FAST(4500)→ULTRA(7500 mm/min)→SLOW。起動時はFAST |

AtomのLCD: Link、TEST、ALARM、JOG、SPD(速度段階、VERY FAST以上は橙)、電池、各軸の値(-1000..1000)、直前のイベント。

## 安全仕様

- TEST_MODE不要。ジョブ中・alarm中・homing中は無視(`JOY_JOG ignored reason=...`)
- HOMEDで始めたジョグはHOMEDを保ち、X 0..390 / Y 0..200の手前で減速して止まる(`JOG soft_limit axis=..`)。HOMEDでなければ従来どおりHOMED=NOのまま(復旧用)
- スティックを離す、リンクが250ms途絶える、STOP/ABORTで減速停止。サーボは現在位置で保持
- ジョグ中のSerialコマンドではジョグは終わらない。`POS`等の読み取りは即答、移動・設定系は`NACK_<CMD> reason=jog_active`
- Serialの`JOG`が来るとそちらが優先され、そのセッション中はジョイスティック入力を無視する
- リミットが新たにONになったら即停止。ONのリミットへ向かう成分は0
- サーボはhold check、runaway guard、読出し欠落で停止。XY移動中はサーボのジョグを開始しない
- Core2は起動後に最初に受信したAtomのMACに固定する(`JOY_ALLOWED_MAC`で固定指定も可)

## 確認手順

1. Core2の`CONFIG`で`JOY enabled=1 ready=1 core2_mac=... atom_mac=...`、`POS`で`JOY=LINK`
2. テストモードOFFのまま左スティックを倒す → 動く(`JOG start ... src=JOY`)
3. `HOME`後にジョグ → 終了後も`HOMED=YES`。X 390 / Y 200へ向けて倒し続けると手前で止まる
4. 各軸の向きを確認(LCDの値: 右/上で+になるか。逆なら`AtomJoyMain.cpp`の`AXIS_SIGN`)
5. 左スティック: XYが倒している間動き、離すと止まる(`JOG start` / `JOG end reason=idle`)
6. XY移動中にXスイッチを手で押す → 即停止(`reason=limit_x`)。押したまま-X → 動かない
7. 右スティック上下でZ、左右でアームが回り、離すと保持する(回転方向が逆なら`JOY_Z_SIGN`/`JOY_ARM_SIGN`)
8. ジョグ中にAtomの電源を切る → 250ms以内に停止(`reason=link_lost`)
9. ジョグ中に右ボタン → 停止、`ALARM`。画面ボタン長押しで解除
10. ジョグ中にSerialから`POS`を10Hzで送る → ジョグは続き、`POS ... MOTION=YES`が毎回返る
11. ジョグ中にSerialから`STOP` → 減速停止(`reason=command`)、`STOP complete`、アラームなし

## Serial CSV

`tools/serial_tool/examples/joystick_check.csv`は、CONFIG/POSのJOY表示の確認用。
