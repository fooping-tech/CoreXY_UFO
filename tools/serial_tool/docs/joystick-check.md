# Atom JoyStick Check

Atom JoyStick(v1.0、AtomS3)から、ESP-NOWでテストモードのXY・Z(サーボID1)・アーム(サーボID2)を
連続ジョグする簡易テストの確認手順です。本番の指示はPCからUSBシリアルで行います。

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
| 左ボタン 1秒長押し | テストモード ON / OFF |
| 画面ボタン 1秒長押し | ALARM_CLEAR |
| 左スティック押し込み | 速度切替 SLOW(600)→FAST(3000)→VERY FAST(4500)→ULTRA(7500 mm/min)→SLOW。起動時はFAST |

AtomのLCD: Link、TEST、ALARM、JOG、SPD(速度段階、VERY FAST以上は橙)、電池、各軸の値(-1000..1000)、直前のイベント。

## 安全仕様

- テストモード中だけ動く。ジョブ中・alarm中・homing中は無視(`JOY_JOG ignored reason=...`)
- スティックを離す、リンクが250ms途絶える、Serialコマンドが来る、テストモードOFFで減速停止。サーボは現在位置で保持
- リミットが新たにONになったら即停止。ONのリミットへ向かう成分は0
- サーボはhold check、runaway guard、読出し欠落で停止。XY移動中はサーボのジョグを開始しない
- Core2は起動後に最初に受信したAtomのMACに固定する(`JOY_ALLOWED_MAC`で固定指定も可)

## 確認手順

1. Core2の`CONFIG`で`JOY enabled=1 ready=1 core2_mac=... atom_mac=...`、`POS`で`JOY=LINK`
2. テストモードOFFのまま左スティックを倒す → 動かない、`JOY_JOG ignored reason=test_mode_off`
3. 左ボタン長押し → `JOY button: TEST_MODE ON`、AtomのLCDが`TEST: ON`
4. 各軸の向きを確認(LCDの値: 右/上で+になるか。逆なら`AtomJoyMain.cpp`の`AXIS_SIGN`)
5. 左スティック: XYが倒している間動き、離すと止まる(`JOY_JOG start` / `JOY_JOG end reason=idle`)
6. XY移動中にXスイッチを手で押す → 即停止(`reason=limit_x`)。押したまま-X → 動かない
7. 右スティック上下でZ、左右でアームが回り、離すと保持する(回転方向が逆なら`JOY_Z_SIGN`/`JOY_ARM_SIGN`)
8. ジョグ中にAtomの電源を切る → 250ms以内に停止(`reason=link_lost`)
9. ジョグ中に右ボタン → 停止、`ALARM`。画面ボタン長押しで解除
10. ジョグ中にSerialから`POS` → `reason=serial_command`で停止
11. 左ボタン長押しでテストモードOFF → `HOME`してからジョブ

## Serial CSV

`tools/serial_tool/examples/joystick_check.csv`は、CONFIG/POSのJOY表示とテストモード状態の確認用。
