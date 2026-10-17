# Test Mode Check (LCD テストモード)

通常ファームウェアに組み込んだLCDテストモードの確認手順です。
PCとSerialが無くても、Core2の画面だけで方向確認・リミット確認・TMC診断ができます。

## 入り方 / 抜け方

| 操作 | 動作 |
|---|---|
| 画面下の**中央ボタン(BtnB)を1.5秒長押し** | テストモードへ入る(どのページからでも可) |
| もう一度BtnB長押し、または下部タブ`EXIT` | 通常画面へ戻る |
| 左ボタン(BtnA) / 下部タブ`MOTOR` | MOTORページ |
| 右ボタン(BtnC) / 下部タブ`DIAG` | DIAGページ |

- モーション中、homing中、ジョブ実行中はテストモードへ入れない(`Busy`表示)。
- テストモード中にジョブが始まった場合は自動で通常画面へ戻る。
- 長押し時間は`include/PlotterConfig.h`の`TEST_MODE_HOLD_MS`。

## MOTORページ

| 表示/ボタン | 内容 |
|---|---|
| 上部帯 | `LIMIT X/Y`のON/OFFと`HOME`状態。どちらかONなら赤 |
| `A+ (+X +Y)` / `A- (-X -Y)` | Aモータだけを動かす。CoreXYでは対角へ動く |
| `B+ (+X -Y)` / `B- (-X +Y)` | Bモータだけを動かす |
| `STEP` | 移動量を 80 / 400 / 1600 steps で切り替え(モータ1 / 5 / 20mm) |
| `STOP` | 実行中のテスト移動を中断(Serial `ABORT`と同じ) |
| 最下行 | 最後のテスト結果 `TEST_A +400 DONE` / `STOPPED` / `REJECT` |

- 速度は`TEST_MODE_MOTOR_FEED_MM_MIN`(600mm/min相当、800 steps/s)。
- **モータテストを行うとHOMEDは解除される**。描画前に`HOME`をやり直す。
- alarm中はモータボタンが無効。CTRLページの`CLEAR ALARM`で解除する。

### 方向の判定

座標は「+X = Xスイッチから離れる、+Y = Yスイッチから離れる」。

| ボタン | 正しい動き |
|---|---|
| `A+` | X・Yともスイッチから離れる |
| `B+` | Xはスイッチから離れ、Yはスイッチ側 |

ずれる場合は[Direction Check](direction-check.md)の判定表で設定を直す。

## DIAGページ

| 表示/ボタン | 内容 |
|---|---|
| `LIMIT X GPIO36` / `LIMIT Y GPIO35` | debounced状態を大きく表示、`raw`は生入力。約100ms周期で更新 |
| `SELFTEST` | CoreXY変換の自己診断。結果は`SELFTEST: PASS/FAIL` |
| `TMC INIT` | TMC2209を通常profileで初期化。成功で`TMC profile: READY` |
| `TMC STATUS` | TMC2209 A/BをUARTで低頻度診断し、下段パネルへ表示 |
| `TMC A/B`パネル | `UART OK ver 0x21`、`DRV OK`。異常時は`UART NG (code n)`、`OT`/`SHORT`/`DRV_ERR`(赤)、`OTPW`/`UV`(黄) |

- `UV`(GSTAT uv_cp)はVM未接続や電源投入直後に立つことがある。VM接続後に`TMC INIT`→`TMC STATUS`で再確認する。
- 診断はモーション中には実行しない(ボタン無効)。

## 実機手順

1. 通常ファームウェアを書き込み、`SELFTEST`がSerialでPASSすることを確認。
2. INFOページ下部に`Hold center button: TEST MODE`が出ていることを確認。
3. BtnBを長押ししてテストモードへ入る。
4. DIAGページ: `SELFTEST` → `PASS`、`TMC INIT` → `READY`、`TMC STATUS` → A/Bとも`UART OK ver 0x21` `DRV OK`。
5. DIAGページ: X/Yスイッチを手で押し、該当インジケータがONになり、離すとOFFに戻ることを確認。
6. 可動部を作業範囲中央へ移し、MOTORページ: `STEP 400`で`A+`、`B+`、`A-`、`B-`の方向を確認。
7. `STEP 1600`で`A+`中に`STOP`を押し、途中で止まり最下行が`STOPPED`になることを確認。
8. `EXIT`で通常画面へ戻り、CTRLページで`HOME`をやり直す。

## Serial併用CSV

`tools/serial_tool/examples/test_mode_check.csv`はDIAGページの表示とSerialログ(`TMC_DIAG`、`LIMIT_STATUS`)を突き合わせるためのもの。

```bash
python tools/serial_tool/serial_send.py \
  --port /dev/cu.usbserial-XXXX \
  --csv tools/serial_tool/examples/test_mode_check.csv \
  --startup-delay 4 \
  --timeout 10 \
  --echo
```

`TMC_STATUS`は従来の`TMC_STATUS`/`TMC_A`/`TMC_B`行に加え、`TMC_DIAG A ... result=OK|NG`と`TMC_DIAG B ...`を出力する。
