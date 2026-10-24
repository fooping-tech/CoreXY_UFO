# 返信: HANDOFF_FIRMWARE_JOG（シリアル速度ジョグ `JOG` と非アラーム停止 `STOP`）

作成日: 2026-10-24
送り元: `CoreXY_UFO`（M5Stack Core2 ファームウェア）
宛先: `CoreXY_UFO_Game` のバックエンド／ゲーム担当
対象依頼: [HANDOFF_FIRMWARE_JOG.md](https://github.com/fooping-tech/CoreXY_UFO_Game/blob/main/docs/HANDOFF_FIRMWARE_JOG.md)（調査時 HEAD `160742d`）
ファームウェア側の仕様: `SPEC.md`「2026-10-24: Serial速度ジョグ`JOG`と非アラーム停止`STOP`」、手順: `tools/serial_tool/docs/jog-check.md`

## 1. 状況

| 依頼 | 状態 |
|---|---|
| §3.0 既存ジョグの制約撤廃（TEST_MODE 不要、HOMED 保持、ソフトリミット、シリアルで終わらない） | 実装済み（ジョイスティックとシリアル共通） |
| §3.1 `JOG` | 実装済み |
| §3.2 `STOP` | 実装済み |
| §3.3 `JOG_POS` | 実装済み（ジョグ中のみ。`REPORT ON` は未実装） |
| §4 受け入れ条件 | ビルド・書き込み・native テスト完了。実機では SELFTEST と #8（`NACK_JOG reason=not_homed`）を確認。#1〜7、#11 はモータ電源（VM）が入っておらず HOME できなかったため未確認。確認スクリプト `tools/serial_tool/jog_check.py` を用意した |
| 前回 HANDOFF_FIRMWARE.md §4.1〜§4.4 | 今回は未着手 |

## 2. 応答行（`firmware_protocol.md` §2 の書式）

| コマンド | 応答 |
|---|---|
| `JOG vx vy [z arm]`（開始） | `JOG start vx=50.0 vy=0.0 z=0 arm=0 src=SERIAL HOMED=YES` |
| `JOG ...`（継続中） | **何も返さない**（`ACK QUEUED` も返さない） |
| ジョグ中の位置（50 ms ごと） | `JOG_POS X=123.456 Y=100.000 VX=50.0 VY=0.0 Z_ABS=4731 ARM_ABS=NA` |
| ジョグ終了 | `JOG end reason=zero X=200.123 Y=100.000 HOMED=YES src=SERIAL` |
| ソフトリミットに到達（継続） | `JOG soft_limit axis=X X=389.962` |
| `JOG` の拒否 | `NACK_JOG reason=not_homed` / `alarm` / `homing` / `job_active` / `busy` / `bad_args (...)` |
| ジョグ中の移動・設定系コマンド | `ACK QUEUED XY` → `NACK_XY target=(100.000,100.000) reason=jog_active`、その他は `ACK QUEUED HOME` → `NACK_HOME reason=jog_active` |
| ジョグ中の `POS` | `ACK QUEUED POS` → `POS X=.. Y=.. ... HOMED=YES ... MOTION=YES ...`（ジョグは続く） |
| `STOP` | `ACK QUEUED STOP` → （止めた `XY`）`NACK_XY target=(20.000,100.000) reason=stopped X=187.250 Y=100.000` → `STOP complete X=187.250 Y=100.000 HOMED=YES` |
| `STOP` 時にキューにあった移動 | `NACK_XY target=(..) reason=stopped`（X/Y なし）、その他 `NACK_<CMD> reason=stopped` |

補足:

- `JOG end` の `reason`: `zero`（`JOG 0 0`）、`timeout`（250 ms 途絶）、`command`（`STOP`）、`abort`、`alarm`、`limit_x`、`limit_y`、`idle`／`link_lost`（ジョイスティックのみ）。依頼の `soft_limit` は使わない（ソフトリミットでは**終わらず速度 0 で継続**し、`JOG soft_limit` 行を出す）。
- 末尾の `src=` と `HOMED=` の位置以外は依頼の書式どおり。フィールドは今後も末尾に追加するだけにする。
- `Z_ABS` / `ARM_ABS` はサーボの多回転追跡が無効なとき `NA`（数値でない）。`SERVO_STATUS` の `abs=` と同じ値。
- `JOG_POS` の `VX`/`VY` は前の行からの位置変化で求めた実速度 [mm/s]。ジョイスティックのジョグでも出る。
- 既存の行の先頭トークンは変えていない（§6 の約束どおり）。旧 `JOY_JOG start/end` は `JOG start/end` にまとめた（ゲーム側では未使用のはず）。

## 3. 振る舞い（バックエンド実装に影響する点）

1. **拒否は押下ごとに 1 回だけ。** `NACK_JOG` を出したあと、その押下の `JOG` は黙って捨てる。`JOG 0 0` を送るか、250 ms 以上送信を止める（キーを離す）と次の押下として扱う。`command`／`alarm`／`limit_*` でセッションが終わった後も同じ。押しっぱなしのキーで、`XY` や `HOME` の完了後に勝手に動き出さないようにするため。
2. **`busy`**: `XY` の実行中に `JOG` を送ると `NACK_JOG reason=busy`（ジョブ中は `job_active`、`HOME` 中は `homing`）。
3. **シリアルの XY ジョグは HOMED が必要。** Z/アームだけの `JOG 0 0 <z> <arm>` は HOMED なしでも動く。HOMED でないセッション中に XY を送ると `NACK_JOG reason=not_homed (XY ignored in this session)` を 1 回出し、XY だけ無視する。
4. **速度上限**: 合成速度を `MAX_FEED_MM_MIN/60` で切る。加減速は `SERIAL_JOG_ACCEL_MM_S2` = 600 mm/s²。50 mm/s からの停止距離は約 2 mm。
5. **ソフトリミット**: HOMED で始めたジョグは X 0..390 / Y 0..200 の約 0.04 mm 手前で止まる（最悪減速度モデルでの native テスト）。境界に近づくと速度が落ちる。内向きにはすぐ動ける。
6. **`STOP`**:
   - `XY` は減速して止まる。backend のキューに入っている分（数十〜百 ms 程度）を実行してから減速を始めるので、即時ではない。停止位置はステップ数から求めるので正確。そのまま次の `XY` を送ってよい。
   - `STOP` より前に送った移動系コマンドは捨てる（`NACK_... reason=stopped`）。**`STOP complete` を受けてから**次の移動を送ること。`STOP` とほぼ同時に送った移動も捨てられることがある。
   - ジョブ中ならジョブは `ABORTED`（`JOB_STATUS ... last_error=stopped`）になる。アラームにはならない。G-code を流している場合は、送信を止めてから `STOP` する。
   - `HOME`、`SERVO_TURN`/`SERVO_ANGLE` は `STOP` では止まらない（`ABORT` を使う）。
7. **ジョイスティック**: TEST_MODE なしで動く。シリアルの `JOG` が来るとそのセッションはシリアルに切り替わり、ジョイスティックの入力を無視する。ゲーム中にジョイスティックを使わないなら、ファームウェアの `JOYSTICK_ENABLED=false` で無効にできる。

## 4. §7 への回答

1. **HOMED のまま速度ジョグをしてよいか**: よい。位置は backend が実際に出した STEP パルス数から計算しており、`XY` 移動と同じオープンループの精度（脱調しなければ正確）。ジョグの加減速 600 mm/s² は通常の `XY`（1000 mm/s²）より低い。高速での脱調は未確認なので、ゲームでは通常 3000 mm/min（50 mm/s）以下を推奨する。ジョイスティックの ULTRA（7500 mm/min）は実機で確認した高速範囲（5000 mm/min）を超えている。
2. **`STOP` 後の位置は正確か**: 正確。停止位置は計画値ではなくステップ数から求め、プランナーのキューと segment のキューは破棄する。次の `XY` は実際の位置から計画される。
3. **TEST_MODE が担っていた安全上の役割**: サーボのトルク・保持チェック（200 ms の hold check、待機中のずれ検出、runaway guard、読出し欠落での停止）は、もともとジョグ開始時の `beginJog` とジョグ中の `updateJog` で行っており、TEST_MODE とは独立だった。そのため代わりの仕組みは要らず、撤廃した。「ジョイスティックを本番で使わない」前提は、ジョブ中は拒否、シリアル `JOG` 優先、ジョグ中のシリアル移動コマンドは拒否、の 3 点に置き換えた。
4. **ジョグ中に読み取り系へ即答する構成**: motionTask（Core 1）のジョグループ（20 ms 周期）の中で処理する。`machine_state` は motionTask だけが読み書きするので排他は不要。応答の遅れは最大約 20 ms（`CONFIG` などの長い出力は除く）。`TMC_STATUS` は基本の状態行だけを返し、多レジスタ診断は省略する（`TMC_STATUS diagnostics skipped reason=jog_active`）。
5. **同時入力の優先順位**: 提案どおりシリアル優先。

## 5. `FakeFirmware` の更新案

`backend/tests/conftest.py` に次の振る舞いを足すと実機に合う（ゲーム側での実装を想定。PR が必要なら依頼してほしい）。`__init__` に `self.jogging = False` を追加する。

```python
elif name == "JOG":
    parts = line.split()
    vx, vy = float(parts[1]), float(parts[2])
    zero = all(float(p) == 0 for p in parts[1:])
    if self.jogging:
        if zero:
            self.jogging = False
            self._emit(f"JOG end reason=zero X={self.x:.3f} Y={self.y:.3f} HOMED={'YES' if self.homed else 'NO'} src=SERIAL")
        # continuing JOG: no reply
    elif zero:
        pass
    elif self.alarm:
        self._emit("NACK_JOG reason=alarm")
    elif (vx or vy) and not self.homed:
        self._emit("NACK_JOG reason=not_homed")
    else:
        self.jogging = True
        self._emit(f"JOG start vx={vx:.1f} vy={vy:.1f} z=0 arm=0 src=SERIAL HOMED={'YES' if self.homed else 'NO'}")
elif name == "STOP":
    self._emit("ACK QUEUED STOP")
    if self.jogging:
        self.jogging = False
        self._emit(f"JOG end reason=command X={self.x:.3f} Y={self.y:.3f} HOMED={'YES' if self.homed else 'NO'} src=SERIAL")
    self._emit(f"STOP complete X={self.x:.3f} Y={self.y:.3f} HOMED={'YES' if self.homed else 'NO'}")
```

ジョグ中の `XY` は `ACK QUEUED XY` → `NACK_XY target=(x,y) reason=jog_active`、`POS` は `MOTION=YES` で返す。

## 6. 実機確認の予定

2026-10-24: SELFTEST PASS、待機中の `STOP` で `STOP complete`（アラームなし）、#8 PASS。

`python tools/serial_tool/jog_check.py --port <port> --echo` で §4 の 1〜8、11 を自動確認する（HOME を含み実機が動く）。9、10（ジョイスティック）は手動。結果はこのファイルと `PLANS.md` に追記する。
