# 返信: HANDOFF_FIRMWARE_Z_SERVO（サーボだけ電源が切れたあとの JOG）

作成日: 2026-11-08
送り元: `CoreXY_UFO`（M5Stack Core2 ファームウェア）
宛先: `CoreXY_UFO_Game` のバックエンド／ゲーム担当
対象依頼: [HANDOFF_FIRMWARE_Z_SERVO.md](https://github.com/fooping-tech/CoreXY_UFO_Game/blob/fix/62-z-servo-reset/docs/HANDOFF_FIRMWARE_Z_SERVO.md)（ゲーム側 Issue #62、調査時 HEAD `bf70657`）
ファームウェア側: Issue #11、`SPEC.md`「2026-11-08: サーボだけ電源が切れたあとの多回転の検知と再設定」、手順: `tools/serial_tool/docs/servo-power-cycle-check.md`

調査時 HEAD `bf70657` は PR #10（開発ワークフロー、未マージ）のブランチです。サーボまわりのコードは main（`a49bc54`）と同じで、§2「現状」の記述はすべてソースと一致しました。

## 1. 状況

| 依頼 | 状態 |
|---|---|
| §3.1 `beginJog()` で制限を確かめる | 実装済み。**(a) を採用**: その場で多回転に戻して続け、`tracking=RESET` を付ける |
| §3.2 状態報告の矛盾をなくす | 実装済み: `SERVO_STATUS` が `tracking=LOST` を出す（状態は変えない） |
| §3.3 爪（ID 2）の JOG | §3.1 で一緒に直った（同じ `beginJog()`） |
| §3.3 `moveToAngle()`（`SERVO_ANGLE`） | 変更不要。`moveRelative()` を通るので、動かす前に同じ確認で `multi_turn_lost (retry)` になる |
| §5 受け入れ確認（実機） | **未実施**（§6）。ビルドとネイティブテストのみ通過 |

依頼にない危険も1つ直しました。今の `beginJog()` は、制限が戻っていても保持確認（`holdAndVerify`）で古い `abs` を位置の指令として書きます。1回転モードのサーボは 0〜4095 の外の指令を端の値に丸めるので、`abs` がその範囲の外にあると**端まで回り**、保持確認がずれを検出して**トルクを切ります**。荷重のある Z ドラムは落ちる可能性があります。今回の変更で、制限の確認は保持確認より前に行われます。

## 2. 応答行（`firmware_protocol.md` §2 の書式）

| コマンド | 応答 |
|---|---|
| `SERVO_STATUS id`（多回転が失われている） | `SERVO_STATUS id=1 result=OK flags=0x00 mode=0 torque=0 raw=2048 goal=0 limits=0/4095 tracking=LOST abs=-8240 turns=-2.01 voltage=7.6 temp=24 moving=0` |
| `SERVO_STATUS id`（通常） | `... limits=0/0 tracking=YES abs=..`（未追跡は `tracking=NO`。従来どおり） |
| `JOG ...`（サーボの開始） | `JOG servo id=1 begin_ms=230 (hold check; XY waits) tracking=KEPT` |
| 同上（未追跡から開始） | `JOG servo id=1 begin_ms=240 (hold check; XY waits) tracking=NEW` |
| 同上（多回転を設定し直した） | `JOG servo id=1 begin_ms=245 (hold check; XY waits) tracking=RESET` |
| 同上（設定し直しに失敗） | `JOG servo id=1 disabled reason=goal_mismatch goal=1200 raw=2048 (power-cycle servo) multi_turn_lost=YES` |
| `SERVO_TURN` / `SERVO_ANGLE`（多回転が失われている） | 従来どおり `SERVO_TURN id=1 value=0.01 result=REJECTED reason=multi_turn_lost (retry)` |

- 先頭トークンは変えていません。`JOG servo ... begin_ms=` の行は末尾に `tracking=` が付きました。
- `SERVO_STATUS` の `tracking=` が取りうる値は `YES|NO|LOST` になりました。

## 3. 振る舞い（バックエンド実装に影響する点）

- **`tracking=LOST` の意味**: ファームウェアは多回転で追跡しているのに、サーボは1回転モード（`limits≠0/0`）に戻っています。`abs`/`turns`、および JOG 中の `JOG_POS Z_ABS` は電源断より前の古い値です。`SERVO_STATUS` を読んでも状態は変わらず、次の JOG か `SERVO_TURN` まで `LOST` が続きます。
- **`tracking=RESET` の意味**: `abs` は今の1回転内の位置（0〜4095）から振り直されています。**Z の記録点（top/bottom）や爪の 0° 基準は使えなくなる**ので、捨ててください。JOG の動きは押した分だけで、記録点の情報は使いません。
- **`tracking=NEW`**: 起動後の初回など、追跡していなかったサーボで追跡を始めた。ESP32 だけ再起動した場合は、サーボの多回転の位置から `abs` を引き継ぎます（従来どおり）。
- JOG が多回転を設定し直すのは、**JOG の開始時（`beginJog()`）だけ**です。ジョグ中にサーボの電源が切れた場合は、従来どおり位置が読めなくなって `JOG servo id=N stopped reason=feedback_lost tracking=INVALID` になります。
- 設定し直しには、「電源投入直後の状態か」の判定（goal=0 かつ torque=0、または goal≈raw）を使います。電源を切っている間にドラムを手で回した場合や、旧ファームウェアで JOG を試したあとでも、通常は通ります。失敗した場合は `disabled ... multi_turn_lost=YES` になり、そのセッションではサーボを動かしません。
- `SERVO_TURN` は変えていません。ゲーム側の `z_servo_recover`（`SERVO_TURN 1 0.01` で1回拒否させる）もそのまま動きますが、今後は不要です。

## 4. 判断への回答

1. **(a) と (b) のどちらにするか**: 推奨どおり (a) にしました。JOG は押している間の相対移動で、電源の入れ直しも起動直後の初回 JOG と同じ確認（電源投入状態の判定 → 保持確認）を通るので、安全性は起動直後と変わりません。
2. **状態報告（§3.2）**: `tracking=LOST` を出す方にしました。読むだけで状態を変えない方式で、`SERVO_TURN` の従来の流れ（1回拒否）が変わらないためです。**ゲーム側の暫定検知（`tracking=YES` かつ `limits≠0/0`）は、このファームウェアでは反応しなくなります**。§5 のとおり `LOST` を見るように変えてください。
3. **`moveToAngle()`**: `moveRelative()` を呼ぶので、追跡中に制限が戻っていれば、動かす前に `multi_turn_lost (retry)` で拒否します。未追跡のときは1回転内の最短経路で動かします（従来どおり）。

## 5. `FakeFirmware` と `protocol.py` の更新案

`protocol.py`（`parse_servo_status`）:

```python
tracking = rest.get("tracking", "NO")
st.tracking = tracking in ("YES", "LOST")
st.tracking_lost = tracking == "LOST"   # ServoStatus に field を追加

@property
def multi_turn_lost(self) -> bool:
    # 新ファームウェアは LOST を出す。旧ファームウェア（YES + limits≠0/0）にも対応
    return self.tracking_lost or (
        self.tracking and self.limits is not None and self.limits != (0, 0))
```

`JOG servo` 行（Z の基準を捨てる判定）:

```python
_JOG_SERVO_BEGIN_RE = re.compile(
    r"^JOG servo id=(?P<id>\d+) begin_ms=\d+.*?(?: tracking=(?P<tracking>KEPT|NEW|RESET))?$")
# tracking == "RESET" なら、その id の記録点（Z top/bottom、爪 0°）を捨てる
```

`FakeFirmware`（`conftest.py`）:

```python
# SERVO_STATUS
lost = self.servo_tracking[sid] and self.servo_limits[sid] != "0/0"
tracking = "LOST" if lost else ("YES" if self.servo_tracking[sid] else "NO")

# JOG 開始時（サーボ成分があるとき、JOG start の後）
for sid, rate in ((1, z), (2, arm)):
    if not rate:
        continue
    if self.servo_tracking[sid] and self.servo_limits[sid] != "0/0":
        state = "RESET"
    elif self.servo_tracking[sid]:
        state = "KEPT"
    else:
        state = "NEW"
    self.servo_tracking[sid], self.servo_limits[sid] = True, "0/0"
    if state == "RESET":
        self.servo_abs[sid] = 100  # raw から振り直し
    self._emit(f"JOG servo id={sid} begin_ms=230 (hold check; XY waits) tracking={state}")
```

## 6. 実機確認

- [x] ビルド（`m5stack-core2` / `-sim` / `-hwtest`）、ネイティブテスト
- [ ] 書き込み・`SELFTEST`
- [ ] `tools/serial_tool/servo_power_cycle_check.py --id 1`: Z を下げる → サーボだけ電源断 → `tracking=LOST` → JOG で `tracking=RESET` → もう一度 JOG で `tracking=KEPT` かつ `abs` が JOG の分だけ変わる
- [ ] 同 `--id 2`（爪）、`--servo-turn`（`SERVO_TURN` の1回拒否 → 再試行で PASS）
- [ ] ジョイスティックの Z/アームでも、入れ直し後の最初の押下で動く

結果はこのファイルと `PLANS.md` に追記します。
