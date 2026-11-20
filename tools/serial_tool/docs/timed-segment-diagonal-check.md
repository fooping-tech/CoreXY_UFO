# Timed Segment Diagonal Check

45° 付近の XY 直線移動で `timed segment queue lost position confidence` が出ないことの確認手順です(Issue #15)。

## 背景

CoreXY では、ほぼ 45° の移動で片方のモーター(A = X+Y、B = X-Y)がほとんど回らない。2026-11-19 のゲーム側ログで、B/A のステップ比 0.018 と 0.046 の移動が、開始の約 1 秒後にこのアラームで止まった。このアラームはキューへの投入の失敗をソフトウェアが検出したもので、脱調の検出ではない。

**原因はまだ特定できていない。** このチェックは、失敗を再現して原因の行(`TIMED_SEGMENT_FAIL`)を採るためのものでもある。

## 見るログ

失敗したとき、アラームの行に続けて次の 2 行が出る。**この 2 行を Issue #15 に貼る。**

```
TIMED_SEGMENT_FAIL stage=b_timeout segment=3 start=1 raw_a=0 raw_b=4 a_steps=120 b_steps=1 ticks=320000 entries_a=6 entries_b=0 running_a=1 running_b=0 waited_ms=1000 b_retries=1000 pause_ticks_a=0 pause_ticks_b=0 skew_ticks=0
TIMED_SEGMENT_STATS segments=3 underruns_a=0 underruns_b=0 pauses_a=0 pauses_b=0 pad_ticks_a=0 pad_ticks_b=0 b_retries=0 max_b_wait_ms=0 skew_ticks=0
```

| フィールド | 意味 |
|---|---|
| `stage` | `b_timeout`: A を積んだ後、B が 1000 ms 断り続けた。`a_append`/`b_append`: `moveTimed()` がエラーを返した。`too_large`/`invalid_segment`: 投入前に断った |
| `raw_a` / `raw_b` | FastAccelStepper `moveTimed()` の最後の戻り値。0 OK、1 QueueFull、2 DirPinIsBusy、4 DeviceNotReady、5/6 方向切替の待機を挿入(移動は未登録)、7 Busy、8 Empty、-4 TooLarge、-5 CommandsSuspended |
| `segment` | その移動で何個目のセグメントか(0 始まり) |
| `entries_a` / `entries_b` | 失敗時のキューのエントリ数(最大 32) |
| `running_a` / `running_b` | 失敗時にキューが動いていたか |
| `waited_ms` / `b_retries` | B を待った時間と再試行の回数 |
| `pause_ticks_*` | この投入で挿入された方向切替の待機 [ticks、16 MHz] |
| `skew_ticks` | A に積んだ時間 − B に積んだ時間 |

移動は完了したが途中でキューが空になった場合は、`WARN: timed segment queue ran dry during the move` と `TIMED_SEGMENT_STATS` が出る(`underruns_a`/`underruns_b` が 1 以上)。到達位置は正しいが、途中の経路は計画した直線からずれている。

## Safety

- **このCSVは実機を最大速度(15900 mm/min)まで動かす。** E-stop またはモーター電源をすぐ切れる状態で実行する。
- CSV内で `HOME` する。開始前に原点方向の移動が安全なことを確かめる。
- 使う範囲は X 10..385 mm、Y 4..175 mm(soft limit は X 0..390、Y 0..200)。作業域に物を置かない。
- Z アームは上げ、荷を吊らない状態で行う。荷重を変える試験は別に、低速・低加速度で行う。
- 速度は移動ごとに 3000 → 9000 → 15900 mm/min と上がる。異音、脱調、ベルト飛びがあれば、その速度で止める。

## Run

```bash
python tools/serial_tool/serial_send.py \
  --port /dev/cu.usbserial-XXXX \
  --csv tools/serial_tool/examples/timed_segment_diagonal_check.csv \
  --startup-delay 6 --timeout 60 --echo
```

## 確認

| 移動 | A/B ステップ | 期待 |
|---|---|---|
| (46.325,101.338) ↔ (105.490,166.260) | A=9927、B=-461(Issue の例 1) | F3000 / F6000 / F9000 で完了、各回の後 `HOMED=YES` |
| (207.684,3.791) ↔ (384.840,174.660) | A=27842、B=503(Issue の例 2) | F3000 / F9000 / F15900 で完了 |
| (50,20) ↔ (200,170) | B=0 | 同上 |
| (50,170) ↔ (200,20) | A=0 | 同上 |

- ログに `TIMED_SEGMENT_FAIL` と `WARN: timed segment queue ran dry` が出ない。
- 最後の `POS` が `HOMED=YES`。
- 失敗した場合は、上の 2 行と、その移動の `XY` 行を記録する。`ALARM_CLEAR` → `HOME` で復帰する。

## 実機結果

- 2026-11-20: 未実施(デバイス未接続)。`--dry-run` のみ。
