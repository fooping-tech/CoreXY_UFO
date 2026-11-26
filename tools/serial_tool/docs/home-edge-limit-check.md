# Home Edge Limit Check (#32)

原点側のリミットスイッチを原点のすぐ近く(`HOME_EDGE_LIMIT_ZONE_MM` = 8 mm 以内)で踏んだときに、
アラームにせず「端に触れた」として止まることの確認手順です。機械が動くので、作業範囲を空け、
電源断の手段を確認してから行ってください。

## 期待する動作

| 状況 | ログ | 状態 |
|---|---|---|
| 原点から8 mm以内で原点limitがON | `LIMIT edge axis=Y Y=0.715 (no alarm)` | `ALARM=NO`、`HOMED=YES`、`LIMIT_Y=ON` |
| JOGで原点limitへ向かっていてON | `JOG end reason=limit_y X=.. Y=.. HOMED=YES` | 同上 |
| XYで原点limitへ向かっていてON | `NACK_XY target=(..) reason=limit_y X=.. Y=..` | 同上(後続の先読みブロックは`reason=stopped`) |
| limitがONのまま原点側へXY | `REJECT: Y limit active and move pushes toward limit` → `NACK_XY ... reason=rejected` | 動かない |
| limitがONのまま原点側へJOG | (原点側の成分が0。セッションは続く) | 動かない |
| 離れる向きへ動いてlimitがOFF | `LIMIT edge released axis=Y Y=.. moved=..` | 通常 |
| 触れた点から20 mm離れてもlimitがON | `ALARM: Y home limit did not release` | アラーム |
| 原点から8 mmより遠い位置でlimitがON | `hard limit active away from home ...` | アラーム(従来どおり) |

HOME直後はX・Yともlimitが0点でONのままなので、`LIMIT edge axis=X X=0.000 (no alarm)`と
`LIMIT edge axis=Y Y=0.000 (no alarm)`が出て、最初に離れたときに`LIMIT edge released`が出る。

## 手順(手動)

1. `HOME`。`HOME complete`、上の2行の`LIMIT edge`を確認。
2. `XY 100 50 F6000`で離れる。`LIMIT edge released axis=X`/`Y`を確認。
3. `JOG 0 -60`を50 msごとに送り続け、−Yの端まで動かす(ゲームの手のスティックと同じ速さ)。
   - `JOG end reason=limit_y X=.. Y=.. HOMED=YES`、`LIMIT edge axis=Y Y=.. (no alarm)`。
   - 1 秒後の`POS`が`ALARM=NO`、`HOMED=YES`。
4. もう一度`JOG 0 -60`: −Y の成分は0で動かない(アラームにならない)。`JOG 0 0`。
5. `JOG 0 60`で離れる: 動き、`LIMIT edge released axis=Y`。
6. `XY 100 0 F6000`(Y の端へ)。位置がずれていなければ`ACK_XY`、ずれていてlimitが先にONになれば
   `NACK_XY ... reason=limit_y X=.. Y=..`。どちらでもアラームにならない。

原点から遠い位置でのアラームは今までどおりなので、ここでは確認しない(手でスイッチを押すと
`hard limit active away from home`、`ALARM_CLEAR`でHOMEDが外れる)。
