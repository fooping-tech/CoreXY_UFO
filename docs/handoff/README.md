# CoreXY_UFO_Game とのハンドオフ

CoreXY_UFO_Game（https://github.com/fooping-tech/CoreXY_UFO_Game ）の `docs/HANDOFF_*.md` で受けた依頼と、
このリポジトリから返した回答の一覧。手順は `.claude/skills/game-handoff/SKILL.md`。

| 受けた依頼 | 受けた日 | 依頼側 HEAD | 回答 | 状態 |
|---|---|---|---|---|
| [HANDOFF_FIRMWARE.md](https://github.com/fooping-tech/CoreXY_UFO_Game/blob/main/docs/HANDOFF_FIRMWARE.md)（§4.1 ホストウォッチドッグ、§4.2 Z 専用コマンド、§4.3 移動中の位置通知、§4.4 POS にサーボ abs） | 2026-10-24 | — | — | 未着手 |
| [HANDOFF_FIRMWARE_JOG.md](https://github.com/fooping-tech/CoreXY_UFO_Game/blob/main/docs/HANDOFF_FIRMWARE_JOG.md)（`JOG`、`STOP`、ジョグ制約撤廃、`JOG_POS`） | 2026-10-24 | 160742d（調査対象 FW） | [REPLY_FIRMWARE_JOG.md](REPLY_FIRMWARE_JOG.md) | 実装済み・実機確認済み（ジョイスティック #9/#10 は未確認） |
| [HANDOFF_FIRMWARE_Z_SERVO.md](https://github.com/fooping-tech/CoreXY_UFO_Game/blob/fix/62-z-servo-reset/docs/HANDOFF_FIRMWARE_Z_SERVO.md)（サーボだけ電源が切れたあとの JOG、ゲーム側 Issue #62） | 2026-11-08 | bf70657（調査対象 FW） | [REPLY_FIRMWARE_Z_SERVO.md](REPLY_FIRMWARE_Z_SERVO.md) | 実装済み・実機未確認（Issue #11） |
