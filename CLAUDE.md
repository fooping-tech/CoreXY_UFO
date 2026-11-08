# CLAUDE.md

プロジェクトの設計・実装規約は `AGENTS.md` に従う。ここには GitHub 運用ルールだけを置く。

## GitHub 運用

- Issue/PR/ブランチ/マージに関わる作業は `/dev` スキル(`.claude/skills/dev/SKILL.md`)の手順で行う。
- main へ直接 push しない。作業は `<type>/<Issue番号>-<slug>` ブランチで行い、PR を通してマージする。
- 1 Issue につき 1 PR。既存のブランチや PR があれば、新しく作らずに使う。PR 本文には `Closes #<番号>` を入れる。
- PR を作る前に `.claude/skills/dev/scripts/verify.sh` を実行し、実機テストの結果を PR 本文に書く。
- マージはユーザーの明示的な承認を得てから行う。CI の成功を確認する。
- force push、`reset --hard`、`branch -D` などの破壊的な git 操作をしない(`.claude/hooks/git-guard.sh` が止める)。
- 認証情報、秘密鍵、トークンをコミットしない。
