---
name: dev
description: GitHub Issue起点の開発ワークフロー(Issue取得→調査→ブランチ→実装・テスト→commit/push→PR→CI・レビュー確認→修正→承認後マージ→Issueクローズ)を、GitHubの状態から現在のフェーズを判定して実行・再開する。「Issue #12を実装して」「次に対応するIssueを選んで」「PRまで作って」「PR #15のレビューに対応して」「今の開発状況を教えて」「マージできる状態？」「このPRをマージして」「さっきの作業の続きをやって」、/dev、issue/PR/branch/merge/CI/review に関する依頼で使う。
---

# /dev: Issue → PR → マージ

ユーザーはGitHub操作に不慣れ。操作方法を逐一質問せず、GitHubとgitの状態を調べて自分で
次の手を決める。確認するのは次の3つだけ:

1. 仕様の重要な曖昧さ(受け入れ条件が決められない、安全性・挙動の選択肢が複数ある)
2. 破壊的・取り消しにくい操作(履歴の書き換え、ブランチ削除、Issue/PRのクローズ、機械が動く実機テスト)
3. マージ(毎回、明示的な承認を得る)

AGENTS.md(層構造、Core分離、ピン制約、§15 SPEC/PLANS同期、§16 Build/Test)は常に優先する。
ゲーム側ハンドオフの依頼は、中身を `game-handoff` スキルで処理し、GitHubの手順はこのスキルに従う。

## 0. まず状態を取る(毎回)

```bash
.claude/skills/dev/scripts/dev-state.sh              # 全体 + 現在ブランチ
.claude/skills/dev/scripts/dev-state.sh issue <N>    # Issue単位
.claude/skills/dev/scripts/dev-state.sh pr <N>       # PR単位
```

読み取り専用(fetchのみ)。セッションの記憶よりGitHubとgitの状態を信じる。出力から
下表でフェーズを判定し、完了済みのフェーズはやり直さない。

| 状態 | 完了しているフェーズ | 次 |
|---|---|---|
| Issueなし(自由文の依頼) | – | 1: Issueを作る |
| Issue OPEN、ブランチ `*/<N>-*` なし | 1 | 2→3 |
| ブランチあり、`origin/main` より先のコミットなし | 3 | 4 |
| コミットあり、未push / 作業ツリーがdirty | 4途中 | 4の続き(検証)→5 |
| push済み、PRなし | 5 | 6 |
| PR OPEN、CI実行中 | 6 | 7: 完了を待つ |
| PR OPEN、CI失敗 / UNRESOLVEDスレッド / CHANGES_REQUESTED | 7 | 8 |
| PR OPEN、CI成功、未解決の必須指摘なし | 8 | 9: 承認待ち |
| PR MERGED、Issue OPEN | 9 | 10 |
| PR MERGED、Issue CLOSED | 全部 | 結果報告のみ |

実機テストの結果はGitHubに残らない。そのためPR本文の「テスト結果」に必ず書き、再開時はそこを読む。

## 指示の振り分け

| 指示 | 動き |
|---|---|
| 「次に対応するIssueを選んで」 | 0 → OPENでPRのないIssueを選ぶ(bug > 依存されている > 古い順)。理由と作業計画を示し、実装まで頼まれていれば1から進める |
| 「Issue #N を実装して」 | 0(issue N) → 判定したフェーズから6(PR作成)まで |
| 「PRまで作って」 | 現在の作業(ブランチ/Issue)について6まで |
| 「PR #N のレビューに対応して」 | 0(pr N) → 7 → 8 → 再度7 |
| 「今の開発状況を教えて」 | 0 → 報告のみ(変更しない)。Open Issue/PR、各PRのCI・レビュー、ローカルの未push |
| 「マージできる状態？」 | 9のチェックリストだけ実行し、可否と不足を報告。マージしない |
| 「このPRをマージして」 | 9(チェック → 承認 → マージ)→ 10 |
| 「さっきの作業の続き」 | 0(引数なし)。現在ブランチが main なら `git for-each-ref --sort=-committerdate refs/heads --format='%(refname:short) %(committerdate:relative)' \| head` と Open PR から直近の作業を特定 → 判定したフェーズから |

## フェーズ

### 1. Issue取得と要件整理

- `gh issue view <N> --comments`。要望、受け入れ条件、制約、関連Issue/PRを箇条書きにする。
- 受け入れ条件がなければ本文から作り、ユーザーに示す。安全性や挙動に関わる曖昧さがあれば、ここで1回だけ質問する。
- 依頼にIssueがなければ、`.github/ISSUE_TEMPLATE/task.md` の形でIssueを作り(`gh issue create --title .. --body-file ..`)、番号をユーザーに伝える。

### 2. 調査

- AGENTS.md、SPEC.md/PLANS.mdの関連節、対象コードを読む。広い探索はExploreサブエージェントに任せる。
- 変更するファイル、影響する env(`platformio.ini`)、追加するテスト、SPEC/PLANSの更新要否、実機確認の内容(機械が動くか)を決める。
- 大きい変更なら、計画を短く示してから進める。

### 3. ブランチ

- 名前は `<type>/<N>-<slug>`。type は `feature` / `fix` / `docs` / `chore`、slug は英小文字とハイフン(例: `feature/12-pen-speed`)。
- 既存ブランチ(ローカル `*/<N>-*` または `origin/*/<N>-*`)があれば作らず `git switch` する。リモートだけにあれば `git switch <name>` で追跡する。
- 新規の場合: `git fetch origin && git switch -c <name> origin/main`。
- main上に未コミットの変更があれば、そのIssueの作業かを確かめてから持っていく。無関係なら止めて聞く。

### 4. 実装とテスト

- AGENTS.mdどおりに実装する。純粋ロジックは `test/native/` にテストを足す(`tools/run_native_motion_tests.sh` へ登録)。機能を追加したら `tools/serial_tool/examples/<feature>_check.csv` と `docs/<feature>-check.md` も作る。
- 計画にない仕様・暫定値・安全対策を入れたら、SPEC.md と PLANS.md を更新する(AGENTS.md §15)。
- 検証:
  ```bash
  .claude/skills/dev/scripts/verify.sh          # secret scan + native + Python + plotter 3 envのビルド
  .claude/skills/dev/scripts/verify.sh --all    # atoms3/sts3020 側を触ったとき
  ```
  失敗したら直して再実行する。直せないものは「未解決事項」に回す。
- 実機(AGENTS.md §16):
  - ポートを `ls /dev/cu.usbserial-*` で探し、`lsof <port>` で使用中でないか見る。使用中なら、他人のプロセスは止めずユーザーに頼む。
  - `~/.platformio/penv/bin/pio run -e m5stack-core2 -t upload --upload-port <port>` で書き込み、`SELFTEST` と動作しないチェックCSVを実行する。
  - 機械が動くテスト(HOME、XY、JOG)は、作業域が空いているかをユーザーに確認してから行う。
  - 実行できなかった手順と理由は記録する。

### 5. コミットとpush

- `git add <ファイル名>` で個別に追加する(`-A` は使わない)。`.venv/`、`.pio/`、ログ、認証情報は入れない。
- メッセージは既存履歴に合わせ、英語の命令形で1行要約を書く。末尾にシステムが指定する Co-Authored-By 行を付ける。
- コミットは追加のみ。amend、rebase、force pushはしない(push済みのものは特に)。
- `git push -u origin <branch>`。mainへのpushは禁止(hookがブロックする)。

### 6. PR作成

- 既存PRを確認: `gh pr list --head <branch> --state all`。OPENがあれば作らず更新する(本文は `gh pr edit --body-file`)。
- 本文は `.github/pull_request_template.md` の見出しで書く(日本語):
  - `Closes #N`(これでマージ時にIssueが閉じる)
  - 変更概要(受け入れ条件ごとの対応)
  - テスト結果(コマンドと結果、実機の測定値。未実施は `[ ]` のまま理由を書く)
  - 確認方法
  - 未解決事項
  - 末尾にシステムが指定するPR用のattribution行
- `gh pr create --base main --head <branch> --title "<日本語の要約>" --body-file <scratchpad>/pr.md`
- ユーザーにPRのURLを伝える。

### 7. CIとレビューの確認

- CI: `gh pr checks <N> --watch` をバックグラウンドで実行し、完了の通知を待つ。失敗したら `gh run view <run-id> --log-failed` で原因を調べる。
- セルフレビュー: PR作成後、独立したサブエージェント(general-purpose)に、Issue本文とPR差分(`gh pr diff <N>`)を渡してレビューさせる。観点:
  - 受け入れ条件を満たしているか
  - 既存機能の破壊(特に `game-handoff` で約束したシリアル応答行の先頭トークン)
  - 不要な変更
  - セキュリティ(秘密情報、外部入力の検証)
  - テストの過不足
  - AGENTS.mdの禁止事項(§14)

  バグ探しは `/code-review` スキルも使える。
- 人と bot のレビュー: `dev-state.sh pr <N>` の Reviews、UNRESOLVEDスレッド、Conversation comments を読む。
  このリポジトリでは `chatgpt-codex-connector`(Codex)がPR作成の数分後に自動レビューする。
  レビューが付くまで待ってから9へ進む。バッジ P0/P1 は必須、P2 は推奨、P3 は任意として扱う。
  指摘の全文は `gh api repos/{owner}/{repo}/pulls/<N>/comments` で読む。
- 指摘は次の重要度で整理して報告する:
  - **必須**: バグ、受け入れ条件未達、安全・セキュリティ
  - **推奨**: 設計・保守性
  - **任意**: 好みやnit
  - **対応しない**: 理由付き

### 8. 修正と再検証

- 必須を全部、推奨は妥当なものを直す。仕様が変わる指摘はユーザーに確認する。
- 4の検証を再実行し、新しいコミットとしてpushする。PR本文のテスト結果を更新する。
- 対応した指摘にはスレッドへ返信する(`gh api repos/{owner}/{repo}/pulls/<N>/comments/<id>/replies -f body=..`、全体には `gh pr comment`)。スレッドのresolveはレビュアーに任せる。
- 7に戻る。

### 9. マージ(承認必須)

チェックリスト(「マージできる状態？」はここまで):

- [ ] PRがOPENでdraftでない
- [ ] `mergeable: MERGEABLE`。mainより遅れていれば `git merge origin/main` をブランチに入れてpushする(rebaseはしない)。競合時は解消内容をユーザーに見せる
- [ ] CIが全部成功(CIがまだ設定されていない場合は、ローカルの `verify.sh` の結果で代える)
- [ ] CHANGES_REQUESTEDがない。必須指摘が残っていない
- [ ] PR本文のテスト結果に実機確認の状態が書かれている

満たしたら、要約(PR、変更内容、テスト結果、未解決事項、マージ方法)を示し、AskUserQuestionで承認を得る。
承認後: `gh pr merge <N> --merge --delete-branch`(既存履歴と同じmerge commit。マージ済みブランチを削除する)。
承認なしにマージしない。前の会話でもらった承認も使わない。

### 10. Issueの確認と報告

- `gh issue view <N> --json state`。`Closes #N` で閉じていなければ、マージ済みPRへのリンクを付けてコメントし、ユーザーに確認してからクローズする。
- `git switch main && git pull --ff-only`
- 報告: マージしたPR、閉じたIssue、残った未解決事項(必要なら後続Issueの作成を提案)。

## 運用ポリシー(hookと設定で一部を強制)

- `.claude/hooks/git-guard.sh` がブロックする操作: mainへのpush、force push、リモートブランチ削除、`reset --hard`、`clean -f`、`branch -D`、作業ツリーの全破棄、履歴の書き換え、認証情報を含むコミット。ブロックされたら回避策を探さず、ユーザーに理由を伝える。
- `gh pr merge`、`gh issue close`、`gh pr close` は `.claude/settings.json` で毎回確認が出る。
- 1 Issue = 1ブランチ = 1 PR。既存があれば再利用する。
- Issue番号はブランチ名とPR本文(`Closes #N`)に入れる。
- 認証情報、秘密鍵、トークン、`.env` はコミットしない(`verify.sh` と hook が検査する)。

## 報告の形

各フェーズの終わりに、日本語で短く報告する:

- 今いるフェーズ、やったこと
- 結果(テストのPASS/FAIL、PRやCIのURL)
- 次にやること、またはユーザーに判断してほしいこと

GitHub用語(ブランチ、PR、CI)には、必要に応じて一言で説明を添える。
