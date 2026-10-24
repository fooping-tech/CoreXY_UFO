#!/bin/bash
# PreToolUse hook (Bash): enforces the GitHub policy in CLAUDE.md.
# Blocks (exit 2, reason on stderr):
#   - git push to main / while on main
#   - force push (--force, -f, --force-with-lease, +refspec)
#   - destructive git: reset --hard, clean -f, branch -D, checkout/restore that discards work,
#     push --delete, filter-branch, gc --prune=now
#   - git commit when the staged diff contains something that looks like a credential
# Anything else passes through. A human can still run these commands directly in a terminal.
set -u

INPUT=$(cat)
CMD=$(printf '%s' "$INPUT" | jq -r '.tool_input.command // empty')
CWD=$(printf '%s' "$INPUT" | jq -r '.cwd // empty')
[ -z "$CMD" ] && exit 0
case "$CMD" in *git*) ;; *) exit 0 ;; esac

block() {
  echo "git-guard: $1" >&2
  echo "このリポジトリの運用ルール(CLAUDE.md)で禁止されています。必要ならユーザーに理由を説明し、ユーザー自身に実行してもらってください。" >&2
  exit 2
}

# Split compound commands so each git invocation is checked on its own.
SEGMENTS=$(printf '%s' "$CMD" | sed -E 's/(&&|\|\||;|\|)/\n/g')

while IFS= read -r seg; do
  seg=$(printf '%s' "$seg" | sed -E 's/^[[:space:]]+//')
  # Drop a leading "git -C dir" so the subcommand is the second word.
  dir=$CWD
  if [[ "$seg" =~ ^git[[:space:]]+-C[[:space:]]+([^[:space:]]+)[[:space:]]+(.*)$ ]]; then
    dir=${BASH_REMATCH[1]}
    seg="git ${BASH_REMATCH[2]}"
  fi
  [[ "$seg" =~ ^git[[:space:]] ]] || continue
  sub=$(printf '%s' "$seg" | awk '{print $2}')

  case "$sub" in
    push)
      if [[ "$seg" =~ (^|[[:space:]])(--force|--force-with-lease|-f)([[:space:]=]|$) ]] || [[ "$seg" =~ [[:space:]]\+[^[:space:]]+ ]]; then
        block "force push は禁止です: $seg"
      fi
      if [[ "$seg" =~ (^|[[:space:]])(--delete|-d)([[:space:]]|$) ]] || [[ "$seg" =~ [[:space:]]:[^[:space:]]+ ]]; then
        block "リモートブランチの削除は禁止です: $seg"
      fi
      if [[ "$seg" =~ (^|[[:space:]:])(main|master)([[:space:]]|$) ]]; then
        block "main への直接 push は禁止です。作業ブランチから PR を作成してください: $seg"
      fi
      # "git push" / "git push origin" with no refspec pushes the current branch.
      cur=$(git -C "${dir:-.}" rev-parse --abbrev-ref HEAD 2>/dev/null)
      nargs=$(printf '%s' "$seg" | awk '{n=0; for(i=3;i<=NF;i++) if ($i !~ /^-/) n++; print n}')
      if { [ "$cur" = main ] || [ "$cur" = master ]; } && { [ "$nargs" -le 1 ] || [[ "$seg" =~ [[:space:]]HEAD([[:space:]]|$) ]]; }; then
        block "main ブランチ上での push は禁止です。作業ブランチを作成してください。"
      fi
      ;;
    reset)
      [[ "$seg" =~ --hard ]] && block "git reset --hard は禁止です(未コミットの変更が失われます)。"
      ;;
    clean)
      [[ "$seg" =~ (^|[[:space:]])-[a-zA-Z]*f ]] && block "git clean -f は禁止です。"
      ;;
    branch)
      [[ "$seg" =~ (^|[[:space:]])(-D|--delete[[:space:]]+--force|-d[[:space:]]+-f)([[:space:]]|$) ]] && block "git branch -D は禁止です。マージ済みブランチは -d で削除してください。"
      ;;
    checkout|restore)
      [[ "$sub" = restore && "$seg" =~ --staged && ! "$seg" =~ --worktree ]] && continue
      [[ "$seg" =~ (^|[[:space:]])(--[[:space:]]+\.|\.)([[:space:]]|$) ]] && block "作業ツリー全体を破棄する $sub は禁止です。"
      ;;
    filter-branch|filter-repo)
      block "履歴の書き換えは禁止です。"
      ;;
    gc)
      [[ "$seg" =~ --prune=now ]] && block "git gc --prune=now は禁止です。"
      ;;
    commit)
      hits=$(git -C "${dir:-.}" diff --cached 2>/dev/null | grep -E '^\+' | grep -E \
        'gh[pousr]_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{30,}|AKIA[0-9A-Z]{16}|sk-[A-Za-z0-9_-]{30,}|-----BEGIN [A-Z ]*PRIVATE KEY-----|xox[baprs]-[A-Za-z0-9-]{10,}' | head -3)
      [ -n "$hits" ] && block "ステージされた変更に認証情報らしき文字列があります。コミットから外してください。"
      files=$(git -C "${dir:-.}" diff --cached --name-only 2>/dev/null | grep -E '(^|/)(\.env|.*\.pem|.*\.key|id_rsa.*|credentials.*|secrets?\..*)$' | head -3)
      [ -n "$files" ] && block "認証情報らしきファイルがステージされています: $files"
      ;;
  esac
done <<< "$SEGMENTS"

exit 0
