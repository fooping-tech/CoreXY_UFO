#!/bin/bash
# Collect the GitHub/git state the /dev skill needs to decide which phase is next.
# Read-only: fetches from origin but never changes branches, commits or GitHub.
#
#   dev-state.sh                 overview + the current branch
#   dev-state.sh issue <N>       one issue: branches, PRs, CI, reviews
#   dev-state.sh pr <N>          one PR: checks, reviews, mergeability
set -u

ROOT=$(git rev-parse --show-toplevel 2>/dev/null) || { echo "not in a git repository" >&2; exit 1; }
cd "$ROOT"
DEFAULT=$(gh repo view --json defaultBranchRef --jq .defaultBranchRef.name 2>/dev/null || echo main)

section() { printf '\n## %s\n' "$1"; }

git fetch --quiet --prune origin 2>/dev/null || echo "(warning: git fetch failed; remote state may be stale)"

local_state() {
  section "Local"
  local br
  br=$(git rev-parse --abbrev-ref HEAD)
  echo "branch: $br (default: $DEFAULT)"
  local dirty
  dirty=$(git status --porcelain)
  if [ -n "$dirty" ]; then echo "working tree: DIRTY"; echo "$dirty" | head -20; else echo "working tree: clean"; fi
  if git rev-parse --verify --quiet "origin/$br" >/dev/null; then
    echo "vs origin/$br: ahead $(git rev-list --count "origin/$br..$br"), behind $(git rev-list --count "$br..origin/$br")"
  else
    echo "origin/$br: not pushed"
  fi
  if [ "$br" != "$DEFAULT" ]; then
    echo "vs origin/$DEFAULT: ahead $(git rev-list --count "origin/$DEFAULT..$br"), behind $(git rev-list --count "$br..origin/$DEFAULT")"
    echo "commits on branch:"
    git log --oneline "origin/$DEFAULT..$br" | head -20
  fi
}

pr_detail() {
  local n=$1
  section "PR #$n"
  gh pr view "$n" --json number,title,state,isDraft,headRefName,baseRefName,mergeable,mergeStateStatus,reviewDecision,url,closingIssuesReferences \
    --jq '"#\(.number) \(.title)\nstate: \(.state) draft: \(.isDraft)  \(.headRefName) -> \(.baseRefName)\nmergeable: \(.mergeable) mergeState: \(.mergeStateStatus) reviewDecision: \(.reviewDecision // "none")\ncloses: \([.closingIssuesReferences[].number] | map("#\(.)") | join(" "))\n\(.url)"'
  section "CI checks (PR #$n)"
  gh pr checks "$n" 2>&1 || true
  section "Reviews (PR #$n)"
  gh pr view "$n" --json reviews --jq '.reviews[] | "\(.state) by \(.author.login) at \(.submittedAt): \(.body | split("\n")[0])"' 2>/dev/null
  local repo
  repo=$(gh repo view --json nameWithOwner --jq .nameWithOwner)
  # Inline review threads: unresolved ones are the work list for the fix phase.
  gh api graphql -f query='query($o:String!,$r:String!,$n:Int!){repository(owner:$o,name:$r){pullRequest(number:$n){reviewThreads(first:100){nodes{isResolved path line comments(first:1){nodes{author{login} body}}}}}}}' \
    -F o="${repo%/*}" -F r="${repo#*/}" -F n="$n" \
    --jq '.data.repository.pullRequest.reviewThreads.nodes[] | "\(if .isResolved then "resolved  " else "UNRESOLVED" end) \(.path):\(.line // "-") \(.comments.nodes[0].author.login): \(.comments.nodes[0].body | split("\n")[0])"' 2>/dev/null
  section "Conversation comments (PR #$n, last 10)"
  gh pr view "$n" --json comments --jq '.comments[-10:][] | "\(.author.login) at \(.createdAt): \(.body | split("\n")[0])"' 2>/dev/null
}

issue_detail() {
  local n=$1
  section "Issue #$n"
  gh issue view "$n" --json number,title,state,stateReason,labels,assignees,url \
    --jq '"#\(.number) \(.title)\nstate: \(.state) \(.stateReason // "")  labels: \([.labels[].name] | join(","))\n\(.url)"' || return 1
  section "Branches for #$n (pattern */$n-*)"
  git branch -a --format='%(refname:short)' | grep -E "(^|/)[a-z]+/$n-" || echo "(none)"
  section "PRs referencing #$n"
  local prs
  prs=$(gh pr list --state all --limit 100 --json number,title,state,headRefName,body \
    --jq "[.[] | select((.headRefName | test(\"^[a-z]+/$n-\")) or (.body // \"\" | test(\"#$n([^0-9]|\$)\")))] | .[] | \"\(.number)\t\(.state)\t\(.headRefName)\t\(.title)\"")
  if [ -z "$prs" ]; then echo "(none)"; else echo "$prs"; fi
  echo "$prs" | awk -F'\t' '$2=="OPEN"{print $1}' | while read -r p; do [ -n "$p" ] && pr_detail "$p"; done
}

overview() {
  section "Open issues"
  gh issue list --state open --limit 30 --json number,title,labels,assignees \
    --jq '.[] | "#\(.number)\t\(.title)\t[\([.labels[].name] | join(","))]"' || true
  section "Open PRs"
  gh pr list --state open --json number,title,headRefName,isDraft,reviewDecision \
    --jq '.[] | "#\(.number)\t\(.headRefName)\t\(.title)\tdraft=\(.isDraft) review=\(.reviewDecision // "none")"' || true
}

case "${1:-}" in
  issue) [ -n "${2:-}" ] || { echo "usage: $0 issue <N>" >&2; exit 2; }; local_state; issue_detail "$2" ;;
  pr)    [ -n "${2:-}" ] || { echo "usage: $0 pr <N>" >&2; exit 2; }; local_state; pr_detail "$2" ;;
  "")
    local_state
    overview
    br=$(git rev-parse --abbrev-ref HEAD)
    n=$(echo "$br" | sed -nE 's#^[a-z]+/([0-9]+)-.*#\1#p')
    if [ -n "$n" ]; then issue_detail "$n"
    else
      p=$(gh pr view --json number --jq .number 2>/dev/null)
      [ -n "$p" ] && pr_detail "$p"
    fi
    ;;
  *) echo "usage: $0 [issue <N> | pr <N>]" >&2; exit 2 ;;
esac
