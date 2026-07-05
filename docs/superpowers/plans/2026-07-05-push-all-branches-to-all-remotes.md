# Push All Branches to All Remotes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a safe one-command Bash script that pushes every local branch to every configured Git remote.

**Architecture:** A repository-root Bash script discovers local branches and remotes at runtime, confirms destructive intent for real pushes, and invokes `git push <remote> --all` once per remote. A standalone Bash integration test uses temporary source and bare repositories so all push behavior is validated locally without contacting configured Gitee or GitHub remotes.

**Tech Stack:** Bash, Git CLI, temporary local bare Git repositories

---

### Task 1: Add failing integration tests

**Files:**
- Create: `tests/test_push_all_branches_to_all_remotes.sh`
- Test: `tests/test_push_all_branches_to_all_remotes.sh`

- [ ] **Step 1: Write the test harness and behavior tests**

Create an executable Bash test containing helpers to initialize an isolated source repository, create two bare remotes, run the target script from the source repository, and assert exit codes and refs. Cover these exact cases:

```bash
#!/usr/bin/env bash
set -uo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
SCRIPT="$ROOT_DIR/push_all_branches_to_all_remotes.sh"
TMP_ROOT=$(mktemp -d)
trap 'rm -rf "$TMP_ROOT"' EXIT

pass_count=0

fail() {
  echo "FAIL: $*" >&2
  exit 1
}

assert_ref_exists() {
  local remote=$1
  local ref=$2
  git --git-dir="$remote" show-ref --verify --quiet "$ref" ||
    fail "missing $ref in $remote"
}

assert_no_refs() {
  local remote=$1
  if git --git-dir="$remote" show-ref --quiet; then
    fail "expected no refs in $remote"
  fi
}

new_source_repo() {
  local name=$1
  local source="$TMP_ROOT/$name/source"
  mkdir -p "$source"
  git -C "$source" init -q
  git -C "$source" config user.name "Push Script Test"
  git -C "$source" config user.email "push-script-test@example.invalid"
  git -C "$source" checkout -q -b main
  git -C "$source" commit -q --allow-empty -m initial
  git -C "$source" branch feature
  printf '%s\n' "$source"
}

new_bare_remote() {
  local path=$1
  git init -q --bare "$path"
}

run_test() {
  local name=$1
  shift
  "$@"
  pass_count=$((pass_count + 1))
  echo "PASS: $name"
}

test_dry_run_does_not_update_remotes() {
  local source remote_a remote_b
  source=$(new_source_repo dry-run)
  remote_a="$TMP_ROOT/dry-run/remote-a.git"
  remote_b="$TMP_ROOT/dry-run/remote-b.git"
  new_bare_remote "$remote_a"
  new_bare_remote "$remote_b"
  git -C "$source" remote add remote-a "$remote_a"
  git -C "$source" remote add remote-b "$remote_b"

  (cd "$source" && "$SCRIPT" --dry-run) >/dev/null
  assert_no_refs "$remote_a"
  assert_no_refs "$remote_b"
}

test_confirmed_push_updates_all_remotes() {
  local source remote_a remote_b
  source=$(new_source_repo confirmed)
  remote_a="$TMP_ROOT/confirmed/remote-a.git"
  remote_b="$TMP_ROOT/confirmed/remote-b.git"
  new_bare_remote "$remote_a"
  new_bare_remote "$remote_b"
  git -C "$source" remote add remote-a "$remote_a"
  git -C "$source" remote add remote-b "$remote_b"

  printf 'y\n' | (cd "$source" && "$SCRIPT") >/dev/null
  for remote in "$remote_a" "$remote_b"; do
    assert_ref_exists "$remote" refs/heads/main
    assert_ref_exists "$remote" refs/heads/feature
  done
}

test_rejected_confirmation_does_not_push() {
  local source remote
  source=$(new_source_repo rejected)
  remote="$TMP_ROOT/rejected/remote.git"
  new_bare_remote "$remote"
  git -C "$source" remote add target "$remote"

  printf 'n\n' | (cd "$source" && "$SCRIPT") >/dev/null
  assert_no_refs "$remote"
}

test_failed_remote_does_not_block_later_remote() {
  local source good_remote output status
  source=$(new_source_repo partial-failure)
  good_remote="$TMP_ROOT/partial-failure/good.git"
  new_bare_remote "$good_remote"
  git -C "$source" remote add a-broken "$TMP_ROOT/partial-failure/missing.git"
  git -C "$source" remote add z-good "$good_remote"

  set +e
  output=$(printf 'y\n' | (cd "$source" && "$SCRIPT") 2>&1)
  status=$?
  set -e
  [ "$status" -ne 0 ] || fail "partial failure returned success"
  assert_ref_exists "$good_remote" refs/heads/main
  printf '%s\n' "$output" | grep -q 'a-broken' || fail "missing failed remote summary"
  printf '%s\n' "$output" | grep -q 'z-good' || fail "missing successful remote summary"
}

test_invalid_argument_fails() {
  local source status
  source=$(new_source_repo invalid-argument)
  set +e
  (cd "$source" && "$SCRIPT" --unknown) >/dev/null 2>&1
  status=$?
  set -e
  [ "$status" -ne 0 ] || fail "unknown argument returned success"
}

test_missing_remote_fails() {
  local source status
  source=$(new_source_repo no-remote)
  set +e
  (cd "$source" && "$SCRIPT" --dry-run) >/dev/null 2>&1
  status=$?
  set -e
  [ "$status" -ne 0 ] || fail "missing remote returned success"
}

run_test "dry-run leaves remotes unchanged" test_dry_run_does_not_update_remotes
run_test "confirmed push updates all remotes" test_confirmed_push_updates_all_remotes
run_test "rejected confirmation does not push" test_rejected_confirmation_does_not_push
run_test "failed remote does not block later remote" test_failed_remote_does_not_block_later_remote
run_test "invalid argument fails" test_invalid_argument_fails
run_test "missing remote fails" test_missing_remote_fails

echo "All $pass_count tests passed."
```

- [ ] **Step 2: Make the test executable**

Run: `chmod +x tests/test_push_all_branches_to_all_remotes.sh`

- [ ] **Step 3: Run the tests and verify RED**

Run: `bash tests/test_push_all_branches_to_all_remotes.sh`

Expected: FAIL because `push_all_branches_to_all_remotes.sh` does not exist; this confirms the tests exercise the missing feature.

### Task 2: Implement the minimal safe push script

**Files:**
- Create: `push_all_branches_to_all_remotes.sh`
- Test: `tests/test_push_all_branches_to_all_remotes.sh`

- [ ] **Step 1: Implement the script**

Create this executable script:

```bash
#!/usr/bin/env bash
set -uo pipefail

usage() {
  echo "Usage: $0 [--dry-run]"
}

dry_run=false
case ${1-} in
  "") ;;
  --dry-run) dry_run=true ;;
  *)
    usage >&2
    exit 2
    ;;
esac

if [ "$#" -gt 1 ]; then
  usage >&2
  exit 2
fi

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "错误：当前目录不属于 Git 仓库。" >&2
  exit 1
fi

mapfile -t branches < <(git for-each-ref --format='%(refname:short)' refs/heads/)
mapfile -t remotes < <(git remote)

if [ "${#branches[@]}" -eq 0 ]; then
  echo "错误：当前仓库没有本地分支。" >&2
  exit 1
fi

if [ "${#remotes[@]}" -eq 0 ]; then
  echo "错误：当前仓库没有配置远程仓库。" >&2
  exit 1
fi

echo "本地分支（${#branches[@]}）："
printf '  - %s\n' "${branches[@]}"
echo "远程仓库（${#remotes[@]}）："
printf '  - %s\n' "${remotes[@]}"

if [ "$dry_run" = false ]; then
  printf '将所有本地分支推送到以上全部远程仓库，是否继续？[y/N] '
  read -r answer
  case $answer in
    y|Y) ;;
    *)
      echo "已取消，未执行推送。"
      exit 0
      ;;
  esac
fi

push_options=()
if [ "$dry_run" = true ]; then
  push_options+=(--dry-run)
  echo "Dry-run：只预览，不更新远程仓库。"
fi

successful_remotes=()
failed_remotes=()
for remote in "${remotes[@]}"; do
  echo
  echo "==> 推送到 $remote"
  if git push "${push_options[@]}" "$remote" --all; then
    successful_remotes+=("$remote")
  else
    failed_remotes+=("$remote")
  fi
done

echo
echo "同步结果："
if [ "${#successful_remotes[@]}" -gt 0 ]; then
  printf '  成功：%s\n' "${successful_remotes[*]}"
fi
if [ "${#failed_remotes[@]}" -gt 0 ]; then
  printf '  失败：%s\n' "${failed_remotes[*]}" >&2
  exit 1
fi

echo "全部远程仓库处理成功。"
```

- [ ] **Step 2: Make the script executable**

Run: `chmod +x push_all_branches_to_all_remotes.sh`

- [ ] **Step 3: Run the tests and verify GREEN**

Run: `bash tests/test_push_all_branches_to_all_remotes.sh`

Expected: `All 6 tests passed.` No configured network remote is contacted.

### Task 3: Audit and document the implementation

**Files:**
- Modify: `docs/ai_refactor_records/20260705_push_all_branches_to_all_remotes.md`
- Inspect: `push_all_branches_to_all_remotes.sh`
- Inspect: `tests/test_push_all_branches_to_all_remotes.sh`

- [ ] **Step 1: Run static syntax checks**

Run: `bash -n push_all_branches_to_all_remotes.sh tests/test_push_all_branches_to_all_remotes.sh`

Expected: exit status 0 with no output.

- [ ] **Step 2: Verify the safety boundary**

Run: `rg -n -- '--force|--mirror|--prune|push|remote|refs/heads' push_all_branches_to_all_remotes.sh tests/test_push_all_branches_to_all_remotes.sh`

Expected: production pushes use only `git push [--dry-run] <remote> --all`; forbidden destructive options are absent from the production script.

- [ ] **Step 3: Inspect scope and whitespace**

Run: `git diff --check && git status --short && git diff -- push_all_branches_to_all_remotes.sh tests/test_push_all_branches_to_all_remotes.sh docs/ai_refactor_records/20260705_push_all_branches_to_all_remotes.md`

Expected: only the planned script, tests, and record are changed by implementation; pre-existing user changes remain untouched.

- [ ] **Step 4: Complete the refactor record**

Replace the pending Modifier and Auditor sections with the exact files changed, checks performed, behavior preserved, intentional behavior, issues found, and `PASS` or `NEEDS_FIX`. Explicitly record that no build and no real remote push were run.

- [ ] **Step 5: Commit implementation files**

Run:

```bash
git add push_all_branches_to_all_remotes.sh tests/test_push_all_branches_to_all_remotes.sh
git add -f docs/ai_refactor_records/20260705_push_all_branches_to_all_remotes.md docs/superpowers/plans/2026-07-05-push-all-branches-to-all-remotes.md
git commit -m "feat: push all branches to every remote"
```

Expected: one implementation commit containing only the planned files; the pre-existing `AGENTS.md` working-tree change remains uncommitted.
