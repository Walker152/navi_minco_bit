# Exclude Remotes Option Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a repeatable `--exclude <remote>` option that prevents selected remotes from receiving any push.

**Architecture:** Extend the existing Bash argument parser to accept `--dry-run` and repeated exclusions in any order. Validate exclusions against configured remotes before confirmation, partition remotes into selected and excluded arrays, and retain the existing per-remote push loop for selected remotes only.

**Tech Stack:** Bash, Git CLI, temporary local bare Git repositories

---

### Task 1: Add failing exclusion tests

**Files:**
- Modify: `tests/test_push_all_branches_to_all_remotes.sh`

- [ ] **Step 1: Add a test that excludes one remote**

Append a test that creates two bare remotes, runs `--exclude remote-a`, verifies `remote-a` has no refs, and verifies both local branches exist in `remote-b`:

```bash
test_excluded_remote_is_not_pushed() {
  local source remote_a remote_b
  source=$(new_source_repo excluded)
  remote_a="$TMP_ROOT/excluded/remote-a.git"
  remote_b="$TMP_ROOT/excluded/remote-b.git"
  new_bare_remote "$remote_a"
  new_bare_remote "$remote_b"
  git -C "$source" remote add remote-a "$remote_a"
  git -C "$source" remote add remote-b "$remote_b"

  printf 'y\n' | (cd "$source" && "$SCRIPT" --exclude remote-a) >/dev/null ||
    fail "push with exclusion failed"
  assert_no_refs "$remote_a"
  assert_ref_exists "$remote_b" refs/heads/main
  assert_ref_exists "$remote_b" refs/heads/feature
}
```

- [ ] **Step 2: Add repeated-exclusion and argument-order coverage**

Append a test with three remotes. Invoke `--exclude remote-a --dry-run --exclude remote-b` to verify arbitrary ordering succeeds without updates, then invoke repeated exclusions without dry-run and verify only `remote-c` receives branches:

```bash
test_repeated_exclusions_and_dry_run_order() {
  local source remote_a remote_b remote_c
  source=$(new_source_repo repeated-exclusions)
  remote_a="$TMP_ROOT/repeated-exclusions/remote-a.git"
  remote_b="$TMP_ROOT/repeated-exclusions/remote-b.git"
  remote_c="$TMP_ROOT/repeated-exclusions/remote-c.git"
  new_bare_remote "$remote_a"
  new_bare_remote "$remote_b"
  new_bare_remote "$remote_c"
  git -C "$source" remote add remote-a "$remote_a"
  git -C "$source" remote add remote-b "$remote_b"
  git -C "$source" remote add remote-c "$remote_c"

  (cd "$source" && "$SCRIPT" --exclude remote-a --dry-run --exclude remote-b) >/dev/null ||
    fail "combined dry-run and exclusions failed"
  assert_no_refs "$remote_a"
  assert_no_refs "$remote_b"
  assert_no_refs "$remote_c"

  printf 'y\n' | (cd "$source" && "$SCRIPT" --exclude remote-a --exclude remote-b) >/dev/null ||
    fail "push with repeated exclusions failed"
  assert_no_refs "$remote_a"
  assert_no_refs "$remote_b"
  assert_ref_exists "$remote_c" refs/heads/main
  assert_ref_exists "$remote_c" refs/heads/feature
}
```

- [ ] **Step 3: Add validation tests**

Add three failure cases for unknown remote, missing value, and excluding every remote. Each must return nonzero before any remote is updated:

```bash
test_invalid_exclusions_fail_before_push() {
  local source remote status
  source=$(new_source_repo invalid-exclusions)
  remote="$TMP_ROOT/invalid-exclusions/remote.git"
  new_bare_remote "$remote"
  git -C "$source" remote add target "$remote"

  for arguments in "--exclude missing" "--exclude" "--exclude target"; do
    set +e
    (cd "$source" && "$SCRIPT" $arguments) >/dev/null 2>&1
    status=$?
    set -e
    [ "$status" -ne 0 ] || fail "invalid exclusion returned success: $arguments"
    assert_no_refs "$remote"
  done
}
```

Register all three functions with `run_test` before the final total line.

- [ ] **Step 4: Run tests and verify RED**

Run: `bash tests/test_push_all_branches_to_all_remotes.sh`

Expected: FAIL because the existing script rejects `--exclude` as an unknown argument.

### Task 2: Implement exclusion parsing and filtering

**Files:**
- Modify: `push_all_branches_to_all_remotes.sh`

- [ ] **Step 1: Replace single-argument parsing**

Change usage to `用法：$0 [--dry-run] [--exclude <remote>]...` and replace the existing `case` plus argument-count check with:

```bash
dry_run=false
excluded_remote_names=()
while [ "$#" -gt 0 ]; do
  case $1 in
    --dry-run)
      dry_run=true
      shift
      ;;
    --exclude)
      if [ "$#" -lt 2 ]; then
        echo "错误：--exclude 缺少远程仓库名称。" >&2
        usage >&2
        exit 2
      fi
      excluded_remote_names+=("$2")
      shift 2
      ;;
    *)
      usage >&2
      exit 2
      ;;
  esac
done
```

- [ ] **Step 2: Validate and partition remotes**

After checking that configured remotes are non-empty, validate every requested exclusion and build arrays in configured-remote order:

```bash
for excluded_name in "${excluded_remote_names[@]}"; do
  found=false
  for remote in "${remotes[@]}"; do
    if [ "$remote" = "$excluded_name" ]; then
      found=true
      break
    fi
  done
  if [ "$found" = false ]; then
    echo "错误：未找到要排除的远程仓库：$excluded_name" >&2
    exit 2
  fi
done

selected_remotes=()
excluded_remotes=()
for remote in "${remotes[@]}"; do
  excluded=false
  for excluded_name in "${excluded_remote_names[@]}"; do
    if [ "$remote" = "$excluded_name" ]; then
      excluded=true
      break
    fi
  done
  if [ "$excluded" = true ]; then
    excluded_remotes+=("$remote")
  else
    selected_remotes+=("$remote")
  fi
done

if [ "${#selected_remotes[@]}" -eq 0 ]; then
  echo "错误：所有远程仓库都已被排除。" >&2
  exit 2
fi
```

- [ ] **Step 3: Display and push only selected remotes**

Display `selected_remotes` under `将推送的远程仓库`, display `excluded_remotes` only when non-empty, and change the push loop to:

```bash
for remote in "${selected_remotes[@]}"; do
```

- [ ] **Step 4: Run tests and verify GREEN**

Run: `bash tests/test_push_all_branches_to_all_remotes.sh`

Expected: all 9 tests pass, and every push targets a temporary local bare repository.

### Task 3: Audit and record

**Files:**
- Create: `docs/ai_refactor_records/20260705_exclude_remote_from_push.md`
- Inspect: `push_all_branches_to_all_remotes.sh`
- Inspect: `tests/test_push_all_branches_to_all_remotes.sh`

- [ ] **Step 1: Run syntax and regression checks**

Run: `bash -n push_all_branches_to_all_remotes.sh tests/test_push_all_branches_to_all_remotes.sh`

Run: `bash tests/test_push_all_branches_to_all_remotes.sh`

Expected: syntax succeeds silently and all 9 tests pass.

- [ ] **Step 2: Check safety and scope**

Run: `rg -n -- '--exclude|--force|--mirror|--prune|git push' push_all_branches_to_all_remotes.sh tests/test_push_all_branches_to_all_remotes.sh`

Run: `git diff --check && git status --short`

Expected: production pushes still omit destructive options; only planned files are modified, alongside untouched pre-existing user changes.

- [ ] **Step 3: Write the required refactor record**

Record user intent, scope, Explorer findings, exact changed files, preserved behavior, checks, absence of real remote pushes/builds, issues found, and final `PASS` or `NEEDS_FIX`. Do not restore or alter the user-deleted previous record.
