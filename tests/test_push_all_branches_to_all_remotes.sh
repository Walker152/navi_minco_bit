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

  (cd "$source" && "$SCRIPT" --dry-run) >/dev/null || fail "dry-run failed"
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

  printf 'y\n' | (cd "$source" && "$SCRIPT") >/dev/null || fail "confirmed push failed"
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

  printf 'n\n' | (cd "$source" && "$SCRIPT") >/dev/null || fail "cancelled push failed"
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

run_test "dry-run leaves remotes unchanged" test_dry_run_does_not_update_remotes
run_test "confirmed push updates all remotes" test_confirmed_push_updates_all_remotes
run_test "rejected confirmation does not push" test_rejected_confirmation_does_not_push
run_test "failed remote does not block later remote" test_failed_remote_does_not_block_later_remote
run_test "invalid argument fails" test_invalid_argument_fails
run_test "missing remote fails" test_missing_remote_fails
run_test "excluded remote is not pushed" test_excluded_remote_is_not_pushed
run_test "repeated exclusions combine with dry-run" test_repeated_exclusions_and_dry_run_order
run_test "invalid exclusions fail before push" test_invalid_exclusions_fail_before_push

echo "All $pass_count tests passed."
