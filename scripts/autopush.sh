#!/usr/bin/env bash
set -uo pipefail

usage() {
  echo "用法：$0 [--dry-run] [--exclude <remote>]..."
}

dry_run=false
excluded_remote_names=()
while [ "$#" -gt 0 ]; do
  case $1 in
    --dry-run)
      dry_run=true
      shift
      ;;
    --exclude)
      if [ "$#" -lt 2 ] || [[ $2 == --* ]]; then
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

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "错误：当前目录不属于 Git 仓库。" >&2
  exit 1
fi

repo_root="$(git rev-parse --show-toplevel)"
repositories=("$repo_root")
repository_labels=(".")
uninitialized_submodules=()
conflicted_submodules=()
github_blob_limit_bytes=$((100 * 1024 * 1024))

submodule_status="$(git -C "$repo_root" submodule status --recursive 2>/dev/null || true)"
while IFS= read -r status_line; do
  [ -n "$status_line" ] || continue
  status_prefix=${status_line:0:1}
  status_body=${status_line:1}
  read -r _ submodule_path _ <<<"$status_body"
  [ -n "${submodule_path-}" ] || continue

  case "$status_prefix" in
    -)
      uninitialized_submodules+=("$submodule_path")
      continue
      ;;
    U)
      conflicted_submodules+=("$submodule_path")
      continue
      ;;
  esac

  submodule_repository="$repo_root/$submodule_path"
  if git -C "$submodule_repository" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    # Recursive status lists parents before their nested children. Prepending
    # therefore makes every child repository publish before its parent.
    repositories=("$submodule_repository" "${repositories[@]}")
    repository_labels=("$submodule_path" "${repository_labels[@]}")
  else
    uninitialized_submodules+=("$submodule_path")
  fi
done <<<"$submodule_status"

if [ "${#uninitialized_submodules[@]}" -gt 0 ]; then
  for submodule_path in "${uninitialized_submodules[@]}"; do
    echo "警告：子模块未初始化，已跳过：$submodule_path" >&2
  done
fi
if [ "${#conflicted_submodules[@]}" -gt 0 ]; then
  for submodule_path in "${conflicted_submodules[@]}"; do
    echo "警告：子模块状态冲突，已跳过：$submodule_path" >&2
  done
fi

remote_is_excluded() {
  local candidate=$1
  local excluded_name
  for excluded_name in "${excluded_remote_names[@]}"; do
    if [ "$candidate" = "$excluded_name" ]; then
      return 0
    fi
  done
  return 1
}

remote_is_github() {
  local repository=$1
  local remote=$2
  local remote_url

  remote_url="$(git -C "$repository" remote get-url "$remote" 2>/dev/null || true)"
  [[ $remote_url == *github.com:* || $remote_url == *github.com/* ]]
}

find_oversized_outgoing_blobs() {
  local repository=$1
  local remote=$2
  local local_sha=$3

  git -C "$repository" rev-list --objects "$local_sha" --not --remotes="$remote" |
    git -C "$repository" cat-file \
      --batch-check='%(objecttype) %(objectsize) %(objectname) %(rest)' |
    awk -v limit="$github_blob_limit_bytes" '
      $1 == "blob" && $2 >= limit {
        size = $2
        object = $3
        $1 = $2 = $3 = ""
        sub(/^ +/, "")
        printf "%s\t%s\t%s\n", size, object, $0
      }
    '
}

for excluded_name in "${excluded_remote_names[@]}"; do
  found=false
  for repository in "${repositories[@]}"; do
    while IFS= read -r remote; do
      if [ "$remote" = "$excluded_name" ]; then
        found=true
        break 2
      fi
    done < <(git -C "$repository" remote)
  done
  if [ "$found" = false ]; then
    echo "错误：未找到要排除的远程仓库：$excluded_name" >&2
    exit 2
  fi
done

total_selected_remotes=0
echo "待处理仓库（${#repositories[@]}）："
for repository_index in "${!repositories[@]}"; do
  repository=${repositories[$repository_index]}
  repository_label=${repository_labels[$repository_index]}
  mapfile -t branches < <(git -C "$repository" for-each-ref --format='%(refname:short)' refs/heads/)
  mapfile -t remotes < <(git -C "$repository" remote)

  selected_remotes=()
  for remote in "${remotes[@]}"; do
    if ! remote_is_excluded "$remote"; then
      selected_remotes+=("$remote")
    fi
  done
  total_selected_remotes=$((total_selected_remotes + ${#selected_remotes[@]}))

  echo "  - $repository_label"
  if [ "${#branches[@]}" -gt 0 ]; then
    printf '      分支：%s\n' "${branches[*]}"
  else
    echo "      分支：无"
  fi
  if [ "${#selected_remotes[@]}" -gt 0 ]; then
    printf '      远端：%s\n' "${selected_remotes[*]}"
  else
    echo "      远端：无（全部被排除或未配置）"
  fi
done

if [ "$total_selected_remotes" -eq 0 ]; then
  echo "错误：所有远程仓库都已被排除。" >&2
  exit 2
fi

if [ "$dry_run" = false ]; then
  printf '将逐分支执行安全的快进推送；远端领先或分叉的分支会被跳过。是否继续？[y/N] '
  read -r answer
  case ${answer-} in
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
  echo "Dry-run：会刷新本地远端引用并预览 push，但不会更新远程仓库。"
fi

successful_pushes=()
up_to_date_branches=()
skipped_branches=()
failed_operations=()
submodule_sync_incomplete=false

for repository_index in "${!repositories[@]}"; do
  repository=${repositories[$repository_index]}
  repository_label=${repository_labels[$repository_index]}

  if [ "$repository_label" = "." ] && [ "$submodule_sync_incomplete" = true ]; then
    echo "错误：[.] 子仓库未全部同步，父仓库已跳过，避免发布不可获取的子模块提交。" >&2
    failed_operations+=(".:(子仓库未同步，父仓库已跳过)")
    continue
  fi

  repository_failure_count_before=${#failed_operations[@]}
  repository_skip_count_before=${#skipped_branches[@]}
  mapfile -t branches < <(git -C "$repository" for-each-ref --format='%(refname:short)' refs/heads/)
  mapfile -t remotes < <(git -C "$repository" remote)

  if [ "${#branches[@]}" -eq 0 ]; then
    echo "警告：[$repository_label] 没有本地分支，已跳过。" >&2
    if [ "$repository_label" != "." ]; then
      submodule_sync_incomplete=true
    fi
    continue
  fi

  for remote in "${remotes[@]}"; do
    if remote_is_excluded "$remote"; then
      continue
    fi

    echo
    echo "==> [$repository_label] 检查远端 $remote"
    if ! git -C "$repository" fetch --prune --no-tags "$remote" \
      "+refs/heads/*:refs/remotes/$remote/*"; then
      echo "错误：[$repository_label] 无法刷新远端 $remote。" >&2
      failed_operations+=("$repository_label:$remote(fetch)")
      continue
    fi

    for branch in "${branches[@]}"; do
      branch_ref="refs/heads/$branch"
      remote_ref="refs/remotes/$remote/$branch"
      local_sha="$(git -C "$repository" rev-parse --verify "$branch_ref")"

      if remote_sha="$(git -C "$repository" show-ref --verify --hash "$remote_ref" 2>/dev/null)"; then
        if [ "$local_sha" = "$remote_sha" ]; then
          echo "  [$branch] 已是最新。"
          up_to_date_branches+=("$repository_label:$remote/$branch")
          continue
        fi

        if git -C "$repository" merge-base --is-ancestor "$remote_sha" "$local_sha"; then
          push_reason="快进"
        elif git -C "$repository" merge-base --is-ancestor "$local_sha" "$remote_sha"; then
          echo "  [$branch] 远端领先，已跳过。" >&2
          skipped_branches+=("$repository_label:$remote/$branch(远端领先)")
          continue
        else
          echo "  [$branch] 本地与远端分叉，已跳过。" >&2
          skipped_branches+=("$repository_label:$remote/$branch(分叉)")
          continue
        fi
      else
        push_reason="创建"
      fi

      if remote_is_github "$repository" "$remote"; then
        if ! oversized_blob_output="$(
          find_oversized_outgoing_blobs "$repository" "$remote" "$local_sha"
        )"; then
          echo "错误：[$repository_label] $remote/$branch 大文件检查失败，已停止该推送。" >&2
          failed_operations+=("$repository_label:$remote/$branch(大文件检查失败)")
          continue
        fi

        oversized_blobs=()
        if [ -n "$oversized_blob_output" ]; then
          mapfile -t oversized_blobs <<<"$oversized_blob_output"
        fi
        if [ "${#oversized_blobs[@]}" -gt 0 ]; then
          echo "错误：[$repository_label] $remote/$branch 的待推送历史包含 GitHub 不接受的 100 MiB 及以上文件：" >&2
          for oversized_blob in "${oversized_blobs[@]}"; do
            IFS=$'\t' read -r blob_size blob_object blob_path <<<"$oversized_blob"
            blob_size_mib=$(( (blob_size + 1024 * 1024 - 1) / (1024 * 1024) ))
            printf '    %s (%s MiB, %s)\n' \
              "${blob_path:-<路径不可用>}" "$blob_size_mib" "$blob_object" >&2
          done
          echo "  [$branch] 已在上传前跳过；请清理历史或改用 Git LFS。" >&2
          failed_operations+=("$repository_label:$remote/$branch(大文件)")
          continue
        fi
      fi

      push_refspec="$branch_ref:refs/heads/$branch"
      echo "  [$branch] $push_reason推送。"
      if git -C "$repository" push "${push_options[@]}" "$remote" "$push_refspec"; then
        successful_pushes+=("$repository_label:$remote/$branch($push_reason)")
      else
        echo "错误：[$repository_label] $remote/$branch 推送失败。" >&2
        failed_operations+=("$repository_label:$remote/$branch(push)")
      fi
    done
  done

  if [ "$repository_label" != "." ] && {
    [ "${#failed_operations[@]}" -gt "$repository_failure_count_before" ] ||
      [ "${#skipped_branches[@]}" -gt "$repository_skip_count_before" ];
  }; then
    submodule_sync_incomplete=true
  fi

  for remote in "${remotes[@]}"; do
    if remote_is_excluded "$remote"; then
      continue
    fi

    echo
    echo "==> [$repository_label] 检查远端 $remote"
    if ! git -C "$repository" fetch --prune --no-tags "$remote" \
      "+refs/heads/*:refs/remotes/$remote/*"; then
      echo "错误：[$repository_label] 无法刷新远端 $remote。" >&2
      failed_operations+=("$repository_label:$remote(fetch)")
      continue
    fi

    for branch in "${branches[@]}"; do
      branch_ref="refs/heads/$branch"
      remote_ref="refs/remotes/$remote/$branch"
      local_sha="$(git -C "$repository" rev-parse --verify "$branch_ref")"

      if remote_sha="$(git -C "$repository" show-ref --verify --hash "$remote_ref" 2>/dev/null)"; then
        if [ "$local_sha" = "$remote_sha" ]; then
          echo "  [$branch] 已是最新。"
          up_to_date_branches+=("$repository_label:$remote/$branch")
          continue
        fi

        if git -C "$repository" merge-base --is-ancestor "$remote_sha" "$local_sha"; then
          push_reason="快进"
        elif git -C "$repository" merge-base --is-ancestor "$local_sha" "$remote_sha"; then
          echo "  [$branch] 远端领先，已跳过。" >&2
          skipped_branches+=("$repository_label:$remote/$branch(远端领先)")
          continue
        else
          echo "  [$branch] 本地与远端分叉，已跳过。" >&2
          skipped_branches+=("$repository_label:$remote/$branch(分叉)")
          continue
        fi
      else
        push_reason="创建"
      fi

      push_refspec="$branch_ref:refs/heads/$branch"
      echo "  [$branch] $push_reason推送。"
      if git -C "$repository" push "${push_options[@]}" "$remote" "$push_refspec"; then
        successful_pushes+=("$repository_label:$remote/$branch($push_reason)")
      else
        echo "错误：[$repository_label] $remote/$branch 推送失败。" >&2
        failed_operations+=("$repository_label:$remote/$branch(push)")
      fi
    done
  done
done

echo
echo "同步结果："
if [ "${#successful_pushes[@]}" -gt 0 ]; then
  printf '  推送：%s\n' "${successful_pushes[*]}"
fi
if [ "${#up_to_date_branches[@]}" -gt 0 ]; then
  printf '  已同步：%s\n' "${up_to_date_branches[*]}"
fi
if [ "${#skipped_branches[@]}" -gt 0 ]; then
  printf '  已跳过：%s\n' "${skipped_branches[*]}" >&2
fi
if [ "${#failed_operations[@]}" -gt 0 ]; then
  printf '  失败：%s\n' "${failed_operations[*]}" >&2
  exit 1
fi

echo "处理完成；跳过的分支未修改远端历史。"
