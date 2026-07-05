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

echo "本地分支（${#branches[@]}）："
printf '  - %s\n' "${branches[@]}"
echo "将推送的远程仓库（${#selected_remotes[@]}）："
printf '  - %s\n' "${selected_remotes[@]}"
if [ "${#excluded_remotes[@]}" -gt 0 ]; then
  echo "已排除的远程仓库（${#excluded_remotes[@]}）："
  printf '  - %s\n' "${excluded_remotes[@]}"
fi

if [ "$dry_run" = false ]; then
  printf '将所有本地分支推送到以上待推送远程仓库，是否继续？[y/N] '
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
  echo "Dry-run：只预览，不更新远程仓库。"
fi

successful_remotes=()
failed_remotes=()
for remote in "${selected_remotes[@]}"; do
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

echo "全部待推送远程仓库处理成功。"
