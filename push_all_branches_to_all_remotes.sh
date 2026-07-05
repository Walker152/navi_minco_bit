#!/usr/bin/env bash
set -uo pipefail

usage() {
  echo "用法：$0 [--dry-run]"
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
