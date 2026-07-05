# 推送脚本排除远程仓库改造记录

## User Intent

为全分支多远程推送脚本增加选项，使用户可以在单次执行中排除指定远程仓库。

## Scope

- 模块：文档 / 实验记录（仓库维护脚本）。
- 目标：新增可重复的 `--exclude <remote>` 参数。
- 运行行为：允许新增参数行为，不改变默认推送行为或机器人运行逻辑。
- 比赛验证逻辑：不涉及。

## Out of Scope

- 不增加通配符、配置文件或远程仓库分组。
- 不修改 push 的 fast-forward、安全或 tag 语义。
- 不执行真实远程推送或项目构建。
- 不恢复或修改用户已删除的旧改造记录。

## Explorer Findings

### Files inspected

- `push_all_branches_to_all_remotes.sh`
- `tests/test_push_all_branches_to_all_remotes.sh`
- `docs/superpowers/specs/2026-07-05-push-all-branches-to-all-remotes-design.md`
- 当前 Git 状态与最近提交

### Active logic path

原脚本仅接受一个可选 `--dry-run` 参数，随后读取全部 remote，并对每个 remote 执行普通 `git push --all`。现有测试使用 `/tmp` 下临时 bare 仓库验证推送，不访问真实远程仓库。

### Data flow

命令行参数 → 排除名称列表 → Git 配置的 remote 列表 → 排除名称合法性检查 → 待推送/已排除分组 → 确认或 dry-run → 仅遍历待推送 remote。

### Risk notes

- 拼写错误若被静默忽略，会导致本应排除的 remote 被推送，因此不存在的名称必须报错。
- 排除全部 remote 时必须在确认和 push 前失败。
- `--dry-run` 与多个 `--exclude` 的参数顺序不应影响解析。
- 不得加入 `--force`、`--mirror` 或 `--prune`。

### Recommended modification boundary

仅修改现有推送脚本和对应集成测试，并新增本记录；不修改业务代码、Git remote 配置或真实远端状态。

## Modifier Changes

### Files changed

- `push_all_branches_to_all_remotes.sh`
- `tests/test_push_all_branches_to_all_remotes.sh`
- `docs/superpowers/specs/2026-07-05-push-all-branches-to-all-remotes-design.md`
- `docs/superpowers/plans/2026-07-05-exclude-remotes-option.md`
- `docs/ai_refactor_records/20260705_exclude_remote_from_push.md`

### Key changes

- 增加可重复的 `--exclude <remote>` 参数，并允许与 `--dry-run` 任意排序。
- 精确验证排除名称，缺值、不存在或排除全部 remote 时在 push 前失败。
- 分开展示待推送和已排除 remote，推送循环仅遍历待推送列表。
- 新增单个排除、多个排除、参数混排和错误输入集成测试。

### Behavior preserved

不传 `--exclude` 时行为保持不变。仍使用普通 `git push [--dry-run] <remote> --all`，不强推、不删除远端引用、不推送 tags；单个待推送 remote 失败后仍继续其余待推送 remote。

### Behavior intentionally adjusted

指定的 remote 不参与确认后的推送或 dry-run。无效排除请求以非零状态退出，防止因拼写错误误推送。

### Notes

测试仅对 `/tmp` 下临时 bare 仓库执行 Git push。未访问当前配置的 Gitee/GitHub remote。

## Auditor Review

### Checks performed

- [x] 关键路径检查
- [x] diff 检查
- [x] grep 检查
- [x] XML / launch / yaml 检查（本任务不涉及）
- [x] 用户允许范围内的测试或静态检查
- [x] 如需构建，已取得用户明确许可（本任务不需要构建）

### Issues found

- 初次审计发现确认和成功文案仍使用“全部远程仓库”，与已排除列表存在歧义；已改为“待推送远程仓库”。
- 未发现破坏性 Git 选项、越界业务修改或真实远程访问。
- 工作区已有 `AGENTS.md` 修改及旧记录删除，本任务未修改或暂存这些用户变更。

### Final result

PASS
