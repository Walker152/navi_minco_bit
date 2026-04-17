#!/usr/bin/env bash
set -euo pipefail

# Configure VS Code shortcuts + settings for clang-format formatting.
# - Binds Ctrl+I to Format Document (C/C++ only)
# - Binds Ctrl+Shift+I to run the workspace task: "Format: src (clang-format)"
# - Sets default formatter for C/C++ to xaver.clang-format and forces style=file
#
# Usage:
#   bash scripts/setup_vscode_clangformat_shortcuts.sh
#
# Optional env overrides:
#   VSCODE_USER_DIR="$HOME/.config/Code/User"      # default
#   WORKSPACE_DIR="/path/to/workspace"            # default: script's repo root

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR_DEFAULT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-$WORKSPACE_DIR_DEFAULT}"

VSCODE_USER_DIR_DEFAULT="$HOME/.config/Code/User"
VSCODE_USER_DIR="${VSCODE_USER_DIR:-$VSCODE_USER_DIR_DEFAULT}"

export WORKSPACE_DIR
export VSCODE_USER_DIR

KEYBINDINGS_JSON="$VSCODE_USER_DIR/keybindings.json"
USER_SETTINGS_JSON="$VSCODE_USER_DIR/settings.json"

WORKSPACE_SETTINGS_JSON="$WORKSPACE_DIR/.vscode/settings.json"
WORKSPACE_TASKS_JSON="$WORKSPACE_DIR/.vscode/tasks.json"

TS="$(date +%Y%m%d_%H%M%S)"

backup_file() {
  local path="$1"
  if [[ -f "$path" ]]; then
    cp -a "$path" "${path}.bak.${TS}"
  fi
}

mkdir -p "$VSCODE_USER_DIR"
mkdir -p "$WORKSPACE_DIR/.vscode"

backup_file "$KEYBINDINGS_JSON"
backup_file "$USER_SETTINGS_JSON"
backup_file "$WORKSPACE_SETTINGS_JSON"
backup_file "$WORKSPACE_TASKS_JSON"

python3 - <<'PY'
import json
import os
from pathlib import Path

workspace_dir = Path(os.environ["WORKSPACE_DIR"])
vscode_user_dir = Path(os.environ["VSCODE_USER_DIR"])

keybindings_path = vscode_user_dir / "keybindings.json"
user_settings_path = vscode_user_dir / "settings.json"
workspace_settings_path = workspace_dir / ".vscode" / "settings.json"
workspace_tasks_path = workspace_dir / ".vscode" / "tasks.json"

# --- Helpers ---

def load_json(path: Path, default):
    if not path.exists():
        return default
    try:
        text = path.read_text(encoding="utf-8")
        if not text.strip():
            return default
        return json.loads(text)
    except Exception:
        return default


def write_json(path: Path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def upsert_keybinding(bindings, *, key, command, when=None, args=None):
    # Remove exact duplicates for (key, command)
    new_bindings = []
    for b in bindings:
        if not isinstance(b, dict):
            continue
        if b.get("key") == key and b.get("command") == command:
            continue
        new_bindings.append(b)

    entry = {"key": key, "command": command}
    if when is not None:
        entry["when"] = when
    if args is not None:
        entry["args"] = args

    new_bindings.append(entry)
    return new_bindings


def merge_formatter_settings(settings: dict):
    # Extension: xaver.clang-format
    settings["clang-format.style"] = "file"
    settings.setdefault("[cpp]", {})
    settings.setdefault("[c]", {})
    if isinstance(settings["[cpp]"], dict):
        settings["[cpp]"]["editor.defaultFormatter"] = "xaver.clang-format"
    if isinstance(settings["[c]"], dict):
        settings["[c]"]["editor.defaultFormatter"] = "xaver.clang-format"
    return settings


# --- 1) User keybindings ---
bindings = load_json(keybindings_path, default=[])
if not isinstance(bindings, list):
    bindings = []

bindings = upsert_keybinding(
    bindings,
    key="ctrl+i",
    command="editor.action.formatDocument",
    when="editorTextFocus && !editorReadonly && editorLangId =~ /^(cpp|c|cuda-cpp)$/",
)

bindings = upsert_keybinding(
    bindings,
    key="ctrl+shift+i",
    command="workbench.action.tasks.runTask",
    args="Format: src (clang-format)",
    when="workspaceFolderCount == 1",
)

write_json(keybindings_path, bindings)

# --- 2) User settings ---
user_settings = load_json(user_settings_path, default={})
if not isinstance(user_settings, dict):
    user_settings = {}
user_settings = merge_formatter_settings(user_settings)
write_json(user_settings_path, user_settings)

# --- 3) Workspace settings (keep existing keys, add formatter defaults) ---
workspace_settings = load_json(workspace_settings_path, default={})
if not isinstance(workspace_settings, dict):
    workspace_settings = {}
workspace_settings = merge_formatter_settings(workspace_settings)
write_json(workspace_settings_path, workspace_settings)

# --- 4) Workspace task (idempotent) ---
tasks = load_json(workspace_tasks_path, default={"version": "2.0.0", "tasks": []})
if not isinstance(tasks, dict):
    tasks = {"version": "2.0.0", "tasks": []}

existing_tasks = tasks.get("tasks")
if not isinstance(existing_tasks, list):
    existing_tasks = []

label = "Format: src (clang-format)"
command = "find src -type d -name 'third_party' -prune -o -type f \\( -name '*.cpp' -o -name '*.hpp' \\) -print0 | xargs -0 clang-format -i -style=file"

# Remove tasks with same label
existing_tasks = [t for t in existing_tasks if not (isinstance(t, dict) and t.get("label") == label)]
existing_tasks.append(
    {
        "label": label,
        "type": "shell",
        "command": command,
        "options": {"cwd": "${workspaceFolder}"},
        "problemMatcher": [],
    }
)

tasks["version"] = tasks.get("version") or "2.0.0"
tasks["tasks"] = existing_tasks
write_json(workspace_tasks_path, tasks)

print("OK")
PY