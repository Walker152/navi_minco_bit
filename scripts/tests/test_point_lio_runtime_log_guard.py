#!/usr/bin/env python3
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SOURCE = REPO_ROOT / "src/perception/Point-LIO/src/laserMapping.cpp"


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    brace = source.index("{", start)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[brace + 1 : index]
    raise AssertionError(f"unterminated function: {signature}")


source = SOURCE.read_text(encoding="utf-8")
dump_body = function_body(source, "inline void dump_lio_state_to_log(FILE * fp_)")
initialize_body = function_body(source, "void initialize()")

assert "if (fp_ == nullptr)" in dump_body, "dump must reject a null FILE pointer"
assert "std::filesystem::create_directories" in initialize_body, "log directory must be created"

runtime_guard = initialize_body.index("if (runtime_pos_log)")
create_directory = initialize_body.index("std::filesystem::create_directories", runtime_guard)
open_position_log = initialize_body.index("fopen", runtime_guard)
assert create_directory < open_position_log, "directory creation must happen before fopen"

open_failure = initialize_body.index("if (!fp_ || !fout_out || !fout_imu_pbp)", open_position_log)
disable_logging = initialize_body.index("runtime_pos_log = false", open_failure)
assert disable_logging > open_failure, "failed log initialization must disable runtime logging"

print("PASS: Point-LIO runtime log initialization is guarded")
