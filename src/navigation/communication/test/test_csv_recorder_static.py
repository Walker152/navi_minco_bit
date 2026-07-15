#!/usr/bin/env python3
"""Static checks for the communication CSV recorder.

These checks intentionally avoid compiling the ROS 2 package because repository policy
requires explicit user approval before any build.
"""

from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]


def main() -> None:
    header = (PACKAGE / "include" / "csv_recorder.hpp").read_text()
    source = (PACKAGE / "src" / "csv_recorder.cpp").read_text()
    com = (PACKAGE / "include" / "com.hpp").read_text()

    assert "/tmp/communication_logs" in source
    assert "stream_.flush()" in source
    assert "CsvRecorder::record(data_packet, packet_type, send_result)" in com
    assert "CsvRecorder::initialize()" in (PACKAGE / "src" / "com.cpp").read_text()

    chassis_fields = (
        "vx_mps", "vy_mps", "vw_rpm", "current_yaw", "current_vx", "current_vy",
        "current_vw", "fx_global", "fy_global", "fw_global", "use_speed_control", "delta_yaw",
    )
    behavior_fields = (
        "pitch_mode", "desire_stance", "desire_lifter_pos", "scan_yaw_min_deg",
        "scan_yaw_max_deg", "ammo_purchase_request", "revive_request",
        "remote_revive_request", "remote_ammo_request", "remote_health_request",
        "use_limited_scan", "not_aim_enemy", "use_capacitor",
    )
    for field in chassis_fields + behavior_fields:
        assert field in source, f"missing CSV field mapping: {field}"

    assert "const ChassisTarget &" in header
    assert "const BehaviorData &" in header


if __name__ == "__main__":
    main()
