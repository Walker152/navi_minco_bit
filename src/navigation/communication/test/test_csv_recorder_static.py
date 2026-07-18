#!/usr/bin/env python3
"""Static checks for the communication CSV recorder.

These checks intentionally avoid compiling the ROS 2 package because repository policy
requires explicit user approval before any build.
"""

import ast
import re
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]


def main() -> None:
    header = (PACKAGE / "include" / "csv_recorder.hpp").read_text()
    source = (PACKAGE / "src" / "csv_recorder.cpp").read_text()
    com = (PACKAGE / "include" / "com.hpp").read_text()
    com_source = (PACKAGE / "src" / "com.cpp").read_text()
    ros_interface = (PACKAGE / "include" / "com_interface_ros.hpp").read_text()

    assert "/tmp/communication_logs" in source
    assert "stream_.flush()" in source
    assert "communication.enable_performance_diagnostics" in ros_interface
    assert '"communication.enable_performance_diagnostics", false' in ros_interface
    assert "Communication::init(performance_diagnostics_enabled_)" in ros_interface
    assert "CsvRecorder::initialize(performance_diagnostics_enabled)" in com_source

    # CSV recording is explicit in the NAV path. The generic send template must
    # not record behavior packets (or any other packet type) by accident.
    assert "CsvRecorder::record(data_packet, packet_type, send_result)" not in com
    assert "CsvRecorder::record(target, flag, diagnostics)" in ros_interface
    assert "const BehaviorData &" not in header
    assert "recordBehavior" not in source
    assert "BEHAVIOR_DATA" not in source

    # The same diagnostics switch gates CSV output and TX/RX frequency output.
    assert "if (performance_diagnostics_enabled_)" in ros_interface
    assert "Communication::printPacketRates()" in ros_interface
    assert "diagnostics_enabled_" in com
    assert "recordTxPacket(packet_type, send_result)" in com
    assert "recordRxPacket(header->packet_type)" in com_source

    chassis_fields = (
        "vx_mps", "vy_mps", "vw_rpm", "current_yaw", "current_vx", "current_vy",
        "current_vw", "fx_global", "fy_global", "fw_global", "use_speed_control", "delta_yaw",
    )
    diagnostics_fields = (
        "odom_stamp_sec", "odom_receive_stamp_sec", "odom_age_ms",
        "chassis_sample_stamp_sec", "chassis_imu_yaw", "history_size",
        "history_span_ms", "odom_minus_oldest_ms", "odom_minus_newest_ms",
        "best_signed_dt_ms", "match_found", "delta_candidate",
        "delta_yaw_initialized", "delta_last_update_age_ms",
        "consecutive_match_failures", "self_packet_count",
        "self_packet_age_ms", "offline_publish_cost_us",
    )
    for field in chassis_fields + diagnostics_fields:
        assert field in source, f"missing CSV field mapping: {field}"

    header_write = re.search(r'stream_ << ("system_time.*?);', source, re.DOTALL)
    assert header_write, "CSV header write was not found"
    csv_header = "".join(ast.literal_eval(token) for token in re.findall(r'"(?:[^"\\]|\\.)*"', header_write.group(1)))
    columns = csv_header.rstrip("\n").split(",")
    expected_columns = (
        "system_time", "elapsed_ms", "send_result", "send_success",
        *chassis_fields, *diagnostics_fields,
    )
    assert tuple(columns) == expected_columns

    assert "const ChassisTarget &" in header
    assert "const DeltaYawDiagnostics &" in header


if __name__ == "__main__":
    main()
