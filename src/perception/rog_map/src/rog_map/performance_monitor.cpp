#include <rog_map/performance_monitor.hpp>

namespace rog_map {

PerformanceMonitor::ScopedTimer::ScopedTimer(PerformanceMonitor *monitor, double RuntimeStats::*field)
    : monitor_(monitor), field_(field) {
    if (monitor_ && monitor_->enabled() && field_) {
        start_ = std::chrono::steady_clock::now();
    } else {
        monitor_ = nullptr;
        field_ = nullptr;
    }
}

PerformanceMonitor::ScopedTimer::~ScopedTimer() {
    if (!monitor_ || !field_) {
        return;
    }
    const auto elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start_).count();
    monitor_->addElapsed(field_, elapsed);
}

void PerformanceMonitor::configure(const PerformanceConfig &config) {
    close();
    config_ = config;
    resetStats();
    if (!csvEnabled()) {
        return;
    }
    performance_csv_.open(config_.csv_path, std::ios::out | std::ios::trunc);
    map_info_csv_.open(config_.map_info_csv_path, std::ios::out | std::ios::trunc);
}

void PerformanceMonitor::writePerformanceCsvHeader(const std::vector<std::string> &fields) {
    if (!csvEnabled() || !performance_csv_.is_open()) {
        return;
    }
    for (size_t i = 0; i < fields.size(); ++i) {
        performance_csv_ << fields[i];
        if (i + 1 < fields.size()) {
            performance_csv_ << ", ";
        }
    }
    performance_csv_ << '\n';
}

void PerformanceMonitor::writePerformanceCsvRow(const std::vector<double> &values) {
    if (!csvEnabled() || !performance_csv_.is_open()) {
        return;
    }
    for (size_t i = 0; i < values.size(); ++i) {
        performance_csv_ << values[i];
        if (i + 1 < values.size()) {
            performance_csv_ << ", ";
        }
    }
    performance_csv_ << '\n';
}

void PerformanceMonitor::close() {
    if (performance_csv_.is_open()) {
        performance_csv_.close();
    }
    if (map_info_csv_.is_open()) {
        map_info_csv_.close();
    }
}

void PerformanceMonitor::addElapsed(double RuntimeStats::*field, double elapsed_ms) {
    if (!field) {
        return;
    }
    stats_.*field += elapsed_ms;
}

}  // namespace rog_map
