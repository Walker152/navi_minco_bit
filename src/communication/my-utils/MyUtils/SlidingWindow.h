#include "ros/ros.h"
#include "vector"

class SlidingWindowFilter {

public:
    SlidingWindowFilter(){};
    void init(int size) {
        m_size = size;
        m_sum = 0;
        m_data.reserve(size);
    }

    double filter(double value) {
        if (m_data.size() == m_size) {
            m_sum -= m_data.front();
            m_data.erase(m_data.begin());

        }
        m_data.push_back(value);
        m_sum += value;
        return m_sum / m_data.size();
    }

private:
    int m_size;                // 滤波器窗口大小
    std::vector<double> m_data;     // 数据存储数组
    double m_sum;              // 数据累加和
};
