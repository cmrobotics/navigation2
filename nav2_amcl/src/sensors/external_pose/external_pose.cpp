#include <cstdint>
#include <cmath>

#include "nav2_amcl/sensors/external_pose/external_pose.hpp"

namespace nav2_amcl 
{

    // TODO: allocate memory for max_storage_size_ elements

void
ExternalPoseBuffer::addMeasurement(const ExternalPoseMeasument measurement)
{
    if(buffer_.size() >= max_buff_size_) buffer_.erase(buffer_.begin());

    buffer_.push_back(measurement);
}

bool
ExternalPoseBuffer::findClosestMeasurement(double query_time_sec, ExternalPoseMeasument& out_measurement) const
{
    double min_abs_diff = std::numeric_limits<double>::max();
    size_t min_diff_idx = 0;
    for(size_t i = 0; i < buffer_.size(); i++){
        double abs_diff = std::abs(buffer_[i].time_sec - query_time_sec);

        if(abs_diff < min_abs_diff){
            min_abs_diff = abs_diff;
            min_diff_idx = i;
        }
    }

    if(min_abs_diff < search_tolerance_sec_){
        out_measurement = buffer_.at(min_diff_idx);
        return true;
    } else {
        return false;
    }

}

}