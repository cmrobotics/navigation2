#include <cstdint>
#include <cmath>
#include <cstring>
#include <memory>

#include "nav2_amcl/sensors/external_pose/external_pose.hpp"

namespace nav2_amcl 
{

// Unconstrained (input args can be of any value, even more/less than 2*pi) angular distance in range [-pi, pi)
// Source: https://stackoverflow.com/a/28037434/13167995
double angular_difference(double angle1, double angle2){
  double diff = fmod(angle2 - angle1 + M_PI_2, M_PI) - M_PI_2;
  
  return diff < -M_PI_2 ? diff + M_PI : diff;
}

double externalPoseSensorFunction(ExternalPoseMeasument * data, pf_sample_set_t * set) {
    double total_weight = 0.0;
    double total_dist_prob = 0.0;

    double covariance_determinant = (data->cov_matrix[0] * data->cov_matrix[4] * data->cov_matrix[8]);
    double max_particle_likelihood = 1/sqrt(pow(2*M_PI, 3) * covariance_determinant);

    
    for(int i = 0; i < set->sample_count; i++)
    {
        // double a1 = angleutils::angle_diff(set->samples[i].pose.v[2], pf->ext_yaw);
        // double a2 = angular_difference(set->samples[i].pose.v[2], pf->ext_yaw);

        // fprintf(stderr, "AMCL: angle diff compare: mine - %f, angleutils - %f\n", a1, a2);

        // See Improved LiDAR Probabilistic Localization for Autonomous Vehicles Using GNSS, #3.2 for details
        double distance = pow(set->samples[i].pose.v[0] - data->x, 2) / data->cov_matrix[0] + 
                            pow(set->samples[i].pose.v[1]- data->y, 2) / data->cov_matrix[4] + 
                            + pow(angular_difference(set->samples[i].pose.v[2], data->yaw), 2) / data->cov_matrix[8]; //

        double ext_pose_likelihood = max_particle_likelihood*exp(-1*distance/2);

        // fprintf(stderr, "AMCL: weights: laser - %f, ext_pose - %f\n", set->samples[i].weight * 50, ext_pose_likelihood);

        // See Improved LiDAR Probabilistic Localization for Autonomous Vehicles Using GNSS, #3.3 for details
        set->samples[i].weight = set->samples[i].weight * 50.0 + ext_pose_likelihood;
        // auto w = set->samples[i].weight * 50.0 + ext_pose_likelihood;

        // fprintf(stderr, "AMCL: weight - %f, pose - [%f, %f, %f]\n", set->samples[i].weight, set->samples[i].pose.v[0], set->samples[i].pose.v[1], set->samples[i].pose.v[2]);


        total_weight += set->samples[i].weight;
        // total_weight += w; // set->samples[i].weight;
        total_dist_prob += ext_pose_likelihood;
    }

    set->total_dist_prob_normalized = total_dist_prob / set->sample_count;

    fprintf(stderr, "AMCL: total_weight - %f, total_dist_prob - %f\n", total_weight, set->total_dist_prob_normalized);

    return total_weight;
}

void externalPoseSensorUpdate(pf_t * pf, std::shared_ptr<ExternalPoseBuffer> buffer, double query_sec) {
    pf->ext_pose_is_valid = 0;
    // 2. call sensorFunction

    ExternalPoseMeasument tmp;

    // TODO: that function may be used in different threads (laser callback and here), it should be thread safe
    if(buffer->findClosestMeasurement(query_sec, tmp)){
        pf->ext_pose_is_valid = 1;

        pf->ext_x = tmp.x;
        pf->ext_y = tmp.y;
        pf->ext_yaw = tmp.yaw;

        memcpy(pf->cov_matrix, tmp.cov_matrix, 9*sizeof(double));
        memcpy(pf->eigen_matrix, tmp.eigen_matrix, 9*sizeof(double));

        pf_update_sensor(pf, (pf_sensor_model_fn_t) externalPoseSensorFunction, &tmp);
    }
}

void
ExternalPoseBuffer::addMeasurement(const ExternalPoseMeasument measurement)
{
    if(buffer_.size() >= max_buff_size_) buffer_.erase(buffer_.begin());

    buffer_.push_back(measurement);
}

bool
ExternalPoseBuffer::findClosestMeasurement(double query_time_sec, ExternalPoseMeasument& out_measurement) const
{
    uint32_t min_abs_diff = INT32_MAX;
    size_t min_diff_idx = 0;
    for(size_t i = 0; i < buffer_.size(); i++){
        uint32_t abs_diff = std::abs(buffer_[i].time_sec - query_time_sec);

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