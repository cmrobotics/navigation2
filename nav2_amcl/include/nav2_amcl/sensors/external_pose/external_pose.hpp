#ifndef NAV2_AMCL__SENSORS__EXTERNAL_POSE_HPP_
#define NAV2_AMCL__SENSORS__EXTERNAL_POSE_HPP_

#include <vector>
#include <memory>

namespace nav2_amcl
{

struct ExternalPoseMeasument {
    double x, y, z; // position
    double qx, qy, qz, qw; // orientation

    double cov_matrix[9];
    double eigen_matrix[9];

    double time_sec; // time the measurement was made

    ExternalPoseMeasument(double x, double y, double z, double qx, double qy, double qz, double qw, const double cov_matrix[], const double eigen_matrix[], double time_sec):
        x(x), y(y), z(z), qx(qx), qy(qy), qz(qz), qw(qw), time_sec(time_sec) {
            for(size_t i = 0; i < 9; i++){
                this->cov_matrix[i] = cov_matrix[i];
                this->eigen_matrix[i] = eigen_matrix[i];
            }
        };
    
    ExternalPoseMeasument(): x(0), y(0), z(0), qx(0), qy(0), qz(0), qw(0), time_sec(0) {
        for(size_t i = 0; i < 9; i++){
            this->cov_matrix[i] = 0;
            this->eigen_matrix[i] = 0;
        }
    };
};

class ExternalPoseBuffer {

public:

// TODO: allocate memory for max_storage_size_ elements
ExternalPoseBuffer(double search_tolerance_sec): search_tolerance_sec_(search_tolerance_sec) {}

/** 
* @brief Add new measurement to the buffer. If buffer size grown larger than max_storage_size_, pop oldest measurement
* @param measurement external pose measurement
* @return
*/
void addMeasurement(const ExternalPoseMeasument measurement);

/** 
* @brief Find closest external pose measurement to the query time, but not further than threshold
* @param query_time_sec time to look closest external pose for i.e. latest LaserScan reading time
* @param out_measurement found closest measurement
* @return True, if closest measurement found, False othervise
*/
bool findClosestMeasurement(double query_time_sec, ExternalPoseMeasument& out_measurement) const;

private:

// how large can be the time difference between corresponding external pose measurement and laser scan
const double search_tolerance_sec_;

const size_t max_buff_size_ = 10;
std::vector<ExternalPoseMeasument> buffer_ = {};

};

}

#endif // NAV2_AMCL__SENSORS__EXTERNAL_POSE_HPP_