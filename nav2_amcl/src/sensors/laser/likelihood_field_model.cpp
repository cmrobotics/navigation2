/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <math.h>
#include <assert.h>

#include "nav2_amcl/sensors/laser/laser.hpp"

namespace nav2_amcl
{

LikelihoodFieldModel::LikelihoodFieldModel(
  double z_hit, double z_rand, double sigma_hit,
  double max_occ_dist, size_t max_beams, map_t * map, double importance_factor)
: Laser(max_beams, map)
{
  z_hit_ = z_hit;
  z_rand_ = z_rand;
  sigma_hit_ = sigma_hit;
  importance_factor_ = importance_factor;
  map_update_cspace(map, max_occ_dist);
}

double
LikelihoodFieldModel::sensorFunction(LaserData * data, pf_sample_set_t * set)
{
  LikelihoodFieldModel * self;
  int i, j, step;
  double z, pz;
  double p, max_p;
  double obs_range, obs_bearing;
  double total_weight;
  double map_range;
  pf_sample_t * sample;
  pf_vector_t pose;
  pf_vector_t hit;
  pf_vector_t perfect_hit;

  self = reinterpret_cast<LikelihoodFieldModel *>(data->laser);

  total_weight = 0.0;

  // Pre-compute a couple of things
  double z_hit_denom = 2 * self->sigma_hit_ * self->sigma_hit_;
  double z_rand_component = self->z_rand_ * (1.0 / data->range_max);
  
  double max_pz = self->z_hit_ + z_rand_component;
  double max_pz_normalized = max_pz * max_pz * max_pz;

  // Compute the sample weights
  for (j = 0; j < set->sample_count; j++) {
    sample = set->samples + j;
    pose = sample->pose;

    // Take account of the laser pose relative to the robot
    pose = pf_vector_coord_add(self->laser_pose_, pose);

    p = 1.0;
    max_p = 1.0;


    step = (data->range_count - 1) / (self->max_beams_ - 1);

    // Step size must be at least 1
    if (step < 1) {
      step = 1;
    }

    for (i = 0; i < data->range_count; i += step) {
      obs_range = data->ranges[i][0];
      obs_bearing = data->ranges[i][1];

      // This model ignores max range readings
      if (obs_range >= data->range_max) {
        continue;
      }

      // Check for NaN
      if (obs_range != obs_range) {
        continue;
      }

      pz = 0.0;

      // 1. calc range for map range (i.e. where do we expect laser to hit)
      // 2 calc map range hit pose 
      // 3. if it is occupied,  add zero a distance from laser hit to map hit

      map_range = map_calc_range(
        self->map_, pose.v[0], pose.v[1],
        pose.v[2] + obs_bearing, data->range_max);

      perfect_hit.v[0] = pose.v[0] + map_range * cos(pose.v[2] + obs_bearing);
      perfect_hit.v[1] = pose.v[1] + map_range * sin(pose.v[2] + obs_bearing);

      // Calc perfect hit point in map coords
      int perfect_mi, perfect_mj;
      perfect_mi = MAP_GXWX(self->map_, perfect_hit.v[0]);
      perfect_mj = MAP_GYWY(self->map_, perfect_hit.v[1]);

      // Check if perfect hit is occupied
      // fprintf(stderr, "checking perfect hit occupancy\n");
      if (MAP_VALID(self->map_, perfect_mi, perfect_mj)) {
        if (self->map_->cells[MAP_INDEX(self->map_, perfect_mi, perfect_mj)].occ_state == 1) {
          // calculate score based on perfect hit
          max_p += max_pz_normalized;
        }
      }

      // Compute the endpoint of the beam
      hit.v[0] = pose.v[0] + obs_range * cos(pose.v[2] + obs_bearing);
      hit.v[1] = pose.v[1] + obs_range * sin(pose.v[2] + obs_bearing);

      // Convert to map grid coords.
      int mi, mj;
      mi = MAP_GXWX(self->map_, hit.v[0]);
      mj = MAP_GYWY(self->map_, hit.v[1]);

      // Part 1: Get distance from the hit to closest obstacle.
      // Off-map penalized as max distance
      if (!MAP_VALID(self->map_, mi, mj)) {
        z = self->map_->max_occ_dist;
      } else {
        z = self->map_->cells[MAP_INDEX(self->map_, mi, mj)].occ_dist;
      }
      // Gaussian model
      // NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
      pz += self->z_hit_ * exp(-(z * z) / z_hit_denom);
      // Part 2: random measurements
      pz += z_rand_component;

      // TODO(?): outlier rejection for short readings

      assert(pz <= 1.0);
      assert(pz >= 0.0);
      //      p *= pz;
      // here we have an ad-hoc weighting scheme for combining beam probs
      // works well, though...
      p += pz * pz * pz;
    }

    sample->raw_weight *= p;
    sample->weight *= pow(p, self->importance_factor_); // According to Probabilistic Robotics, 6.3.4
    sample->max_weight *= max_p;
    total_weight += sample->weight;
  }

  return total_weight;
}


bool
LikelihoodFieldModel::sensorUpdate(pf_t * pf, LaserData * data)
{
  if (max_beams_ < 2) {
    return false;
  }
  pf_update_sensor(pf, (pf_sensor_model_fn_t) sensorFunction, data);

  return true;
}

}  // namespace nav2_amcl
