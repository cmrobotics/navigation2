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

#include <sys/types.h>
#include <math.h>
#include <stdlib.h>
#include <assert.h>

#include "nav2_amcl/sensors/laser/laser.hpp"

namespace nav2_amcl
{

Laser::Laser(size_t max_beams,
  bool enable_grid_based_beam_sampling, double grid_based_beam_sampling_cell_size, size_t max_beam_hits_per_cell, 
  map_t * map)
: max_samples_(0), max_obs_(0), temp_obs_(NULL)
{
  max_beams_ = max_beams;
  map_ = map;
  max_beam_hits_per_cell_ = max_beam_hits_per_cell;
  enable_grid_based_beam_sampling_ = enable_grid_based_beam_sampling;
  grid_based_beam_sampling_cell_size_ = grid_based_beam_sampling_cell_size;
  max_beam_hits_per_cell_ = max_beam_hits_per_cell;
  sampled_beam_indexes_for_particle_w_max_weight_.resize(0);

  const double map_x_size_meters = map->size_x * map->scale;
  const double map_y_size_meters = map->size_y * map->scale;

  beam_sampling_max_x_grid_cells_ = map_x_size_meters/grid_based_beam_sampling_cell_size_; // need this later in sensorFunction
  const int beam_sampling_max_y_grid_cells = map_y_size_meters/grid_based_beam_sampling_cell_size_;

  cell_beam_count_for_current_particle_.resize(beam_sampling_max_x_grid_cells_ * beam_sampling_max_y_grid_cells);
}

Laser::~Laser()
{
  if (temp_obs_) {
    for (int k = 0; k < max_samples_; k++) {
      delete[] temp_obs_[k];
    }
    delete[] temp_obs_;
  }
}

void
Laser::reallocTempData(int new_max_samples, int new_max_obs)
{
  if (temp_obs_) {
    for (int k = 0; k < max_samples_; k++) {
      delete[] temp_obs_[k];
    }
    delete[] temp_obs_;
  }
  max_obs_ = new_max_obs;
  max_samples_ = fmax(max_samples_, new_max_samples);

  temp_obs_ = new double *[max_samples_]();
  for (int k = 0; k < max_samples_; k++) {
    temp_obs_[k] = new double[max_obs_]();
  }
}

void
Laser::SetLaserPose(pf_vector_t & laser_pose)
{
  laser_pose_ = laser_pose;
}

}  // namespace nav2_amcl
