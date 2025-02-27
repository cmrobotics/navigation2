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
/**************************************************************************
 * Desc: Simple particle filter for localization.
 * Author: Andrew Howard
 * Date: 10 Dec 2002
 * CVS: $Id: pf.c 6345 2008-04-17 01:36:39Z gerkey $
 *************************************************************************/

#include <float.h>
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>

#include "nav2_amcl/pf/pf.hpp"
#include "nav2_amcl/pf/pf_pdf.hpp"
#include "nav2_amcl/pf/pf_kdtree.hpp"

#include "nav2_amcl/portable_utils.hpp"


// Compute the required number of samples, given that there are k bins
// with samples in them.
static int pf_resample_limit(pf_t * pf, int k);


// Create a new filter
pf_t * pf_alloc(
  int min_samples, int max_samples,
  double alpha_slow, double alpha_fast,
  pf_init_model_fn_t random_pose_fn,
  void * random_pose_data,
  double max_particle_gen_prob_ext_pose,
  int use_augmented_mcl, long pf_random_seed)
{
  int i, j;
  pf_t * pf;
  pf_sample_set_t * set;
  pf_sample_t * sample;

  pf = calloc(1, sizeof(pf_t));

  pf->random_pose_fn = random_pose_fn;
  pf->random_pose_data = random_pose_data;

  pf->min_samples = min_samples;
  pf->max_samples = max_samples;

  if(pf_random_seed == -1) {
    pf->pf_random_seed = time(NULL);
  } else {
    pf->pf_random_seed = pf_random_seed;
  }
  srand48(pf->pf_random_seed);

  // Control parameters for the population size calculation.  [err] is
  // the max error between the true distribution and the estimated
  // distribution.  [z] is the upper standard normal quantile for (1 -
  // p), where p is the probability that the error on the estimated
  // distrubition will be less than [err].
  pf->pop_err = 0.01;
  pf->pop_z = 3;
  pf->dist_threshold = 0.5;

  pf->current_set = 0;
  for (j = 0; j < 2; j++) {
    set = pf->sets + j;

    set->sample_count = max_samples;
    set->samples = calloc(max_samples, sizeof(pf_sample_t));

    for (i = 0; i < set->sample_count; i++) {
      sample = set->samples + i;
      sample->pose.v[0] = 0.0;
      sample->pose.v[1] = 0.0;
      sample->pose.v[2] = 0.0;
      sample->weight = 1.0 / max_samples;
      sample->raw_weight = 1.0 / max_samples;
    }

    // HACK: is 3 times max_samples enough?
    set->kdtree = pf_kdtree_alloc(3 * max_samples);

    set->cluster_count = 0;
    set->cluster_max_count = max_samples;
    set->clusters = calloc(set->cluster_max_count, sizeof(pf_cluster_t));

    set->mean = pf_vector_zero();
    set->cov = pf_matrix_zero();
  }

  pf->w_slow = 0.0;
  pf->w_fast = 0.0;
  pf->max_particle_gen_prob_ext_pose = max_particle_gen_prob_ext_pose;
  pf->ext_pose_is_valid = 0;

  pf->alpha_slow = alpha_slow;
  pf->alpha_fast = alpha_fast;

  pf->use_augmented_mcl = use_augmented_mcl;

  // set converged to 0
  pf_init_converged(pf);

  return pf;
}

// Free an existing filter
void pf_free(pf_t * pf)
{
  int i;

  for (i = 0; i < 2; i++) {
    free(pf->sets[i].clusters);
    pf_kdtree_free(pf->sets[i].kdtree);
    free(pf->sets[i].samples);
  }
  free(pf);
}

// Initialize the filter using a guassian
void pf_init(pf_t * pf, pf_vector_t mean, pf_matrix_t cov, long gaussian_random_seed)
{
  int i;
  pf_sample_set_t * set;
  pf_sample_t * sample;
  pf_pdf_gaussian_t * pdf;

  set = pf->sets + pf->current_set;

  // Create the kd tree for adaptive sampling
  pf_kdtree_clear(set->kdtree);

  set->sample_count = pf->max_samples;

  pdf = pf_pdf_gaussian_alloc(mean, cov, gaussian_random_seed);

  // Compute the new sample poses
  for (i = 0; i < set->sample_count; i++) {
    sample = set->samples + i;
    sample->weight = 1.0 / pf->max_samples;
    sample->pose = pf_pdf_gaussian_sample(pdf);

    // Add sample to histogram
    pf_kdtree_insert(set->kdtree, sample->pose, sample->weight);
  }

  pf->w_slow = pf->w_fast = 0.0;

  pf_pdf_gaussian_free(pdf);

  // Re-compute cluster statistics
  pf_cluster_stats(pf, set);

  // set converged to 0
  pf_init_converged(pf);

  // Update seeds
  pf->gaussian_random_seed = pdf->seed;

}


// Initialize the filter using some model
void pf_init_model(pf_t * pf, pf_init_model_fn_t init_fn, void * init_data)
{
  int i;
  pf_sample_set_t * set;
  pf_sample_t * sample;

  set = pf->sets + pf->current_set;

  // Create the kd tree for adaptive sampling
  pf_kdtree_clear(set->kdtree);

  set->sample_count = pf->max_samples;

  // Compute the new sample poses
  for (i = 0; i < set->sample_count; i++) {
    sample = set->samples + i;
    sample->weight = 1.0 / pf->max_samples;
    sample->pose = (*init_fn)(init_data);

    // Add sample to histogram
    pf_kdtree_insert(set->kdtree, sample->pose, sample->weight);
  }

  pf->w_slow = pf->w_fast = 0.0;

  // Re-compute cluster statistics
  pf_cluster_stats(pf, set);

  // set converged to 0
  pf_init_converged(pf);
}

void pf_init_converged(pf_t * pf)
{
  pf_sample_set_t * set;
  set = pf->sets + pf->current_set;
  set->converged = 0;
  pf->converged = 0;
}

int pf_update_converged(pf_t * pf)
{
  int i;
  pf_sample_set_t * set;
  pf_sample_t * sample;

  set = pf->sets + pf->current_set;
  double mean_x = 0, mean_y = 0;

  for (i = 0; i < set->sample_count; i++) {
    sample = set->samples + i;

    mean_x += sample->pose.v[0];
    mean_y += sample->pose.v[1];
  }
  mean_x /= set->sample_count;
  mean_y /= set->sample_count;

  for (i = 0; i < set->sample_count; i++) {
    sample = set->samples + i;
    if (fabs(sample->pose.v[0] - mean_x) > pf->dist_threshold ||
      fabs(sample->pose.v[1] - mean_y) > pf->dist_threshold)
    {
      set->converged = 0;
      pf->converged = 0;
      return 0;
    }
  }
  set->converged = 1;
  pf->converged = 1;
  return 1;
}

// Update the filter with some new action
// void pf_update_action(pf_t * pf, pf_action_model_fn_t action_fn, void * action_data)
// {
//   pf_sample_set_t * set;

//   set = pf->sets + pf->current_set;

//   (*action_fn)(action_data, set);
// }

// Update the filter with some new sensor observation
void pf_update_sensor(pf_t * pf, pf_sensor_model_fn_t sensor_fn, void * sensor_data)
{
  int i;
  pf_sample_set_t * set;
  pf_sample_t * sample;
  double total;
  double total_raw = 0.0;

  set = pf->sets + pf->current_set;

  // Compute the sample weights
  total = (*sensor_fn)(sensor_data, set);

  if (total > 0.0) {
    // Normalize weights
    double w_avg = 0.0;
    for (i = 0; i < set->sample_count; i++) {
      sample = set->samples + i;
      sample->weight /= total;

      total_raw += sample->raw_weight;
    }

    // Normalize raw weights
    for(i = 0; i < set->sample_count; i++) {
      sample = set->samples + i;
      w_avg += sample->raw_weight;
      sample->raw_weight /= total_raw;
    }

    // Update running averages of likelihood of samples (Prob Rob p258)
    w_avg /= set->sample_count;
    if (pf->w_slow == 0.0) {
      pf->w_slow = w_avg;
    } else {
      pf->w_slow += pf->alpha_slow * (w_avg - pf->w_slow);
    }
    if (pf->w_fast == 0.0) {
      pf->w_fast = w_avg;
    } else {
      pf->w_fast += pf->alpha_fast * (w_avg - pf->w_fast);
    }
  } else {
    // Handle zero total
    for (i = 0; i < set->sample_count; i++) {
      sample = set->samples + i;
      sample->weight = 1.0 / set->sample_count;
    }
  }
}

double get_determinant(double *cov_matrix)
{
  double det = cov_matrix[0] * cov_matrix[4] * cov_matrix[8] + 
              cov_matrix[1] * cov_matrix[5] * cov_matrix[6] + 
              cov_matrix[3] * cov_matrix[7] * cov_matrix[2] -
              cov_matrix[2] * cov_matrix[4] * cov_matrix[6] - 
              cov_matrix[1] * cov_matrix[3] * cov_matrix[8] - 
              cov_matrix[0] * cov_matrix[5] * cov_matrix[7];

  return det;
}

int get_inverse(double *cov_matrix, double *result)
{
  double det = get_determinant(cov_matrix);
  double inv[COV_MAT_SIZE] = {   1 * (cov_matrix[4] * cov_matrix[8] - cov_matrix[5] * cov_matrix[7]) / det,
                        -1 * (cov_matrix[3] * cov_matrix[8] - cov_matrix[5] * cov_matrix[6]) / det,
                         1 * (cov_matrix[3] * cov_matrix[7] - cov_matrix[4] * cov_matrix[6]) / det,
                        -1 * (cov_matrix[1] * cov_matrix[8] - cov_matrix[2] * cov_matrix[7]) / det,
                         1 * (cov_matrix[0] * cov_matrix[8] - cov_matrix[2] * cov_matrix[6]) / det,
                        -1 * (cov_matrix[0] * cov_matrix[7] - cov_matrix[1] * cov_matrix[6]) / det,
                         1 * (cov_matrix[1] * cov_matrix[5] - cov_matrix[2] * cov_matrix[4]) / det,
                        -1 * (cov_matrix[0] * cov_matrix[5] - cov_matrix[2] * cov_matrix[3]) / det,
                         1 * (cov_matrix[0] * cov_matrix[4] - cov_matrix[1] * cov_matrix[3]) / det};

  memcpy(result, inv, COV_MAT_SIZE*sizeof(double));

  return 0;
}

int mult_1_3_x_3_3(const double *mat_1_3, const double *mat_3_3, double *result)
{
  double r[3] = { mat_1_3[0] * mat_3_3[0] + mat_1_3[1] * mat_3_3[3] + mat_1_3[2] * mat_3_3[6],
                      mat_1_3[0] * mat_3_3[1] + mat_1_3[1] * mat_3_3[4] + mat_1_3[2] * mat_3_3[7],
                      mat_1_3[0] * mat_3_3[2] + mat_1_3[1] * mat_3_3[5] + mat_1_3[2] * mat_3_3[8]};

  memcpy(result, r, 3*sizeof(double));

  return 0;
}

double mult_1_3_x_3_1(const double *mat_1_3, const double *mat_3_1)
{
  double result = mat_1_3[0] * mat_3_1[0] + mat_1_3[1] * mat_3_1[1] + mat_1_3[2] * mat_3_1[2];
  return result;
}

// Return random std mean number
double norm_random()
{
  double u = (double)rand()/RAND_MAX;
  double v = (double)rand()/RAND_MAX;
  double x = sqrt(-2*log(u))*cos(2*M_PI*v); // Check this

  return x;

}

int generate_random_particle(double x, double y, double yaw, const double *cov_matrix, double *pose_v)
{
    // See Improved LiDAR Probabilistic Localization for Autonomous Vehicles Using GNSS, #3.4 for details

    double pose[3] = {norm_random(), norm_random(), norm_random()};
  
    mult_1_3_x_3_3(pose, cov_matrix, pose);
    pose[0] = pose[0] + x;
    pose[1] = pose[1] + y;
    pose[2] = pose[2] + yaw;

    memcpy(pose_v, pose, 3*sizeof(double));

    return 0;
}

// Unconstrained (input args can be of any value, even more/less than 2*pi) angular distance in range [-pi, pi)
// Source: https://stackoverflow.com/a/28037434/13167995
double angular_difference(double angle1, double angle2){
  double diff = fmod(angle2 - angle1 + M_PI_2, M_PI) - M_PI_2;
  
  return diff < -M_PI_2 ? diff + M_PI : diff;
}

// Resample the distribution
void pf_update_resample(pf_t * pf)
{
  int i;
  double total;
  pf_sample_set_t * set_a, * set_b;
  pf_sample_t * sample_a, * sample_b;

  // double r,c,U;
  // int m;
  // double count_inv;
  double * c;

  double w_diff;

  set_a = pf->sets + pf->current_set;
  set_b = pf->sets + (pf->current_set + 1) % 2;

  double total_dist_prob = 0;

  double covariance_determinant = (pf->cov_matrix[0] * pf->cov_matrix[4] * pf->cov_matrix[8]);
  double max_particle_likelihood = 1/sqrt(pow(2*M_PI, 3) * covariance_determinant);

  if(pf->ext_pose_is_valid){
    double total_weight = 0;
    for(i = 0; i < set_a->sample_count; i++)
    {

      // See Improved LiDAR Probabilistic Localization for Autonomous Vehicles Using GNSS, #3.2 for details
      double distance = pow(set_a->samples[i].pose.v[0]-pf->ext_x, 2) / pf->cov_matrix[0] + 
                        pow(set_a->samples[i].pose.v[1]-pf->ext_y, 2) / pf->cov_matrix[4] + 
                        + pow(angular_difference(set_a->samples[i].pose.v[2], pf->ext_yaw), 2) / pf->cov_matrix[8];

      double ext_pose_likelihood = max_particle_likelihood*exp(-1*distance/2);

      total_dist_prob += ext_pose_likelihood;

      // fprintf(stderr, "AMCL: laser weight - %f, ext pose likelihood - %f\n", set_a->samples[i].weight, ext_pose_likelihood);

      // See Improved LiDAR Probabilistic Localization for Autonomous Vehicles Using GNSS, #3.3 for details
      set_a->samples[i].weight = set_a->samples[i].weight * LASER_TO_EXTERNAL_POSE_BALANCE_FACTOR + ext_pose_likelihood;

      total_weight += set_a->samples[i].weight;
    }

    /// Handle total weight of 0
    if(total_weight == 0)
    {
      for(i=0;i<set_a->sample_count;i++){
        set_a->samples[i].weight = 1;
        total_weight += set_a->samples[i].weight;
      }
    } else 
    {
      /// Normalization
      for(i=0;i<set_a->sample_count;i++)
      {
        set_a->samples[i].weight = set_a->samples[i].weight / total_weight;
      }
    }
  }


  // Build up cumulative probability table for resampling.
  // TODO(?): Replace this with a more efficient procedure
  // (e.g., http://www.network-theory.co.uk/docs/gslref/GeneralDiscreteDistributions.html)
  c = (double *)malloc(sizeof(double) * (set_a->sample_count + 1));
  c[0] = 0.0;
  for (i = 0; i < set_a->sample_count; i++) {
    c[i + 1] = c[i] + set_a->samples[i].weight;
  }

  // Create the kd tree for adaptive sampling
  pf_kdtree_clear(set_b->kdtree);

  // Draw samples from set a to create set b.
  total = 0;
  set_b->sample_count = 0;


w_diff = 0.0;
if(pf->ext_pose_is_valid){

  // See Improved LiDAR Probabilistic Localization for Autonomous Vehicles Using GNSS, #3.4 for details
  total_dist_prob = total_dist_prob/set_a->sample_count;
  w_diff = (pf->max_particle_gen_prob_ext_pose * max_particle_likelihood - total_dist_prob) / max_particle_likelihood;

  if(w_diff < 0.0)
    w_diff = 0.0;
  
} else if (pf->use_augmented_mcl) {

  w_diff = 1.0 - pf->w_fast / pf->w_slow;
  if(w_diff < 0.0)
    w_diff = 0.0;
}

  

  // Can't (easily) combine low-variance sampler with KLD adaptive
  // sampling, so we'll take the more traditional route.
  /*
  // Low-variance resampler, taken from Probabilistic Robotics, p110
  count_inv = 1.0/set_a->sample_count;
  r = drand48() * count_inv;
  c = set_a->samples[0].weight;
  i = 0;
  m = 0;
  */
  while (set_b->sample_count < pf->max_samples) {
    sample_b = set_b->samples + set_b->sample_count++;

    if(drand48() < w_diff)
    {
      if(pf->ext_pose_is_valid){
        generate_random_particle(pf->ext_x, pf->ext_y, pf->ext_yaw, pf->eigen_matrix, sample_b->pose.v);
      } else if (pf->use_augmented_mcl) {
        sample_b->pose = (pf->random_pose_fn)(pf->random_pose_data);
      }
    } else
    {
      // Can't (easily) combine low-variance sampler with KLD adaptive
      // sampling, so we'll take the more traditional route.
      /*
      // Low-variance resampler, taken from Probabilistic Robotics, p110
      U = r + m * count_inv;
      while(U>c)
      {
        i++;
        // Handle wrap-around by resetting counters and picking a new random
        // number
        if(i >= set_a->sample_count)
        {
          r = drand48() * count_inv;
          c = set_a->samples[0].weight;
          i = 0;
          m = 0;
          U = r + m * count_inv;
          continue;
        }
        c += set_a->samples[i].weight;
      }
      m++;
      */

      // Naive discrete event sampler
      double r;
      r = drand48();
      for (i = 0; i < set_a->sample_count; i++) {
        if ((c[i] <= r) && (r < c[i + 1])) {
          break;
        }
      }
      assert(i < set_a->sample_count);

      sample_a = set_a->samples + i;

      assert(sample_a->weight > 0);

      // Add sample to list
      sample_b->pose = sample_a->pose;
    }

    sample_b->weight = 1.0;
    total += sample_b->weight;

    // Add sample to histogram
    pf_kdtree_insert(set_b->kdtree, sample_b->pose, sample_b->weight);

    // See if we have enough samples yet
    if (set_b->sample_count > pf_resample_limit(pf, set_b->kdtree->leaf_count)) {
      break;
    }
  }

  // Reset averages, to avoid spiraling off into complete randomness.
  if (w_diff > 0.0) {
    pf->w_slow = pf->w_fast = 0.0;
  }

  // fprintf(stderr, "\n\n");

  // Normalize weights
  for (i = 0; i < set_b->sample_count; i++) {
    sample_b = set_b->samples + i;
    sample_b->weight /= total;
  }

  // Re-compute cluster statistics
  pf_cluster_stats(pf, set_b);

  // Use the newly created sample set
  pf->current_set = (pf->current_set + 1) % 2;

  pf_update_converged(pf);

  free(c);
}


// Compute the required number of samples, given that there are k bins
// with samples in them.  This is taken directly from Fox et al.
int pf_resample_limit(pf_t * pf, int k)
{
  double a, b, c, x;
  int n;

  if (k <= 1) {
    return pf->max_samples;
  }

  a = 1;
  b = 2 / (9 * ((double) k - 1));
  c = sqrt(2 / (9 * ((double) k - 1))) * pf->pop_z;
  x = a - b + c;

  n = (int) ceil((k - 1) / (2 * pf->pop_err) * x * x * x);

  if (n < pf->min_samples) {
    return pf->min_samples;
  }
  if (n > pf->max_samples) {
    return pf->max_samples;
  }

  return n;
}


// Re-compute the cluster statistics for a sample set
void pf_cluster_stats(pf_t * pf, pf_sample_set_t * set)
{
  (void)pf;
  int i, j, k, cidx;
  pf_sample_t * sample;
  pf_cluster_t * cluster;

  // Workspace
  double m[4], c[2][2];
  size_t count;
  double weight;

  // Cluster the samples
  pf_kdtree_cluster(set->kdtree);

  // Initialize cluster stats
  set->cluster_count = 0;

  for (i = 0; i < set->cluster_max_count; i++) {
    cluster = set->clusters + i;
    cluster->count = 0;
    cluster->weight = 0;
    cluster->mean = pf_vector_zero();
    cluster->cov = pf_matrix_zero();

    for (j = 0; j < 4; j++) {
      cluster->m[j] = 0.0;
    }
    for (j = 0; j < 2; j++) {
      for (k = 0; k < 2; k++) {
        cluster->c[j][k] = 0.0;
      }
    }
  }

  // Initialize overall filter stats
  count = 0;
  weight = 0.0;
  set->mean = pf_vector_zero();
  set->cov = pf_matrix_zero();
  for (j = 0; j < 4; j++) {
    m[j] = 0.0;
  }
  for (j = 0; j < 2; j++) {
    for (k = 0; k < 2; k++) {
      c[j][k] = 0.0;
    }
  }

  // Compute cluster stats
  for (i = 0; i < set->sample_count; i++) {
    sample = set->samples + i;

    // printf("%d %f %f %f\n", i, sample->pose.v[0], sample->pose.v[1], sample->pose.v[2]);

    // Get the cluster label for this sample
    cidx = pf_kdtree_get_cluster(set->kdtree, sample->pose);
    assert(cidx >= 0);
    if (cidx >= set->cluster_max_count) {
      continue;
    }
    if (cidx + 1 > set->cluster_count) {
      set->cluster_count = cidx + 1;
    }

    cluster = set->clusters + cidx;

    cluster->count += 1;
    cluster->weight += sample->weight;

    count += 1;
    weight += sample->weight;

    // Compute mean
    cluster->m[0] += sample->weight * sample->pose.v[0];
    cluster->m[1] += sample->weight * sample->pose.v[1];
    cluster->m[2] += sample->weight * cos(sample->pose.v[2]);
    cluster->m[3] += sample->weight * sin(sample->pose.v[2]);

    m[0] += sample->weight * sample->pose.v[0];
    m[1] += sample->weight * sample->pose.v[1];
    m[2] += sample->weight * cos(sample->pose.v[2]);
    m[3] += sample->weight * sin(sample->pose.v[2]);

    // Compute covariance in linear components
    for (j = 0; j < 2; j++) {
      for (k = 0; k < 2; k++) {
        cluster->c[j][k] += sample->weight * sample->pose.v[j] * sample->pose.v[k];
        c[j][k] += sample->weight * sample->pose.v[j] * sample->pose.v[k];
      }
    }
  }

  // Normalize
  for (i = 0; i < set->cluster_count; i++) {
    cluster = set->clusters + i;

    cluster->mean.v[0] = cluster->m[0] / cluster->weight;
    cluster->mean.v[1] = cluster->m[1] / cluster->weight;
    cluster->mean.v[2] = atan2(cluster->m[3], cluster->m[2]);

    cluster->cov = pf_matrix_zero();

    // Covariance in linear components
    for (j = 0; j < 2; j++) {
      for (k = 0; k < 2; k++) {
        cluster->cov.m[j][k] = cluster->c[j][k] / cluster->weight -
          cluster->mean.v[j] * cluster->mean.v[k];
      }
    }

    // Covariance in angular components; I think this is the correct
    // formula for circular statistics.
    cluster->cov.m[2][2] = -2 * log(
      sqrt(
        cluster->m[2] * cluster->m[2] +
        // AMCL miscalculates orientation covariance for clusters https://github.com/ros-planning/navigation/issues/1160
        cluster->m[3] * cluster->m[3]) / cluster->weight);

    // printf("cluster %d %d %f (%f %f %f)\n", i, cluster->count, cluster->weight,
    // cluster->mean.v[0], cluster->mean.v[1], cluster->mean.v[2]);
    // pf_matrix_fprintf(cluster->cov, stdout, "%e");
  }

  // Compute overall filter stats
  set->mean.v[0] = m[0] / weight;
  set->mean.v[1] = m[1] / weight;
  set->mean.v[2] = atan2(m[3], m[2]);

  // Covariance in linear components
  for (j = 0; j < 2; j++) {
    for (k = 0; k < 2; k++) {
      set->cov.m[j][k] = c[j][k] / weight - set->mean.v[j] * set->mean.v[k];
    }
  }

  // Covariance in angular components; I think this is the correct
  // formula for circular statistics.
  set->cov.m[2][2] = -2 * log(sqrt(m[2] * m[2] + m[3] * m[3]));
}


// Compute the CEP statistics (mean and variance).
// void pf_get_cep_stats(pf_t * pf, pf_vector_t * mean, double * var)
// {
//   int i;
//   double mn, mx, my, mrr;
//   pf_sample_set_t * set;
//   pf_sample_t * sample;

//   set = pf->sets + pf->current_set;

//   mn = 0.0;
//   mx = 0.0;
//   my = 0.0;
//   mrr = 0.0;

//   for (i = 0; i < set->sample_count; i++) {
//     sample = set->samples + i;

//     mn += sample->weight;
//     mx += sample->weight * sample->pose.v[0];
//     my += sample->weight * sample->pose.v[1];
//     mrr += sample->weight * sample->pose.v[0] * sample->pose.v[0];
//     mrr += sample->weight * sample->pose.v[1] * sample->pose.v[1];
//   }

//   mean->v[0] = mx / mn;
//   mean->v[1] = my / mn;
//   mean->v[2] = 0.0;

//   *var = mrr / mn - (mx * mx / (mn * mn) + my * my / (mn * mn));
// }


// Get the statistics for a particular cluster.
int pf_get_cluster_stats(
  pf_t * pf, int clabel, double * weight,
  pf_vector_t * mean, pf_matrix_t * cov)
{
  pf_sample_set_t * set;
  pf_cluster_t * cluster;

  set = pf->sets + pf->current_set;

  if (clabel >= set->cluster_count) {
    return 0;
  }
  cluster = set->clusters + clabel;

  *weight = cluster->weight;
  *mean = cluster->mean;
  *cov = cluster->cov;

  return 1;
}
