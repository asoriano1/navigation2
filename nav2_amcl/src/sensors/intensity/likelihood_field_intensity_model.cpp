// Description:   Library to use intensity measures
//
// Company:       Robotnik Automation S.L.L.
// Creation Year: 2025
// Author:        Ángel Soriano <asoriano@robotnik.es>
//
// Copyright (c) 2025, Robotnik Automation S.L.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Robotnik Automation S.L. nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Robotnik Automation S.L.L.
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
// OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
// OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
// OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
// EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "math.h"
#include "nav2_amcl/map/map.hpp"
#include "nav2_amcl/pf/pf.hpp"
#include "nav2_amcl/sensors/intensity/likelihood_field_intensity_model.hpp"
#include <iostream>
#include <memory>


namespace nav2_amcl
{


LikelihoodFieldIntensityModel::LikelihoodFieldIntensityModel() {}

double LikelihoodFieldIntensityModel::sensorUpdate(pf_t * pf, const intensity_data_t * data)
{
  if (!map_ || !data || data->ranges.empty() || data->intensities.empty()) {
    return 1.0;
  }

  const int occ_threshold = 0;   // 0: unknown, +1: occ, -1: free
  double total_likelihood = 0.0;

  pf_sample_set_t * set = pf->sets + pf->current_set;
  for (int i = 0; i < set->sample_count; ++i) {
    pf_sample_t * sample = set->samples + i;
    double particle_weight = 1.0;
    pf_vector_t pose = sample->pose;

    for (size_t beam_idx = 0; beam_idx < data->ranges.size(); ++beam_idx) {
      double angle = data->angle_min + beam_idx * data->angle_increment;
      double measured_range = data->ranges[beam_idx];

      // Skip invalid or infinite ranges
      if (!std::isfinite(measured_range)) {continue;}

      double x_hit = pose.v[0] + measured_range * cos(pose.v[2] + angle);
      double y_hit = pose.v[1] + measured_range * sin(pose.v[2] + angle);

      int mx = static_cast<int>((x_hit - map_->origin_x) / map_->scale);
      int my = static_cast<int>((y_hit - map_->origin_y) / map_->scale);

      if (mx < 0 || mx >= map_->size_x || my < 0 || my >= map_->size_y) {continue;}

      map_cell_t & cell = map_->cells[MAP_INDEX(map_, mx, my)];

      // Only compare intensity if the cell is occupied
      if (cell.occ_state > occ_threshold) {
        float measured_intensity = data->intensities[beam_idx];
        if (!std::isfinite(measured_intensity)) {continue;}
        int expected_intensity = cell.intensity_level;

        std::cout << "INTENSITY: cell (" << mx << "," << my << "), occ_state: " << cell.occ_state
                  << ", expected: " << expected_intensity << ", measured: " << measured_intensity <<
          std::endl;


        double delta = measured_intensity - expected_intensity;
        double p_intensity = exp(-0.5 * (delta * delta) / (sigma_intensity_ * sigma_intensity_));
        p_intensity = std::max(p_intensity, 1e-6);

        // Multiply intensity likelihood (weighted by lambda)
        particle_weight *= pow(p_intensity, lambda_intensity_);
      }
    }
    sample->weight *= particle_weight;
    total_likelihood += sample->weight;
  }
  return total_likelihood;
}

void LikelihoodFieldIntensityModel::setIntensityMap(map_t * map)
{
  map_ = map;
}

}  // namespace nav2_amcl

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_amcl::LikelihoodFieldIntensityModel, nav2_amcl::IntensityModel)
