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

#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "nav2_amcl/map/map.hpp"
#include "nav2_amcl/pf/pf.hpp"
#include "nav2_amcl/sensors/intensity/likelihood_field_intensity_model.hpp"
#include <iostream>
#include <memory>
#include <algorithm>


namespace nav2_amcl
{
/*
// Data passed to sensor function
struct LikelihoodFieldIntensitySensorData
{
  map_t * map;
  double sigma_intensity;
  double lambda_intensity;
  const intensity_data_t * data;
  double total_likelihood;
};

LikelihoodFieldIntensityModel::LikelihoodFieldIntensityModel() {}

double LikelihoodFieldIntensityModel::sensorUpdate(pf_t * pf, const intensity_data_t * data)
{
  if (!map_ || !data || data->ranges.empty() || data->intensities.empty()) {
    return 1.0;
  }

  LikelihoodFieldIntensitySensorData sensor_data{map_, sigma_intensity_, lambda_intensity_, data, 0.0};
  pf_update_sensor(pf, LikelihoodFieldIntensityModel::sensorFunction, &sensor_data);
  return sensor_data.total_likelihood;
}

double LikelihoodFieldIntensityModel::sensorFunction(void * data, pf_sample_set_t * set)
{
  auto * sdata = static_cast<LikelihoodFieldIntensitySensorData *>(data);
  const intensity_data_t * sensor = sdata->data;
  const int occ_threshold = 0;   // 0: unknown, +1: occ, -1: free
  double total_likelihood = 0.0;

  for (int i = 0; i < set->sample_count; ++i) {
    pf_sample_t * sample = set->samples + i;
    double particle_weight = 1.0;
    pf_vector_t pose = sample->pose;

    for (size_t beam_idx = 0; beam_idx < sensor->ranges.size(); ++beam_idx) {
      double angle = sensor->angle_min + beam_idx * sensor->angle_increment;
      double measured_range = sensor->ranges[beam_idx];

      // Skip invalid or infinite ranges
      if (!std::isfinite(measured_range)) {continue;}

      double x_hit = pose.v[0] + measured_range * cos(pose.v[2] + angle);
      double y_hit = pose.v[1] + measured_range * sin(pose.v[2] + angle);

      int mx = static_cast<int>((x_hit - sdata->map->origin_x) / sdata->map->scale);
      int my = static_cast<int>((y_hit - sdata->map->origin_y) / sdata->map->scale);

      if (mx < 0 || mx >= sdata->map->size_x || my < 0 || my >= sdata->map->size_y) {continue;}

      map_cell_t & cell = sdata->map->cells[MAP_INDEX(sdata->map, mx, my)];

      // Only compare intensity if the cell is occupied
      if (cell.occ_state > occ_threshold) {
        float measured_intensity = sensor->intensities[beam_idx];
        if (!std::isfinite(measured_intensity)) {continue;}
        int expected_intensity = cell.intensity_level;

        std::cout << "INTENSITY: cell (" << mx << "," << my << "), occ_state: " << cell.occ_state
                  << ", expected: " << expected_intensity << ", measured: " << measured_intensity <<
          std::endl;

        double delta = measured_intensity - expected_intensity;
        double p_intensity = exp(-0.5 * (delta * delta) /
          (sdata->sigma_intensity * sdata->sigma_intensity));
        p_intensity = std::max(p_intensity, 1e-6);

        // Multiply intensity likelihood (weighted by lambda)
        particle_weight *= pow(p_intensity, sdata->lambda_intensity);
      }
    }
    sample->weight *= particle_weight;
    total_likelihood += sample->weight;
  }

  sdata->total_likelihood = total_likelihood;
  return total_likelihood;
}

void LikelihoodFieldIntensityModel::setIntensityMap(map_t * map)
{
  map_ = map;
}
*/

LikelihoodFieldIntensityModel::LikelihoodFieldIntensityModel() {}

// Helper struct to pass both the model and observation data to the sensor
// function used by pf_update_sensor.
struct IntensitySensorData
{
  const LikelihoodFieldIntensityModel * model;
  const intensity_data_t * data;
};

// Compute particle weights given an intensity observation.
double LikelihoodFieldIntensityModel::sensorFunction(pf_sample_set_t * set)
{
  const intensity_data_t * data = intensity_data_;

  const int occ_threshold = 0;
  double total_weight = 0.0;

  for (int i = 0; i < set->sample_count; ++i) {
    pf_sample_t * sample = set->samples + i;
    double particle_weight = 1.0;
    pf_vector_t pose = sample->pose;

    for (size_t beam_idx = 0; beam_idx < data->ranges.size(); ++beam_idx) {
      double angle = data->angle_min + beam_idx * data->angle_increment;
      double measured_range = data->ranges[beam_idx];

      if (!std::isfinite(measured_range)) {
        continue;
      }

      double x_hit = pose.v[0] + measured_range * cos(pose.v[2] + angle);
      double y_hit = pose.v[1] + measured_range * sin(pose.v[2] + angle);

      int mx = static_cast<int>((x_hit - map_->origin_x) / map_->scale);
      int my = static_cast<int>((y_hit - map_->origin_y) / map_->scale);

      if (mx < 0 || mx >= map_->size_x || my < 0 || my >= map_->size_y) {
        continue;
      }

      map_cell_t & cell = map_->cells[MAP_INDEX(map_, mx, my)];

      // Only compare intensity if the cell is occupied
      if (cell.occ_state > occ_threshold) {
        float measured_intensity = data->intensities[beam_idx];
        if (!std::isfinite(measured_intensity)) {
          continue;
        }

        int expected_intensity = cell.intensity_level;
        double delta = measured_intensity - expected_intensity;
        double p_intensity = exp(-0.5 * delta * delta / (sigma_intensity_ * sigma_intensity_));
        p_intensity = std::max(p_intensity, 1e-6);

        // Multiply intensity likelihood (weighted by lambda)
        particle_weight *= pow(p_intensity, lambda_intensity_);
      }
    }

    sample->weight *= particle_weight;
    total_weight += sample->weight;

    RCLCPP_INFO(
      rclcpp::get_logger("amcl_intensity"),
      "Intensity model total weight: %f", total_weight);


  }

  return total_weight;
}

bool LikelihoodFieldIntensityModel::sensorUpdate(pf_t * pf, const intensity_data_t * data)
{
  if (!map_ || !data || data->ranges.empty() || data->intensities.empty()) {
    RCLCPP_INFO(
      rclcpp::get_logger("amcl_intensity"),
      "Returning false data:%d", data->ranges.empty());
    return false;
  }

  //IntensitySensorData sensor_data{this, data};
  auto sensor_function_wrapper = [](void * obj, pf_sample_set_t * set) -> double {
      auto * self = reinterpret_cast<LikelihoodFieldIntensityModel *>(obj);
      return self->sensorFunction(set);  // Llama al método privado de instancia
    };

  intensity_data_ = data;

  pf_update_sensor(pf, sensor_function_wrapper, this);
  return true;
}

void LikelihoodFieldIntensityModel::setIntensityMap(map_t * map)
{
  map_ = map;
}

}  // namespace nav2_amcl

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_amcl::LikelihoodFieldIntensityModel, nav2_amcl::IntensityModel)
