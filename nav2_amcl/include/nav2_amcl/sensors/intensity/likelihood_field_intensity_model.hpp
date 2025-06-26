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


#pragma once

#include "nav2_amcl/sensors/intensity/intensity_model.hpp"
#include "nav2_amcl/map/map.hpp"
#include "nav2_amcl/pf/pf.hpp"
#include <memory>


namespace nav2_amcl
{

class LikelihoodFieldIntensityModel : public IntensityModel
{
private:
  map_t * map_ = nullptr;
  double sigma_intensity_{30.0};
  double lambda_intensity_{1.0};

public:
  LikelihoodFieldIntensityModel();
  ~LikelihoodFieldIntensityModel() override = default;

  double sensorUpdate(pf_t * pf, const intensity_data_t * data) override;
  void setIntensityMap(map_t * map);
  void setParameters(double sigma_intensity, double lambda_intensity)
  {
    sigma_intensity_ = sigma_intensity;
    lambda_intensity_ = lambda_intensity;
  }
};

}  // namespace nav2_amcl
