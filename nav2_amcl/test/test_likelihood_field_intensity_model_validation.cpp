/*
 * Unit tests for validating the consistency between the intensity map and the occupancy map
 * used by the AMCL node in Nav2.
 *
 * These tests verify that the intensity map received:
 *  - Is not null (occupancy map must be initialized).
 *  - Has the same dimensions (width, height) as the occupancy map.
 *  - Shares the same resolution (meters per pixel).
 *  - Has the same origin (x, y position and orientation).
 *
 * The `TestableAmclNode` subclass is used to access the protected `validateIntensityMap` method
 * and to inject a synthetic occupancy map (`map_`) for testing purposes.
 *
 * Each test creates a valid or deliberately invalid `nav_msgs::msg::OccupancyGrid` and
 * asserts that the validation function returns the expected result.
 *
 * Tests:
 *  - ValidMapPasses: A properly matched map should pass validation.
 *  - InvalidSizeFails: Mismatched width triggers failure.
 *  - InvalidResolutionFails: Different resolution triggers failure.
 *  - InvalidOriginFails: Different origin position triggers failure.
 *  - InvalidOrientationFails: Different orientation triggers failure.
 *  - NullOccupancyMapFails: If the internal occupancy map is null, validation should fail.
 */

/* Author: Ángel Soriano*/

#include <gtest/gtest.h>
#include "nav2_amcl/sensors/intensity/likelihood_field_intensity_model.hpp"
#include "nav2_amcl/pf/pf.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"


namespace nav2_amcl
{

class LikelihoodFieldIntensityModelTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Crear un mapa de ocupación falso
    map_.info.resolution = 0.05;
    map_.info.width = 100;
    map_.info.height = 100;
    map_.info.origin.position.x = 0.0;
    map_.info.origin.position.y = 0.0;
    map_.data.resize(map_.info.width * map_.info.height, 0);
    map_.data[50 * map_.info.width + 50] = 100;  // Punto intenso central

    model_ = std::make_unique<LikelihoodFieldIntensityModel>();

    pf_ = std::shared_ptr<pf_t>(pf_alloc(100, 0, 0.0, 0.0, nullptr), pf_free);
    //pf_ = pf_alloc(1, 0, 0.0, 0.0, nullptr);
    pf_vector_t pose = pf_vector_zero();
    pf_init(pf_.get(), pose, pf_matrix_zero());
  }

  void TearDown() override
  {
    //pf_free(pf_.get());
    //model_ = nullptr;
    //pf_ = nullptr;
  }

  std::unique_ptr<LikelihoodFieldIntensityModel> model_;
  std::shared_ptr<pf_t> pf_;
  nav_msgs::msg::OccupancyGrid map_;
};

TEST_F(LikelihoodFieldIntensityModelTest, SensorUpdateReturnsProbabilityInRange) {
  intensity_data_t data;
  data.intensities.push_back(100.0);

  double prob = model_->sensorUpdate(pf_.get(), &data);
  EXPECT_GE(prob, 0.0);
  EXPECT_LE(prob, 1.0);
}

TEST_F(LikelihoodFieldIntensityModelTest, SensorUpdateWithNoIntensitiesReturnsOne) {
  intensity_data_t data;
  // No intensities, no ranges
  double prob = model_->sensorUpdate(pf_.get(), &data);
  EXPECT_DOUBLE_EQ(prob, 1.0);
}

TEST_F(LikelihoodFieldIntensityModelTest, SensorUpdateWithInvalidRangeIgnored) {
  intensity_data_t data;
  data.ranges.push_back(std::numeric_limits<double>::infinity());
  data.intensities.push_back(100.0);

  double prob = model_->sensorUpdate(pf_.get(), &data);
  EXPECT_DOUBLE_EQ(prob, 1.0);
}

TEST_F(LikelihoodFieldIntensityModelTest, SensorUpdateWithoutMapReturnsOne) {
  model_->setIntensityMap(nullptr);
  intensity_data_t data;
  data.ranges.push_back(1.0);
  data.intensities.push_back(100.0);

  double prob = model_->sensorUpdate(pf_.get(), &data);
  EXPECT_DOUBLE_EQ(prob, 1.0);
}

TEST_F(LikelihoodFieldIntensityModelTest, SensorUpdateWithMultipleBeams) {
  intensity_data_t data;
  data.ranges = {1.0, 2.0, 3.0};
  data.intensities = {100.0, 80.0, 60.0};
  data.angle_min = 0.0;
  data.angle_increment = 0.1;

  double prob = model_->sensorUpdate(pf_.get(), &data);
  EXPECT_GE(prob, 0.0);
  EXPECT_LE(prob, 1.0);
}

TEST_F(LikelihoodFieldIntensityModelTest, SensorUpdateWithOutOfRangeIntensity) {
  intensity_data_t data;
  data.ranges.push_back(1.0);
  data.intensities.push_back(-128.0);

  double prob = model_->sensorUpdate(pf_.get(), &data);
  EXPECT_GE(prob, 0.0);
  EXPECT_LE(prob, 1.0);
}

}  // namespace nav2_amcl
