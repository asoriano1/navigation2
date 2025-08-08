/*
 * Unit tests for LikelihoodFieldIntensityModel::sensorUpdate.
 *
 * These tests verify that the method returns a boolean indicating if the
 * update was performed. The update should succeed when valid data and an
 * intensity map are provided and fail otherwise.
 */

/* Author: Ángel Soriano*/

#include <gtest/gtest.h>
#include <cstdlib>
#include <limits>

#include "nav2_amcl/sensors/intensity/likelihood_field_intensity_model.hpp"
#include "nav2_amcl/map/map.hpp"
#include "nav2_amcl/pf/pf.hpp"
#include "rclcpp/rclcpp.hpp"


namespace nav2_amcl
{

class LikelihoodFieldIntensityModelTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    map_ = map_alloc();
    map_->size_x = 100;
    map_->size_y = 100;
    map_->scale = 0.05;
    map_->origin_x = 0.0;
    map_->origin_y = 0.0;
    map_->cells = reinterpret_cast<map_cell_t *>(
      malloc(sizeof(map_cell_t) * map_->size_x * map_->size_y));
    for (int i = 0; i < map_->size_x * map_->size_y; ++i) {
      map_->cells[i].occ_state = 0;
      map_->cells[i].occ_dist = 0.0;
      map_->cells[i].intensity_level = 0;
    }
    map_->cells[MAP_INDEX(map_, 50, 50)].occ_state = 1;
    map_->cells[MAP_INDEX(map_, 50, 50)].intensity_level = 100;

    model_ = std::make_unique<LikelihoodFieldIntensityModel>();
    model_->setIntensityMap(map_);

    pf_ = std::shared_ptr<pf_t>(pf_alloc(100, 0, 0.0, 0.0, nullptr), pf_free);
    pf_vector_t pose = pf_vector_zero();
    pf_init(pf_.get(), pose, pf_matrix_zero());
  }

  void TearDown() override
  {
    if (map_) {
      map_free(map_);
      map_ = nullptr;
    }
  }

  std::unique_ptr<LikelihoodFieldIntensityModel> model_;
  std::shared_ptr<pf_t> pf_;
  map_t * map_{nullptr};
};

TEST_F(LikelihoodFieldIntensityModelTest, SensorUpdateReturnsTrueWithValidData)
{
  intensity_data_t data;
  data.ranges.push_back(1.0f);
  data.intensities.push_back(100.0f);
  data.angle_min = 0.0;
  data.angle_increment = 0.0;

  bool updated = model_->sensorUpdate(pf_.get(), &data);
  EXPECT_TRUE(updated);
}

TEST_F(LikelihoodFieldIntensityModelTest, SensorUpdateWithNoIntensitiesReturnsFalse)
{
  intensity_data_t data;
  data.ranges.push_back(1.0f);

  bool updated = model_->sensorUpdate(pf_.get(), &data);
  EXPECT_FALSE(updated);
}

TEST_F(LikelihoodFieldIntensityModelTest, SensorUpdateWithInvalidRangeIgnored)
{
  intensity_data_t data;
  data.ranges.push_back(std::numeric_limits<float>::infinity());
  data.intensities.push_back(100.0f);
  data.angle_min = 0.0;
  data.angle_increment = 0.0;

  bool updated = model_->sensorUpdate(pf_.get(), &data);
  EXPECT_TRUE(updated);
}

TEST_F(LikelihoodFieldIntensityModelTest, SensorUpdateWithoutMapReturnsFalse)
{
  model_->setIntensityMap(nullptr);
  intensity_data_t data;
  data.ranges.push_back(1.0f);
  data.intensities.push_back(100.0f);
  data.angle_min = 0.0;
  data.angle_increment = 0.0;

  bool updated = model_->sensorUpdate(pf_.get(), &data);
  EXPECT_FALSE(updated);
}

TEST_F(LikelihoodFieldIntensityModelTest, SensorUpdateWithMultipleBeams)
{
  intensity_data_t data;
  data.ranges = {1.0f, 2.0f, 3.0f};
  data.intensities = {100.0f, 80.0f, 60.0f};
  data.angle_min = 0.0;
  data.angle_increment = 0.1;

  bool updated = model_->sensorUpdate(pf_.get(), &data);
  EXPECT_TRUE(updated);
}

TEST_F(LikelihoodFieldIntensityModelTest, SensorUpdateWithOutOfRangeIntensity)
{
  intensity_data_t data;
  data.ranges.push_back(1.0f);
  data.intensities.push_back(-128.0f);
  data.angle_min = 0.0;
  data.angle_increment = 0.0;

  bool updated = model_->sensorUpdate(pf_.get(), &data);
  EXPECT_TRUE(updated);
}

TEST_F(LikelihoodFieldIntensityModelTest, SensorUpdateWithMismatchedVectorSizesReturnsFalse)
{
  intensity_data_t data;
  data.ranges = {1.0f, 2.0f};
  data.intensities = {100.0f};
  data.angle_min = 0.0;
  data.angle_increment = 0.1;

  bool updated = model_->sensorUpdate(pf_.get(), &data);
  EXPECT_FALSE(updated);
}

}  // namespace nav2_amcl