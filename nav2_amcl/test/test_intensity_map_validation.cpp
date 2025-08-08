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
#include <cstdint>
#include <memory>

#include "nav2_amcl/amcl_node.hpp"
#include "nav2_amcl/map/map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class TestableAmclNode : public nav2_amcl::AmclNode
{
public:
  using nav2_amcl::AmclNode::AmclNode;
  using nav2_amcl::AmclNode::validateIntensityMap;
  using nav2_amcl::AmclNode::convertIntensityMap;

  void setMap(map_t * test_map)
  {
    map_ = test_map;
  }
};

using nav2_amcl::AmclNode;

class AmclIntensityMapValidationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    map_ = map_alloc();
    map_->size_x = 100;
    map_->size_y = 100;
    map_->scale = 0.05;
    map_->origin_x = 1.0;
    map_->origin_y = 2.0;
    map_->cells = reinterpret_cast<map_cell_t *>(malloc(sizeof(map_cell_t) * 100 * 100));

    node_ = std::make_shared<TestableAmclNode>();
    node_->setMap(map_);
  }

  void TearDown() override
  {
    if (map_) {
      map_free(map_);
      map_ = nullptr;
    }
    rclcpp::shutdown();
  }

  nav_msgs::msg::OccupancyGrid validMap()
  {
    nav_msgs::msg::OccupancyGrid map;
    map.info.width = static_cast<uint32_t>(map_->size_x);
    map.info.height = static_cast<uint32_t>(map_->size_y);
    map.info.resolution = static_cast<float>(map_->scale);
    map.info.origin.position.x = static_cast<float>(map_->origin_x);
    map.info.origin.position.y = static_cast<float>(map_->origin_y);
    map.info.origin.orientation.x = 0.0;
    map.info.origin.orientation.y = 0.0;
    map.info.origin.orientation.z = 0.0;
    map.info.origin.orientation.w = 1.0;
    return map;
  }


  std::shared_ptr<TestableAmclNode> node_;
  map_t * map_;
};

TEST_F(AmclIntensityMapValidationTest, ValidMapPasses)
{
  auto map = validMap();
  EXPECT_TRUE(node_->validateIntensityMap(map));
}

TEST_F(AmclIntensityMapValidationTest, InvalidSizeFails)
{
  auto map = validMap();
  map.info.width = 99;
  EXPECT_FALSE(node_->validateIntensityMap(map));
}

TEST_F(AmclIntensityMapValidationTest, InvalidResolutionFails)
{
  auto map = validMap();
  map.info.resolution = 0.1;
  EXPECT_FALSE(node_->validateIntensityMap(map));
}

TEST_F(AmclIntensityMapValidationTest, InvalidOriginFails)
{
  auto map = validMap();
  map.info.origin.position.x = 0.0;
  EXPECT_FALSE(node_->validateIntensityMap(map));
}

TEST_F(AmclIntensityMapValidationTest, InvalidOrientationXFails)
{
  auto map = validMap();
  map.info.origin.orientation.x = 0.5;
  EXPECT_FALSE(node_->validateIntensityMap(map));
}

TEST_F(AmclIntensityMapValidationTest, InvalidOrientationYFails)
{
  auto map = validMap();
  map.info.origin.orientation.y = 0.5;
  EXPECT_FALSE(node_->validateIntensityMap(map));
}

TEST_F(AmclIntensityMapValidationTest, InvalidOrientationZFails)
{
  auto map = validMap();
  map.info.origin.orientation.z = 0.5;
  EXPECT_FALSE(node_->validateIntensityMap(map));
}

TEST_F(AmclIntensityMapValidationTest, NullMapFails)
{
  nav_msgs::msg::OccupancyGrid empty_map;
  EXPECT_FALSE(node_->validateIntensityMap(empty_map));
}

TEST_F(AmclIntensityMapValidationTest, HandlesOutOfRangeValues)
{
  // Usa el helper para crear un intensity_map consistente con el map_ interno.
  auto intensity_map = validMap();
  intensity_map.data.resize(map_->size_x * map_->size_y, 100);

  // Pon algunos valores fuera de rango (pero dimensiones, resolución y origen son correctos)
  intensity_map.data[0] = -5;
  intensity_map.data[1] = static_cast<int8_t>(200);  // >127
  intensity_map.data[2] = -1;  // 255

  // Debe pasar, porque la función solo valida dimensiones, resolución y origen.
  EXPECT_TRUE(node_->validateIntensityMap(intensity_map));
}

TEST_F(AmclIntensityMapValidationTest, ConvertIntensityMapPreservesHighValues)
{
  auto intensity_map = validMap();
  intensity_map.data.resize(map_->size_x * map_->size_y, 0);
  intensity_map.data[0] = static_cast<int8_t>(200);  // Valor >127

  node_->convertIntensityMap(intensity_map);
  EXPECT_EQ(map_->cells[0].intensity_level, 200);
}