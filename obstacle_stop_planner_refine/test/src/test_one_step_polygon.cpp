#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

class TestSuite : public ::testing::Test
{
protected:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
  }
  void TearDown()
  {
    (void)rclcpp::shutdown();
  }
};  // sanity_check

// write test here


int32_t main(int32_t argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
