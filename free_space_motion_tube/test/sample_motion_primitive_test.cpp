#include "gtest/gtest.h"
#include <free_space_motion_tube/basic.h>
#include <free_space_motion_tube/motion_primitive.h>


TEST(MotionPrimitiveStraightLine, TwoPoints) {
  // The sampling interval is equal to the length of the trajectory
  // The number of samples must be two.
  maneuver_t maneuver;
  unicycle_control_t control;
  maneuver.control = &control;
  control.forward_velocity = 0.5;
  control.angular_rate = 0.0;
  maneuver.time_horizon = 1;

  double sampling_interval = .5;

  point2d_t position;
  position.x = 0;
  position.y = 0;

  point2d_array_t samples;
  samples.number_of_points = 0;
  samples.max_number_of_points = 5;
  samples.points = (point2d_t *) 
    malloc(samples.max_number_of_points * sizeof(point2d_t));
  
  motion_primitive_unicycle_sample(&maneuver, &position, sampling_interval, &samples);
  // ASSERT number of samples
  ASSERT_EQ(samples.number_of_points, 2) << "Wrong number of samples";
  // EXPECT values at samples 
  EXPECT_EQ(samples.points[0].x, 0);
  EXPECT_EQ(samples.points[0].y, 0);
  EXPECT_EQ(samples.points[1].x, 0.5);
  EXPECT_EQ(samples.points[1].y, 0);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}