#include "gtest/gtest.h"

#include <free_space_motion_tube/basic.h>

TEST(SampleLineSegment, TwoPoints) {
  // The sampling interval is equal to the length of the line segment
  // The number of samples must be two, and they must coincide with 
  // the points that define the line segment.
  line_segment2d_t line_segment;
  point2d_t p_init{.x=0,.y=0}, p_end{.x=1, .y=0};
  line_segment.endpoints[0] = p_init;
  line_segment.endpoints[1] = p_end;

  double sampling_interval = 1;

  point2d_array_t samples;
  samples.max_number_of_points = 5;
  samples.number_of_points = 0;
  samples.points = (point2d_t *) 
    malloc(samples.max_number_of_points * sizeof(point2d_t));
  
  sample_line_segment(&line_segment, sampling_interval, &samples);
  // ASSERT number of samples
  ASSERT_EQ(samples.number_of_points, 2) << "Wrong number of samples";
  // EXPECT values at samples 
  EXPECT_EQ(samples.points[0].x, 
    line_segment.endpoints[0].x);
  EXPECT_EQ(samples.points[0].y, 
    line_segment.endpoints[0].y);
  EXPECT_EQ(samples.points[1].x, 
    line_segment.endpoints[1].x);
  EXPECT_EQ(samples.points[1].y, 
    line_segment.endpoints[1].y);
}
 
TEST(SampleLineSegment, ThreePoints) {
  // The sampling interval is equal to half of length of the line segment
  // The number of samples must be three, and they must coincide with the
  // points that define the line and the point between them.
  line_segment2d_t line_segment;
  point2d_t p_init{.x=-1,.y=0}, p_end{.x=1, .y=0};
  line_segment.endpoints[0] = p_init;
  line_segment.endpoints[1] = p_end;

  double sampling_interval = 1;

  point2d_array_t samples;
  samples.max_number_of_points = 5;
  samples.number_of_points = 0;
  samples.points = (point2d_t *) 
    malloc(samples.max_number_of_points * sizeof(point2d_t));
  
  sample_line_segment(&line_segment, sampling_interval, &samples);
  // ASSERT number of samples
  ASSERT_EQ(samples.number_of_points, 3) << "Wrong number of samples";
  // EXPECT values at samples 
  EXPECT_EQ(samples.points[0].x, -1);
  EXPECT_EQ(samples.points[0].y, 0);
  EXPECT_EQ(samples.points[1].x, 0);
  EXPECT_EQ(samples.points[1].y, 0);
  EXPECT_EQ(samples.points[2].x, 1);
  EXPECT_EQ(samples.points[2].y, 0);
}

TEST(SampleLineSegment, ForceThreePoints) {
  // The sampling interval is less than the length of the line segment, and
  // more than the half of it. Since the endpoints of the segment must be
  // included in the samples, the constant sampling interval will be such 
  // that there must be at least three samples
  line_segment2d_t line_segment;
  point2d_t p_init{.x=0,.y=0}, p_end{.x=1, .y=0};
  line_segment.endpoints[0] = p_init;
  line_segment.endpoints[1] = p_end;

  double sampling_interval = .9;

  point2d_array_t samples;
  samples.max_number_of_points = 5;
  samples.number_of_points = 0;
  samples.points = (point2d_t *) 
    malloc(samples.max_number_of_points * sizeof(point2d_t));
  
  sample_line_segment(&line_segment, sampling_interval, &samples);
  // ASSERT number of samples
  ASSERT_EQ(samples.number_of_points, 3) << "Wrong number of samples";
  // EXPECT values at samples 
  EXPECT_EQ(samples.points[0].x, 0);
  EXPECT_EQ(samples.points[0].y, 0);
  EXPECT_EQ(samples.points[2].x, 1);
  EXPECT_EQ(samples.points[2].y, 0);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}