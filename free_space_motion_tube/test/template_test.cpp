#include "gtest/gtest.h"
#include "math.h"
extern "C"{
  #include <free_space/basic.h>
  #include <free_space/free_space.h>
}

TEST(StraightTemplate, TwoPointsPerSide) {
  maneuver_t maneuver;
  unicycle_control_t control;
  maneuver.control = (void *) &control;
  control.forward_velocity = .5;
  control.angular_rate = 0.;
  maneuver.time_horizon = 1;

  body_t body;
  polyline_t body_geometry;
  body_geometry.number_of_points = 4;
  body_geometry.points = (point2d_t *) malloc(
    body_geometry.number_of_points * sizeof(point2d_t));
  body_geometry.points[FRONT_LEFT].x = 0.5;
  body_geometry.points[FRONT_LEFT].y = 0.25;
  body_geometry.points[FRONT_RIGHT].x = 0.5;
  body_geometry.points[FRONT_RIGHT].y = -0.25; 
  body.type = POLYLINE;
  body.geometry= (void *) &body_geometry;

  double sampling_interval = .5;

  template_t ms_template;
  point2d_array_t samples[3];
  for(int i=0; i<3; i++){
    samples[i].number_of_points = 0;
    samples[i].max_number_of_points = 5;
    samples[i].points = (point2d_t *) malloc(
      samples[i].max_number_of_points * sizeof(point2d_t));
  }
  ms_template.left = samples[0];
  ms_template.right = samples[1];
  ms_template.front = samples[2];

  sample_free_space_template_in_cartesian(&maneuver, &body, sampling_interval,
    &ms_template);

  // LEFT side
  ASSERT_EQ(ms_template.left.number_of_points, 2) << "Wrong number of samples on left side";
  EXPECT_EQ(ms_template.left.points[0].x, 0.5);
  EXPECT_EQ(ms_template.left.points[0].y, .25);
  EXPECT_EQ(ms_template.left.points[1].x, 1.0);
  EXPECT_EQ(ms_template.left.points[1].y, .25);

  // RIGHT side
  ASSERT_EQ(ms_template.right.number_of_points, 2) << "Wrong number of samples on right side";
  EXPECT_EQ(ms_template.right.points[0].x, 0.5);
  EXPECT_EQ(ms_template.right.points[0].y, -.25);
  EXPECT_EQ(ms_template.right.points[1].x, 1.0);
  EXPECT_EQ(ms_template.right.points[1].y, -.25);

  // FRONT
  ASSERT_EQ(ms_template.front.number_of_points, 2) << "Wrong number of samples on front side";
  EXPECT_EQ(ms_template.front.points[0].x, 1.0);
  EXPECT_EQ(ms_template.front.points[0].y, .25);
  EXPECT_EQ(ms_template.front.points[1].x, 1.0);
  EXPECT_EQ(ms_template.front.points[1].y, -.25);
}

TEST(StraightTemplate, ThreePointsPerSide) {
  maneuver_t maneuver;
  unicycle_control_t control;
  maneuver.control = (void *) &control;
  control.forward_velocity = .5;
  control.angular_rate = 0.;
  maneuver.time_horizon = 1;

  body_t body;
  polyline_t body_geometry;
  body_geometry.number_of_points = 4;
  body_geometry.points = (point2d_t *) malloc(
    body_geometry.number_of_points * sizeof(point2d_t));
  body_geometry.points[FRONT_LEFT].x = 0.5;
  body_geometry.points[FRONT_LEFT].y = 0.25;
  body_geometry.points[FRONT_RIGHT].x = 0.5;
  body_geometry.points[FRONT_RIGHT].y = -0.25; 
  body.type = POLYLINE;
  body.geometry= (void *) &body_geometry;

  double sampling_interval = .25;

  template_t ms_template;
  point2d_array_t samples[3];
  for(int i=0; i<3; i++){
    samples[i].number_of_points = 0;
    samples[i].max_number_of_points = 5;
    samples[i].points = (point2d_t *) malloc(
      samples[i].max_number_of_points * sizeof(point2d_t));
  }
  ms_template.left = samples[0];
  ms_template.right = samples[1];
  ms_template.front = samples[2];

  sample_free_space_template_in_cartesian(&maneuver, &body, sampling_interval,
    &ms_template);

  // LEFT side
  ASSERT_EQ(ms_template.left.number_of_points, 3) << "Wrong number of samples on left side";
  EXPECT_EQ(ms_template.left.points[0].x, 0.5);
  EXPECT_EQ(ms_template.left.points[0].y, .25);
  EXPECT_EQ(ms_template.left.points[1].x, .75);
  EXPECT_EQ(ms_template.left.points[1].y, .25);
  EXPECT_EQ(ms_template.left.points[2].x, 1.0);
  EXPECT_EQ(ms_template.left.points[2].y, .25);

  // RIGHT side
  ASSERT_EQ(ms_template.right.number_of_points, 3) << "Wrong number of samples on right side";
  EXPECT_EQ(ms_template.right.points[0].x, 0.5);
  EXPECT_EQ(ms_template.right.points[0].y, -.25);
  EXPECT_EQ(ms_template.right.points[1].x, .75);
  EXPECT_EQ(ms_template.right.points[1].y, -.25);
  EXPECT_EQ(ms_template.right.points[2].x, 1.0);
  EXPECT_EQ(ms_template.right.points[2].y, -.25);
  // FRONT
  ASSERT_EQ(ms_template.front.number_of_points, 3) << "Wrong number of samples on front side";
  EXPECT_EQ(ms_template.front.points[0].x, 1.0);
  EXPECT_EQ(ms_template.front.points[0].y, .25);
  EXPECT_EQ(ms_template.front.points[1].x, 1.0);
  EXPECT_EQ(ms_template.front.points[1].y, 0.0);
  EXPECT_EQ(ms_template.front.points[2].x, 1.0);
  EXPECT_EQ(ms_template.front.points[2].y, -.25);

  range_sensor_t range_sensor;
  range_sensor.angular_resolution = .005;
  range_sensor.min_distance = .1;
  range_sensor.max_distance = 10;
  range_sensor.min_angle = -3.1415;
  range_sensor.max_angle = 3.1415;

  point2d_t sensor_pos = {.x=0, .y=0};
  motion_tube_sensor_space_t motion_tube_sensor_space;
  motion_tube_sensor_space.max_number_of_beams = 20;
  motion_tube_sensor_space.beams = (free_space_beam_t *) 
    malloc(motion_tube_sensor_space.max_number_of_beams * sizeof(free_space_beam_t));
  motion_tube_sensor_space.number_of_beams = 0;
}

TEST(SteerLeftTemplate, ThreePointsPerSide) {
  maneuver_t maneuver;
  unicycle_control_t control;
  maneuver.control = (void *) &control;
  control.forward_velocity = .3;
  control.angular_rate = -M_PI/(3);
  maneuver.time_horizon = 3;

  body_t body;
  polyline_t body_geometry;
  body_geometry.number_of_points = 4;
  body_geometry.points = (point2d_t *) malloc(
    body_geometry.number_of_points * sizeof(point2d_t));
  body_geometry.points[FRONT_LEFT].x = 0.5;
  body_geometry.points[FRONT_LEFT].y = 0.25;
  body_geometry.points[AXLE_LEFT].x = 0.0;
  body_geometry.points[AXLE_LEFT].y = 0.25; 
  body_geometry.points[FRONT_RIGHT].x = 0.5;
  body_geometry.points[FRONT_RIGHT].y = -0.25; 
  body_geometry.points[AXLE_RIGHT].x = 0.0;
  body_geometry.points[AXLE_RIGHT].y = -0.25; 

  body.type = POLYLINE;
  body.geometry= (void *) &body_geometry;

  double sampling_interval = .1;

  template_t ms_template;
  point2d_array_t samples[3];
  for(int i=0; i<3; i++){
    samples[i].number_of_points = 0;
    samples[i].max_number_of_points = 100;
    samples[i].points = (point2d_t *) malloc(
      samples[i].max_number_of_points * sizeof(point2d_t));
  }
  ms_template.left = samples[0];
  ms_template.front = samples[1];
  ms_template.right = samples[2];

  sample_free_space_template_in_cartesian(&maneuver, &body, sampling_interval,
    &ms_template);

  samples[0].number_of_points = ms_template.left.number_of_points;
  samples[1].number_of_points = ms_template.front.number_of_points;
  samples[2].number_of_points = ms_template.right.number_of_points;

  printf("number of points: %d\n", ms_template.left.number_of_points);
  int num = 0;
  for (int k=0; k<3; k++){
    for(int i=0; i<samples[k].number_of_points; i++){
      printf("%d/%d: (%f, %f)\n", k, num, samples[k].points[i].x, samples[k].points[i].y);
      num++;
    }
  }

  range_sensor_t range_sensor;
  range_sensor.angular_resolution = .005;
  range_sensor.min_distance = .1;
  range_sensor.max_distance = 10;
  range_sensor.min_angle = -3.1415;
  range_sensor.max_angle = 3.1415;

  point2d_t sensor_pos = {.x=0, .y=0};
  motion_tube_sensor_space_t motion_tube_sensor_space;
  motion_tube_sensor_space.max_number_of_beams = 20;
  motion_tube_sensor_space.beams = (free_space_beam_t *) 
    malloc(motion_tube_sensor_space.max_number_of_beams * sizeof(free_space_beam_t));
  motion_tube_sensor_space.number_of_beams = 0;
  
    template_to_sensor_space(&ms_template, &range_sensor, &sensor_pos,
    &maneuver, &motion_tube_sensor_space );

  num= 0;
  for (int i=0; i<motion_tube_sensor_space.number_of_beams; i++){
    printf("%d: beam: %d, range: %f\n", num, motion_tube_sensor_space.beams[i].index,
      motion_tube_sensor_space.beams[i].range_outer);
    num++;
  }

}
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


/*
  printf("number of points: %d\n", ms_template.left.number_of_points);
  for(int j=0; j<ms_template.left.number_of_points; j++)
    printf("(%f, %f)\n", ms_template.left.points[j].x, ms_template.left.points[j].y);
  printf("------------\n");

  printf("number of points: %d\n", ms_template.front.number_of_points);
  for(int j=0; j<ms_template.front.number_of_points; j++)
    printf("(%f, %f)\n", ms_template.front.points[j].x, ms_template.front.points[j].y);
  printf("------------\n");

  printf("number of points: %d\n", ms_template.right.number_of_points);
  for(int j=0; j<ms_template.right.number_of_points; j++)
    printf("(%f, %f)\n", ms_template.right.points[j].x, ms_template.right.points[j].y);
  printf("------------\n");
  */