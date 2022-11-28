// Bring in my package's API, which is what I'm testing
#include "arp/cameras/PinholeCamera.hpp"
#include "arp/cameras/RadialTangentialDistortion.hpp"
// Bring in gtest
#include <gtest/gtest.h>

#include <iostream>

// create an arbitrary camera model
auto pinholeCamera = arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>::testObject();

// Test the projection and unprojection
TEST(PinholeCamera, projectBackProject)
{
  for (int i = 0; i < 1000; i++) {
    // create a random visible point in the camera coordinate frame C
    auto point_C = pinholeCamera.createRandomVisiblePoint();

    // project
    Eigen::Vector2d imagePoint;
    auto status = pinholeCamera.project(point_C,&imagePoint);
    EXPECT_EQ(status, arp::cameras::ProjectionStatus::Successful);

    // // backProject
    Eigen::Vector3d ray_C;
    EXPECT_TRUE(pinholeCamera.backProject(imagePoint,&ray_C));

    // // now they should align:
    EXPECT_TRUE(fabs(ray_C.normalized().transpose()*point_C.normalized()-1.0)<1.0e-10);
  }
}

TEST(PinholeCamera, project_negativeZ_yields_ProjectionStatusBehind)
{
  for (int i = 0; i < 1000; i++) {
    auto point_C = pinholeCamera.createRandomVisiblePoint();
    Eigen::Vector2d imagePoint;
    point_C(2) = -std::abs(point_C(2)); // negative z
    auto status = pinholeCamera.project(point_C, &imagePoint);
    EXPECT_EQ(status, arp::cameras::ProjectionStatus::Behind);
  }
}

TEST(PinholeCamera, project_outOfFov_yields_ProjectionStatusOutsideImage)
{
  for (int i = 0; i < 1000; i++) {
    auto point_C = pinholeCamera.createRandomUnvisiblePoint();
    Eigen::Vector2d imagePoint;
    auto status = pinholeCamera.project(point_C, &imagePoint);

    EXPECT_EQ(status, arp::cameras::ProjectionStatus::OutsideImage);
  } 
}

//    RDT can't _not_ work, hence the condition for `ProjectionStatus::Invalid` is unreachable in
//    the pinhole camera with RDT
// TEST(PinholeCamera, project_distortionError_yields_ProjectionStatusInvalid)
// {
//   EXPECT_TRUE(false);
// }

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

