// Bring in my package's API, which is what I'm testing
#include "arp/cameras/PinholeCamera.hpp"
#include "arp/cameras/RadialTangentialDistortion.hpp"
#include "arp/cameras/DistortionBase.hpp"
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

// Test the projection/unprojection and in addition check the analytical Jacobians by comparing them to central differences implementation.
TEST(PinholeCamera, projectBackProject_Jacobian)
{
  for (int i = 0; i < 100; i++) {
    // create a random visible point in the camera coordinate frame C
    auto point_C = pinholeCamera.createRandomVisiblePoint();
    
    // project
    Eigen::Vector2d imagePoint;
    Eigen::Matrix<double, 2, 3> projectJacobian;
    pinholeCamera.project(point_C, &imagePoint, &projectJacobian);

    // backProject
    Eigen::Vector3d ray_C;
    pinholeCamera.backProject(imagePoint,&ray_C);

    // now they should align:
    EXPECT_TRUE(fabs(ray_C.normalized().transpose()*point_C.normalized()-1.0)<1.0e-10);

    // check correctness of analytical Jacobian
    double delta = 1.0e-10;
    Eigen::Vector3d delta_x(delta, 0, 0);    // small step
    Eigen::Vector3d delta_y(0, delta, 0);    // small step
    Eigen::Vector3d delta_z(0, 0, delta);    // small step
    auto point_C_x_minus = point_C - delta_x;
    auto point_C_x_plus = point_C + delta_x;
    auto point_C_y_minus = point_C - delta_y;
    auto point_C_y_plus = point_C + delta_y;
    auto point_C_z_minus = point_C - delta_z;
    auto point_C_z_plus = point_C + delta_z;

    // project
    Eigen::Vector2d imagePoint_x_minus, imagePoint_x_plus, imagePoint_y_minus, imagePoint_y_plus, imagePoint_z_minus, imagePoint_z_plus;
    EXPECT_TRUE(arp::cameras::ProjectionStatus::Successful == pinholeCamera.project(point_C_x_minus,&imagePoint_x_minus));
    EXPECT_TRUE(arp::cameras::ProjectionStatus::Successful == pinholeCamera.project(point_C_x_plus,&imagePoint_x_plus));
    EXPECT_TRUE(arp::cameras::ProjectionStatus::Successful == pinholeCamera.project(point_C_y_minus,&imagePoint_y_minus));
    EXPECT_TRUE(arp::cameras::ProjectionStatus::Successful == pinholeCamera.project(point_C_y_plus,&imagePoint_y_plus));
    EXPECT_TRUE(arp::cameras::ProjectionStatus::Successful == pinholeCamera.project(point_C_z_minus,&imagePoint_z_minus));
    EXPECT_TRUE(arp::cameras::ProjectionStatus::Successful == pinholeCamera.project(point_C_z_plus,&imagePoint_z_plus));

    // compute central differences
    Eigen::Matrix<double, 2, 3> centralDifferences;
    centralDifferences << (imagePoint_x_plus[0] - imagePoint_x_minus[0])/(2.0*delta), (imagePoint_y_plus[0] - imagePoint_y_minus[0])/(2.0*delta), (imagePoint_z_plus[0] - imagePoint_z_minus[0])/(2.0*delta),
                          (imagePoint_x_plus[1] - imagePoint_x_minus[1])/(2.0*delta), (imagePoint_y_plus[1] - imagePoint_y_minus[1])/(2.0*delta), (imagePoint_z_plus[1] - imagePoint_z_minus[1])/(2.0*delta);
    std::cout << std::endl << "Analytical Jacobian: " << projectJacobian << std::endl;
    std::cout << std::endl << "Central Differences: " << centralDifferences << std::endl;
    
    // compare analytical Jacobian with Jacobian from central differences
    double epsilon = 1.0e-5; // precision
    EXPECT_TRUE(projectJacobian.isApprox(centralDifferences, epsilon));
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
  // TODO, more cases with z positive and |x|, |y| large enough to be out of the image 
  Eigen::Vector3d point{100000000, 0, 1};
  Eigen::Vector2d imagePoint;
  auto status = pinholeCamera.project(point, &imagePoint);
  EXPECT_EQ(status, arp::cameras::ProjectionStatus::OutsideImage);
}

TEST(PinholeCamera, project_zEquals0_yields_ProjectionStatusInvalid)
{
  const double& eps = arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion>::Z_EPSILON;
  double newEpsilon;
  Eigen::Vector2d imagePoint;
  Eigen::Matrix<double, 2, 3> J;
  for (int i = 0; i < 1000; i++) {
    auto point_C = pinholeCamera.createRandomVisiblePoint();

    // Test 95% of the range [0,Z_EPSILON]
    // eps*0.95/1000*i, in log space for numerical stability
    newEpsilon = std::exp(std::log(eps) + std::log(0.95) - std::log(1000) + std::log(i));
    // moving point_C along its projection ray until its Z component reaches newEpsilon
    point_C = point_C / point_C(2) * newEpsilon;

    // without jacobian
    auto status = pinholeCamera.project(point_C, &imagePoint);
    EXPECT_EQ(status, arp::cameras::ProjectionStatus::Invalid);

    // with jacobian
    status = pinholeCamera.project(point_C, &imagePoint, &J);
    EXPECT_EQ(status, arp::cameras::ProjectionStatus::Invalid);
  }  
}

// The only other way (except z~0) to get ProjectionStatus::Invalid is for an error to occur in
// `distortion_t::distort`. Since the current implementations (`NoDistortion` and
// `RadialTangentialDistortion`) always return true, We cannot sensibly cover this path.
// This dummy distortion class acts as fuse that we blow on purpose for extended code coverage
class BrakingDistortion : public arp::cameras::DistortionBase {
 public:
  bool distort(const Eigen::Vector2d & pointUndistorted, 
                       Eigen::Vector2d * pointDistorted) const override 
  { 
    return false; 
  }

  bool distort(const Eigen::Vector2d & pointUndistorted,
                       Eigen::Vector2d * pointDistorted,
                       Eigen::Matrix2d * pointJacobian) const override
  {
    return false;
  }

  bool undistort(const Eigen::Vector2d & pointDistorted,
                         Eigen::Vector2d * pointUndistorted) const override
  {
    return false;
  }

  static BrakingDistortion testObject() {
    return BrakingDistortion{};
  }
};

TEST(PinholeCamera, project_distortionError_yields_ProjectionStatusInvalid)
{
  auto brokenCamera = arp::cameras::PinholeCamera<BrakingDistortion>::testObject();
  Eigen::Vector2d imagePoint;
  Eigen::Matrix<double, 2, 3> J;
  for (int i = 0; i < 1000; i++) {
    auto point_C = brokenCamera.createRandomVisiblePoint();

    // without jacobian
    auto status = brokenCamera.project(point_C, &imagePoint);
    EXPECT_EQ(status, arp::cameras::ProjectionStatus::Invalid);

    // with jacobian
    status = brokenCamera.project(point_C, &imagePoint, &J);
    EXPECT_EQ(status, arp::cameras::ProjectionStatus::Invalid);
  }  
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

