#include <memory>
#include <unistd.h>
#include <stdlib.h>

#include <SDL2/SDL.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>

#include <arp/Autopilot.hpp>
#include <arp/cameras/PinholeCamera.hpp>
#include <arp/cameras/RadialTangentialDistortion.hpp>

#include <Commands.hpp>
#include <Renderer.hpp>

class Subscriber
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    uint64_t timeMicroseconds = uint64_t(msg->header.stamp.sec) * 1000000ll
        + msg->header.stamp.nsec / 1000;
    // -- for later use
    std::lock_guard<std::mutex> l(imageMutex_);
    lastImage_ = cv_bridge::toCvShare(msg, "bgr8")->image;
  }

  bool getLastImage(cv::Mat& image)
  {
    std::lock_guard<std::mutex> l(imageMutex_);
    if (lastImage_.empty())
      return false;
    image = lastImage_.clone();
    lastImage_ = cv::Mat();  // clear, only get same image once.
    return true;
  }

  void imuCallback(const sensor_msgs::ImuConstPtr& msg)
  {
    // -- for later use
  }

 private:
  cv::Mat lastImage_;
  std::mutex imageMutex_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arp_node");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  // setup inputs
  Subscriber subscriber;
  image_transport::Subscriber subImage = it.subscribe(
      "ardrone/front/image_raw", 2, &Subscriber::imageCallback, &subscriber);
  ros::Subscriber subImu = nh.subscribe("ardrone/imu", 50,
                                        &Subscriber::imuCallback, &subscriber);

  // set up autopilot
  arp::Autopilot autopilot(nh);
  
  // set up camera model

  // check if at least one param exists, heuristic
  if (!nh.hasParam("/arp_node/k1")) {
    ROS_ERROR("Couldn't find parameter '/arp_node/k1'. Make sure you set up the camera parameters correctly.");
    return -1;
  }
  double k1, k2, p1, p2, fu, fv, cu, cv;
  nh.getParam("/arp_node/k1", k1);
  nh.getParam("/arp_node/k2", k2);
  nh.getParam("/arp_node/p1", p1);
  nh.getParam("/arp_node/p2", p2);
  nh.getParam("/arp_node/fu", fu);
  nh.getParam("/arp_node/fv", fv);
  nh.getParam("/arp_node/cu", cu);
  nh.getParam("/arp_node/cv", cv);
  std::cout << "k^T = [" << k1 << ", " << k2 << ", " << p1 << ", " << p2 << ", 0]" << std::endl;
  std::cout << "camera: fu="  << fu << ", fv=" << fv << ", cu=" << cu << ", cv=" << cv << std::endl;
  arp::cameras::RadialTangentialDistortion distortion{k1, k2, p1, p2};
  // arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> phc{
  //   gui::DRONE_CAM_IMAGE_WIDTH, gui::DRONE_CAM_IMAGE_HEIGHT, fu, fv, cu, cv, distortion};
  arp::cameras::PinholeCamera<arp::cameras::RadialTangentialDistortion> phc{
    gui::WINDOW_WIDTH, gui::WINDOW_HEIGHT, fu, fv, cu, cv, distortion};
  phc.initialiseUndistortMaps();

  // setup rendering
  gui::Renderer renderer{phc};

  // enter main event loop
  std::cout << "===== Hello AR Drone ====" << std::endl;
  cv::Mat image;
  while (ros::ok()) {
    ros::spinOnce();
    ros::Duration dur(0.04);
    dur.sleep();
    if(renderer.checkQuit()) {
      break;
    }

    // render image, if there is a new one available
    if(subscriber.getLastImage(image)) {
      renderer.render(image, autopilot.droneStatus(), autopilot.getBatteryLevel());
    }

    // Check if keys are pressed and execute associated commands
    Commands::checkKeysForCommand(autopilot, renderer);
  }

  // make sure to land the drone...
  bool success = autopilot.land();
}

