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

  // setup rendering
  SDL_Event event;
  SDL_Init(SDL_INIT_VIDEO);
  SDL_Window * window = SDL_CreateWindow("Hello AR Drone", SDL_WINDOWPOS_UNDEFINED,
                                         SDL_WINDOWPOS_UNDEFINED, 2560, 1440, 0);//1280, 720, 0);//640, 360, 0);
  SDL_Renderer * renderer = SDL_CreateRenderer(window, -1, 0);
  SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
  SDL_RenderClear(renderer);
  SDL_RenderPresent(renderer);
  SDL_Texture * texture;

  // enter main event loop
  std::cout << "===== Hello AR Drone ====" << std::endl;
  cv::Mat image;
  while (ros::ok()) {
    ros::spinOnce();
    ros::Duration dur(0.04);
    dur.sleep();
    SDL_PollEvent(&event);
    if (event.type == SDL_QUIT) {
      break;
    }

    // render image, if there is a new one available
    if(subscriber.getLastImage(image)) {

      // TODO: add overlays to the cv::Mat image, e.g. text
      //cv::Mat overlay, overlay_in; 
      //overlay_in = cv::imread("/home/ingo/ardrone_ws/src/ardrone_practicals_2022/src/drone_control.JPG");

      //cv::resize(overlay_in, overlay, cv::Size(), 0.15, 0.15);
      //cv::addWeighted(image,0.4,overlay,0.1,0,image);
      //cv::Mat insetImage(image, cv::Rect(20, 20, overlay.cols, overlay.rows));
      //overlay.copyTo(insetImage);

      // draw w-a-s-d control
      cv::putText(image, "[W]/[S]: up/down         ", cv::Point(5, image.rows - 100), cv::FONT_HERSHEY_DUPLEX, .6, CV_RGB(0, 0, 255), 1);
      cv::putText(image, "[A]/[D]: yaw left/right  ", cv::Point(5, image.rows - 75), cv::FONT_HERSHEY_DUPLEX, .6, CV_RGB(0, 0, 255), 1);

      // draw arrow control
      cv::putText(image, "[^]/[v]: forward/backward", cv::Point(5, image.rows - 35), cv::FONT_HERSHEY_DUPLEX, .6, CV_RGB(0, 0, 255), 1);
      cv::putText(image, "[<]/[>]: left/right      ", cv::Point(5, image.rows - 10), cv::FONT_HERSHEY_DUPLEX, .6, CV_RGB(0, 0, 255), 1);
      
      // draw t,l,e control
      cv::putText(image, "[T]/[L]: takeoff/landing ", cv::Point(image.cols - 250, image.rows - 35), cv::FONT_HERSHEY_DUPLEX, .6, CV_RGB(0, 255, 0), 1);
      cv::putText(image, "[Esc]: shut-off motors   ", cv::Point(image.cols - 250, image.rows - 10), cv::FONT_HERSHEY_DUPLEX, .6, CV_RGB(255, 0, 0), 1);

      // draw battery state of charge
      int batterySoC = (int) autopilot.batteryStateOfCharge();
      // make battery red if SoC < 25% (otherwise white)
      cv::Scalar colorBattery;
      if(batterySoC < 25){
        colorBattery = CV_RGB(255, 0, 0);
      }else{
        colorBattery = CV_RGB(255, 255, 255);
      }
      std::string batteryText = "Battery: " + std::to_string(batterySoC) + "%";
      cv::putText(image, batteryText, cv::Point(image.cols - 125, 15), cv::FONT_HERSHEY_DUPLEX, .6, colorBattery, 1);

      // draw current drone status
      std::string stateAsString;
      switch (autopilot.droneStatus())
      {
      case 0:
        stateAsString = "Unknown";
        break;
      case 1:
        stateAsString = "Inited";
        break;
      case 2:
        stateAsString = "Landed";
        break;
      case 3:
        stateAsString = "Flying";
        break;
      case 4:
        stateAsString = "Hovering";
        break;
      case 5:
        stateAsString = "Test";
        break;
      case 6:
        stateAsString = "TakingOff";
        break;
      case 7:
        stateAsString = "Flying2";
        break;
      case 8:
        stateAsString = "Landing";
        break;
      case 9:
        stateAsString = "Looping";
        break;
      default:
        break;
      }
      std::string statusText = "Status: " + stateAsString;
      cv::putText(image, statusText, cv::Point(5, 15), cv::FONT_HERSHEY_DUPLEX, .6, CV_RGB(255, 255, 255), 1);
      
      // https://stackoverflow.com/questions/22702630/converting-cvmat-to-sdl-texture
      // I'm using SDL_TEXTUREACCESS_STREAMING because it's for a video player, you should
      // pick whatever suits you most: https://wiki.libsdl.org/SDL_TextureAccess
      // remember to pick the right SDL_PIXELFORMAT_* !
      texture = SDL_CreateTexture(
          renderer, SDL_PIXELFORMAT_BGR24, SDL_TEXTUREACCESS_STREAMING, image.cols, image.rows);
      SDL_UpdateTexture(texture, NULL, (void*)image.data, image.step1());
      SDL_RenderClear(renderer);
      SDL_RenderCopy(renderer, texture, NULL, NULL);
      // Show our drawing
      SDL_RenderPresent(renderer);
      // cleanup (only after you're done displaying. you can repeatedly call UpdateTexture without destroying it)
      SDL_DestroyTexture(texture);
    }

    //Multiple Key Capture Begins
    const Uint8 *state = SDL_GetKeyboardState(NULL);

    // check states!
    auto droneStatus = autopilot.droneStatus();
    // command
    if (state[SDL_SCANCODE_ESCAPE]) {
      std::cout << "ESTOP PRESSED, SHUTTING OFF ALL MOTORS status=" << droneStatus;
      bool success = autopilot.estopReset();
      if(success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }
    if (state[SDL_SCANCODE_T]) {
      std::cout << "Taking off...                          status=" << droneStatus;
      bool success = autopilot.takeoff();
      if (success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }

    if (state[SDL_SCANCODE_L]) {
      std::cout << "Going to land...                       status=" << droneStatus;
      bool success = autopilot.land();
      if (success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }
    if (state[SDL_SCANCODE_C]) {
      std::cout << "Requesting flattrim calibration...     status=" << droneStatus;
      bool success = autopilot.flattrimCalibrate();
      if (success) {
        std::cout << " [ OK ]" << std::endl;
      } else {
        std::cout << " [FAIL]" << std::endl;
      }
    }

    // TODO: process moving commands when in state 3,4, or 7
    if (droneStatus == autopilot.Flying || droneStatus == autopilot.Hovering || droneStatus == autopilot.Flying2) {
      std::cout << "Sending move command...               status=" << droneStatus;
      // compute manual move values
      double forward = state[SDL_SCANCODE_UP]   - state[SDL_SCANCODE_DOWN];
      double left    = state[SDL_SCANCODE_LEFT] - state[SDL_SCANCODE_RIGHT];
      double up      = state[SDL_SCANCODE_W]    - state[SDL_SCANCODE_S];
      double yawLeft = state[SDL_SCANCODE_A]    - state[SDL_SCANCODE_D];
      bool success = autopilot.manualMove(forward, left, up, yawLeft);
      if (success)
      {
        std::cout << " [ OK ] " << std::endl;
      } else {
        std::cout << " [FAIL] " << std::endl;
      }
    } 
  }

  // make sure to land the drone...
  bool success = autopilot.land();

  // cleanup
  SDL_DestroyTexture(texture);
  SDL_DestroyRenderer(renderer);
  SDL_DestroyWindow(window);
  SDL_Quit();
}

