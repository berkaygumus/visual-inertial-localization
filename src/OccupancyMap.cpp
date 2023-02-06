#include <OccupancyMap.h>
#include <fstream>
#include <ros/ros.h>

OccupancyMap::OccupancyMap(const std::string& filename)
{
  // open the file:
  std::ifstream mapFile(filename, std::ios::in | std::ios::binary);
  if(!mapFile.is_open()) {
  ROS_FATAL_STREAM("could not open map file " << filename);
  }
  // first read the map size along all the dimensions:
  // int sizes[3];
  int* sizes = dimensions_.data();
  if(!mapFile.read((char*)sizes, 3*sizeof(int))) {
  ROS_FATAL_STREAM("could not read map file " << filename);
  }
  // now read the map data
  mapData_.reserve(sizes[0]*sizes[1]*sizes[2]);
  if(!mapFile.read((char*)mapData_.data(), sizes[0]*sizes[1]*sizes[2])) {
  ROS_FATAL_STREAM("could not read map file " << filename);
  }
  mapFile.close();
  // now wrap it with a cv::Mat for easier access:
  wrapped_ = cv::Mat{3, sizes, CV_8SC1, mapData_.data()};

  std::cout << "Loaded occupancy map \"" << filename << "\"." << std::endl;
}

OccupancyMap::operator const cv::Mat&() const
{
    return wrapped_;
}

const OccupancyMap::Dimensions& OccupancyMap::dimensions() const
{
  return dimensions_;
}

double OccupancyMap::at(int x, int y, int z) const
{
  // TODO pull out using data_.at<double>()?
  return static_cast<double>(wrapped_.at<char>(x, y, z));
}