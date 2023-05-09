#ifndef LASER_PROCESSING_ROS_H
#define LASER_PROCESSING_ROS_H

#include <vector>
#include <string>
#include <ros/ros.h>
#include "laser_line_extraction/LineSegment.h"
#include "laser_line_extraction/LineSegmentList.h"
#include "laser_processing/Lanes.h"

namespace laser_processing
{

class LaserProcessingROS
{

public:
  // Constructor / destructor
  LaserProcessingROS(ros::NodeHandle&, ros::NodeHandle&);
  ~LaserProcessingROS();
  // Running
  void run();

private:
  // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;
  ros::Subscriber line_subscriber_;
  ros::Publisher lanes_publisher_;

  std::array<float, 8> infos_;

  // Parameters
  std::string frame_id_;
  float ref_angle_front_;
  float ref_angle_right_;
  
  // Members
  void loadParameters();
  void populateLanesMsg(const std::array<float, 8>&, laser_processing::Lanes &);
  void laserLineCallback(const laser_line_extraction::LineSegmentList::ConstPtr &);
  float distBetweenPoints(float data1_x, float data1_y, float data2_x, float data2_y);

};

} // namespace line_extraction

#endif
