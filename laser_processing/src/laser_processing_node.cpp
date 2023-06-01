#include "laser_processing/laser_processing_ros.h"
#include <ros/console.h>

int main(int argc, char **argv)
{

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) 
  {
     ros::console::notifyLoggerLevelsChanged();
  }

  ROS_DEBUG("Starting laser processing.");

  ros::init(argc, argv, "laser_processing_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_local("~");
  laser_processing::LaserProcessingROS laser_processor(nh, nh_local);

  double frequency;
  nh_local.param<double>("frequency", frequency, 25);
  ROS_DEBUG("Frequency set to %0.1f Hz", frequency);
  ros::Rate rate(frequency);

  while (ros::ok())
  {
    laser_processor.run();
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

