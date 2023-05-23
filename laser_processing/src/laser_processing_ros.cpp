#include "laser_processing/laser_processing_ros.h"
#include <cmath>
#include <stdlib.h>
#include <ros/console.h>


namespace laser_processing
{

///////////////////////////////////////////////////////////////////////////////
// Constructor / destructor
///////////////////////////////////////////////////////////////////////////////
LaserProcessingROS::LaserProcessingROS(ros::NodeHandle& nh, ros::NodeHandle& nh_local):
  nh_(nh),
  nh_local_(nh_local)
{
  loadParameters();
  infos_ = {0, 0, 0, 0, 0, 0, 0, 0};
  line_subscriber_ = nh_.subscribe("line_segments", 1, &LaserProcessingROS::laserLineCallback, this);
  lanes_publisher_ = nh_.advertise<laser_processing::Lanes>("lanes", 1);
}

LaserProcessingROS::~LaserProcessingROS()
{
}

///////////////////////////////////////////////////////////////////////////////
// Run
///////////////////////////////////////////////////////////////////////////////
void LaserProcessingROS::run()
{
  // ROS_DEBUG("HI");

  // Populate message
  laser_processing::Lanes msg;
  populateLanesMsg(infos_, msg);
  
  // Publish the lines
  lanes_publisher_.publish(msg);
}

///////////////////////////////////////////////////////////////////////////////
// Load ROS parameters
///////////////////////////////////////////////////////////////////////////////
void LaserProcessingROS::loadParameters()
{
  
  ROS_DEBUG("*************************************");
  ROS_DEBUG("PARAMETERS:");

  // Parameters used by this node
  
  std::string frame_id, scan_topic;
  float ref_angle_front, ref_angle_right, ref_distance_front, ref_distance_right;
  bool pub_markers;

  nh_local_.param<std::string>("frame_id", frame_id, "laser");
  frame_id_ = frame_id;
  ROS_DEBUG("frame_id: %s", frame_id_.c_str());

  nh_local_.param<float>("ref_distance_right", ref_distance_right, 0.097);
  ref_distance_right_ = ref_distance_right;
  ROS_DEBUG("ref_distance_right: %3f", ref_distance_right_);

  nh_local_.param<float>("ref_distance_front", ref_distance_front, 0.0);
  ref_distance_front_ = ref_distance_front;
  ROS_DEBUG("ref_distance_front: %3f", ref_distance_front_);

  nh_local_.param<float>("ref_angle_front", ref_angle_front, 3.14);
  ref_angle_front_ = ref_angle_front;
  ROS_DEBUG("ref_angle_front: %3f", ref_angle_front_);

  nh_local_.param<float>("ref_angle_right", ref_angle_right, 1.57);
  ref_angle_right_ = ref_angle_right;
  ROS_DEBUG("ref_angle_right: %3f", ref_angle_right_);

  ROS_DEBUG("*************************************");
}

///////////////////////////////////////////////////////////////////////////////
// Populate messages
///////////////////////////////////////////////////////////////////////////////
void LaserProcessingROS::populateLanesMsg(const std::array<float, 8> &infos,
                                                laser_processing::Lanes &lanes_msg)
{
  lanes_msg.distance_right = infos[0] - ref_distance_right_;
  lanes_msg.distance_left = infos[1] + ref_distance_right_;
  lanes_msg.distance_front = infos[2] - ref_distance_front_;
  lanes_msg.distance_back = infos[3] + ref_distance_front_;
  lanes_msg.angle_right = infos[4];
  lanes_msg.angle_left = infos[5];
  lanes_msg.angle_front = infos[6];
  lanes_msg.angle_back = infos[7];
  
  lanes_msg.header.frame_id = frame_id_;
  lanes_msg.header.stamp = ros::Time::now();
}

///////////////////////////////////////////////////////////////////////////////
// Main LaserScan callback
///////////////////////////////////////////////////////////////////////////////
void LaserProcessingROS::laserLineCallback(const laser_line_extraction::LineSegmentList::ConstPtr &line_msgs)
{
  for (int i=0; i < 4; i++){
    infos_[i] = 0;
    infos_[i+4] = 0;
  }
  std::array<float, 4> max_length = {0.0, 0.0, 0.0, 0.0};
  for (int i=0; i<line_msgs->line_segments.size(); ++i){
    float dist = distBetweenPoints(line_msgs->line_segments[i].start[0], line_msgs->line_segments[i].start[1], line_msgs->line_segments[i].end[0], line_msgs->line_segments[i].end[1]);
    int idx = -1;
    bool change_sign = false;
    if (abs(line_msgs->line_segments[i].angle + ref_angle_front_) < 0.3 | abs(line_msgs->line_segments[i].angle - ref_angle_front_) < 0.3 | abs(line_msgs->line_segments[i].angle - ref_angle_front_ + 3.14) < 0.3){
      if (line_msgs->line_segments[i].start[0] > 0){
        // right 0, 4 -> front
          idx = 1;
        }
        else{
        // left 1, 5 -> back
          idx = 0;
        }
        if ((line_msgs->line_segments[i].start[1] * line_msgs->line_segments[i].end[1]) < 0){
          change_sign = true;
        }
      }
    else if (abs(line_msgs->line_segments[i].angle - ref_angle_right_) < 0.3 | abs(line_msgs->line_segments[i].angle - ref_angle_right_ + 3.14) < 0.3){
      if (line_msgs->line_segments[i].start[1] > 0){
        // front 2, 6 -> left
        idx = 3;
      }
      else{
        // back 3, 7 -> right
        idx = 2;
      }
      if ((line_msgs->line_segments[i].start[0] * line_msgs->line_segments[i].end[0]) < 0){
        change_sign = true;
      }
    }
    if ((idx >= 0) & change_sign){
      max_length[idx] = 100;
      infos_[idx] = line_msgs->line_segments[i].radius;
      infos_[idx+4] = line_msgs->line_segments[i].angle;
    }
    else if ((idx >= 0) & (dist > max_length[idx])){
      max_length[idx] = dist;
      infos_[idx] = line_msgs->line_segments[i].radius;
      infos_[idx+4] = line_msgs->line_segments[i].angle;
    }

    }
  }
  float LaserProcessingROS::distBetweenPoints(float data1_x, float data1_y, float data2_x, float data2_y)
  {
    return sqrt(pow(data1_x - data2_x, 2) + 
                pow(data1_y - data2_y, 2));
  }

}
 // namespace line_extraction

