/**
 * @file /src/kobuki_bumper2pc.cpp
 *
 * @brief Bumper to pointcloud nodelet class implementation.
 *
 * @author Jorge Santos, Yujin Robot
 *
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <pluginlib/class_list_macros.h>

#include "chris_turtlebot_bumper2pc/chris_turtlebot_bumper2pc.hpp"

namespace chris_turtlebot_bumper2pc
{

void Bumper2PcNodelet::coreSensorCB(const kobuki_msgs::SensorState::ConstPtr& msg)
{

  if (pointcloud_pub_.getNumSubscribers() == 0)
    return;

  // We publish just one "no events" pc (with all three points far away) and stop spamming when bumper/cliff conditions disappear
  if (! msg->bumper && ! msg->cliff && ! prev_bumper && ! prev_cliff)
    return;

  prev_bumper = msg->bumper;
  prev_cliff  = msg->cliff;

  // We replicate the sensors order of bumper/cliff event messages: LEFT = 0, CENTER = 1 and RIGHT = 2
  // For any of {left/center/right} with no bumper/cliff event, we publish a faraway point that won't get used
  if ((msg->bumper & kobuki_msgs::SensorState::BUMPER_LEFT) ||
      (msg->cliff  & kobuki_msgs::SensorState::CLIFF_LEFT))
  {
    memcpy(&pointcloud_.data[0], p_bumper_data_, points_per_bumper_*pointcloud_.point_step);
  }
  else
  {
    memcpy(&pointcloud_.data[0], p_max_data_   , points_per_bumper_*pointcloud_.point_step);
  }

  if ((msg->bumper & kobuki_msgs::SensorState::BUMPER_CENTRE) ||
      (msg->cliff  & kobuki_msgs::SensorState::CLIFF_CENTRE))
  {
    memcpy(&pointcloud_.data[1 * points_per_bumper_*pointcloud_.point_step ],
             &p_bumper_data_[1 * points_per_bumper_*3 ], points_per_bumper_*pointcloud_.point_step);
  }
  else
  {
    memcpy(&pointcloud_.data[1 *points_per_bumper_*pointcloud_.point_step],
                &p_max_data_[1 *points_per_bumper_*3], points_per_bumper_*pointcloud_.point_step);
  }

  if ((msg->bumper & kobuki_msgs::SensorState::BUMPER_RIGHT) ||
      (msg->cliff  & kobuki_msgs::SensorState::CLIFF_RIGHT))
  {
    memcpy(&pointcloud_.data[2 * points_per_bumper_*pointcloud_.point_step ],
             &p_bumper_data_[2 * points_per_bumper_*3 ], points_per_bumper_*pointcloud_.point_step);
  }
  else
  {
    memcpy(&pointcloud_.data[2 *points_per_bumper_*pointcloud_.point_step],
                &p_max_data_[2 *points_per_bumper_*3], points_per_bumper_*pointcloud_.point_step);
  }

  pointcloud_.header.stamp = msg->header.stamp;
  pointcloud_pub_.publish(pointcloud_);
}

void Bumper2PcNodelet::onInit()
{
  ros::NodeHandle nh = this->getPrivateNodeHandle();

  // Bumper/cliff pointcloud distance to base frame; should be something like the robot radius plus
  // costmap resolution plus an extra to cope with robot inertia. This is a bit tricky parameter: if
  // it's too low, costmap will ignore this pointcloud (the robot footprint runs over the hit obstacle),
  // but if it's too big, hit obstacles will be mapped too far from the robot and the navigation around
  // them will probably fail.
  std::string base_link_frame;
  double r, h, angle,max_radius, pointcloud_thickness;
  int increments_per_bumper = 0;
  nh.param("points_per_bumper", increments_per_bumper, 3);
  nh.param("pointcloud_max_radius", max_radius, 1000.0);
  nh.param("pointcloud_radius", r, 0.25); pc_radius_ = r;
  nh.param("pointcloud_thickness", pointcloud_thickness, 0.025);
  nh.param("pointcloud_height", h, 0.04); pc_height_ = h;
  nh.param("side_point_max_angle", angle, 1.39626);
  nh.param<std::string>("base_link_frame", base_link_frame, "/base_link");

  if (points_per_bumper_ < 1)
    points_per_bumper_ = 1;

  int rows; // of points
  if (pointcloud_thickness > 0.0)
  {
      points_per_bumper_ = 2*increments_per_bumper;
      rows = 2;
  }
  else
  {
      points_per_bumper_ = increments_per_bumper;
      rows = 1;
  }

  // Store float values as (x,y,z) tuples to memcopy later
  // 3 coordinates x 3 bumpers x ppb
  p_bumper_data_ = new float[3*3*points_per_bumper_];
  p_max_data_    = new float[3*3*points_per_bumper_];

  double starting_angle = angle;

  // Left side processed first is positive angle
  double angle_increment = -2*angle/(3*increments_per_bumper - 1);
  ROS_INFO("Bumper/cliff pointcloud configuring at distance %f, thickness %f, and height %f from base frame with %d increments per bumper and %d total float values",
           pc_radius_, pointcloud_thickness,  pc_height_,increments_per_bumper,3*3*points_per_bumper_ );


  for (int i=0; i < 3*increments_per_bumper; i++, angle += angle_increment)
  { // left-center-right

    p_bumper_data_[i*3*rows + 0] = pc_radius_*cos(angle); // x-coordinate
    p_bumper_data_[i*3*rows + 1] = pc_radius_*sin(angle); // y-coordinate
    p_bumper_data_[i*3*rows + 2] = pc_height_;            // z-coordinate

    p_max_data_[i*3*rows + 0] = max_radius*cos(angle);   // x-coordinate
    p_max_data_[i*3*rows + 1] = max_radius*sin(angle);   // y-coordinate
    p_max_data_[i*3*rows + 2] = pc_height_;              // z-coordinate

    //ROS_INFO("   %3d : (%f, %f, %f)",i*3*rows, p_bumper_data_[i*3*rows + 0], p_bumper_data_[i*3*rows + 1], p_bumper_data_[i*3*rows + 2]);
    if (rows > 1)
    {
        p_bumper_data_[i*3*rows + 3] = (pc_radius_+pointcloud_thickness)*cos(angle); // x-coordinate
        p_bumper_data_[i*3*rows + 4] = (pc_radius_+pointcloud_thickness)*sin(angle); // y-coordinate
        p_bumper_data_[i*3*rows + 5] = pc_height_;                                   // z-coordinate
        //ROS_INFO("  *%3d : (%f, %f, %f)",i*3*rows+3, p_bumper_data_[i*3*rows + 3], p_bumper_data_[i*3*rows + 4], p_bumper_data_[i*3*rows + 5]);

        p_max_data_[i*3*rows + 3] = max_radius*cos(angle);   // x-coordinate
        p_max_data_[i*3*rows + 4] = max_radius*sin(angle);   // y-coordinate
        p_max_data_[i*3*rows + 5] = pc_height_;              // z-coordinate
    }

  }


  // Prepare constant parts of the pointcloud message to be  published
  pointcloud_.header.frame_id = base_link_frame;
  pointcloud_.width  = 3 * points_per_bumper_;
  pointcloud_.height = 1;
  pointcloud_.fields.resize(3); // (x,y,z)

  // Set x/y/z as the only fields
  pointcloud_.fields[0].name = "x";
  pointcloud_.fields[1].name = "y";
  pointcloud_.fields[2].name = "z";

  int offset = 0;
  // All offsets are *4, as all field data types are float32
  for (size_t d = 0; d < pointcloud_.fields.size(); ++d, offset += sizeof(float))
  {
    pointcloud_.fields[d].count    = 1;
    pointcloud_.fields[d].offset   = offset;
    pointcloud_.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
  }

  pointcloud_.point_step = offset;
  pointcloud_.row_step   = pointcloud_.point_step * pointcloud_.width;

  pointcloud_.data.resize(3 * points_per_bumper_ * pointcloud_.point_step);
  pointcloud_.is_bigendian = false;
  pointcloud_.is_dense     = true;

  // Bumper/cliff "points" fix coordinates (the others depend on sensor activation/deactivation)
  memcpy(&pointcloud_.data[0], &p_max_data_[0], pointcloud_.row_step);

  pointcloud_pub_  = nh.advertise <sensor_msgs::PointCloud2> ("pointcloud", 10);
  core_sensor_sub_ = nh.subscribe("core_sensors", 10, &Bumper2PcNodelet::coreSensorCB, this);

  ROS_INFO("Bumper/cliff pointcloud configured at distance %f and height %f from base frame", pc_radius_, pc_height_);
}

} // namespace kobuki_bumper2pc


PLUGINLIB_EXPORT_CLASS(chris_turtlebot_bumper2pc::Bumper2PcNodelet, nodelet::Nodelet);
