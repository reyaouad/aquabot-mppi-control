#ifndef BOAT_H
#define BOAT_H

#include <rclcpp/node.hpp>
#include <aquabot_ekf/gps2enu.h>
#include <aquabot_ekf/covariances.h>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

using namespace std::chrono_literals;

namespace aquabot_ekf
{

struct Boat
{
  rclcpp::Node* node;
  toENU* enu;

  // forwarded messages
  PoseWithCovarianceStamped pose;
  Imu imu;
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pose_pub;
  rclcpp::Publisher<Imu>::SharedPtr imu_pub;

  Boat(rclcpp::Node* node, toENU* enu) : node{node}, enu{enu}
  {
    // init pose and imu messages
    pose.header.frame_id = "world";
    imu.header.frame_id = "aquabot/imu_link";

    for(auto i: {0, 7})
      pose.pose.covariance[i] = Covariances::xy;
    pose.pose.covariance[14] = Covariances::z;
    for(auto i: {21, 28, 35})
      pose.pose.covariance[i] = Covariances::rpy;

    for(auto i: {0, 4, 8})
    {
      imu.linear_acceleration_covariance[i] = Covariances::a;
      imu.angular_velocity_covariance[i] = Covariances::w;
      imu.orientation_covariance[i] = Covariances::rpy;
    }

    pose_pub = node->create_publisher<PoseWithCovarianceStamped>("gps_xy", rclcpp::SensorDataQoS());
    imu_pub = node->create_publisher<Imu>("imu", rclcpp::SensorDataQoS());

  }

  inline void updatePose(const NavSatFix &gps)
  {
    enu->transform(gps.latitude, gps.longitude, gps.altitude,
                   pose.pose.pose.position);

    pose.pose.pose.position.z -= 1.62;  // not quite
    pose.header.stamp = node->get_clock()->now();
    pose_pub->publish(pose);
  }

  inline void updateImu(const Imu &imu)
  {
    this->imu.orientation = imu.orientation;
    this->imu.angular_velocity = imu.angular_velocity;
    //this->imu.linear_acceleration = imu.linear_acceleration;

    this->imu.header.stamp = node->get_clock()->now();
    imu_pub->publish(this->imu);
  }
};
}


#endif // BOAT_H
