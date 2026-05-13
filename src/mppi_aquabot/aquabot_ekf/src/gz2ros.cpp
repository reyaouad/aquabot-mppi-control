#include <aquabot_ekf/boat.h>
#include <aquabot_ekf/gps2enu.h>

#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace aquabot_ekf
{

using std_msgs::msg::Float64;

// main node
class GPS2Pose : public rclcpp::Node
{
public:
  GPS2Pose() : Node("gz2ros"), boat{this, &enu}
  {
    set_parameter(rclcpp::Parameter("use_sim_time", true));

    enu.setReference(declare_parameter("latitude", 48.04630),
                     declare_parameter("longitude", -4.97632));

    static auto gps_sub = create_subscription<NavSatFix>("gps_fix", 5, [this](NavSatFix::UniquePtr msg)
    {boat.updatePose(*msg);});

    static auto imu_sub = create_subscription<Imu>("imu_raw", 1, [this](Imu::UniquePtr msg)
    {boat.updateImu(*msg);});

    obs_pub = create_publisher<PoseArray>("/aquabot/turbines", 1);
    static auto obs_sub = create_subscription<Pose>("/aquabot/ais_sensor/windturbines_positions", 1, [this](Pose::UniquePtr msg)
    {toPoses(*msg);});

    static auto bearing_sub = create_subscription<Float64>("/aquabot/sensors/acoustics/receiver/bearing", 1, [this](Float64::UniquePtr msg)
      {bearing = msg->data;});
    static auto range_sub = create_subscription<Float64>("/aquabot/sensors/acoustics/receiver/range", 1, [this](Float64::UniquePtr msg)
      {publishPinger(msg->data);});
  }

private:

  toENU enu;
  Boat boat;

  tf2_ros::TransformBroadcaster br{this};
  double bearing{};

  void publishPinger(double range)
  {
    static geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = "aquabot/receiver";
    transform.child_frame_id = "pinger";
    transform.transform.translation.x = range * std::cos(bearing);
    transform.transform.translation.y = range * std::sin(bearing);
    br.sendTransform(transform);
  }

  rclcpp::Publisher<PoseArray>::SharedPtr obs_pub;

  void toPoses(const Pose &msg)
  {
    static PoseArray poses;
    static Pose meters;
    // convert to ENU
    enu.transform(msg.position.x, msg.position.y, 1., meters.position);

    if(std::find(poses.poses.begin(), poses.poses.end(), meters) == poses.poses.end())
      poses.poses.push_back(meters);

    if(poses.poses.size() == 3)
      obs_pub->publish(poses);
  }
};

}



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<aquabot_ekf::GPS2Pose>());
  rclcpp::shutdown();
  return 0;
}
