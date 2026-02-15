#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nexus_base_ros/msg/encoders.hpp"

namespace {
constexpr double kDefaultUpdateRateHz = 50.0;
constexpr double kDefaultWheelSeparationWidth = 0.3;
constexpr double kDefaultWheelSeparationLength = 0.3;
}

class NexusOdometry : public rclcpp::Node {
public:
  NexusOdometry()
  : Node("nexus_odometry"),
    x_pos_(0.0),
    y_pos_(0.0),
    heading_(0.0) {
    frame_id_ = declare_parameter<std::string>("frame_id", "odom");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    wheel_separation_width_ = declare_parameter<double>("wheel_separation_width", kDefaultWheelSeparationWidth);
    wheel_separation_length_ = declare_parameter<double>("wheel_separation_length", kDefaultWheelSeparationLength);
    update_rate_hz_ = declare_parameter<double>("update_rate_hz", kDefaultUpdateRateHz);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("sensor_odom", rclcpp::QoS(10).reliable());
    wheel_sub_ = create_subscription<nexus_base_ros::msg::Encoders>(
      "wheel_vel",
      rclcpp::SensorDataQoS(),
      std::bind(&NexusOdometry::wheelCallback, this, std::placeholders::_1));

    if (publish_tf_) {
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    auto period = std::chrono::duration<double>(1.0 / std::max(1e-3, update_rate_hz_));
    update_timer_ = create_wall_timer(period, std::bind(&NexusOdometry::update, this));
    last_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

private:
  void wheelCallback(const nexus_base_ros::msg::Encoders::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(sample_mutex_);
    sample_.valid = true;
    sample_.stamp = msg->header.stamp;
    sample_.lf = msg->enc0;
    sample_.lr = msg->enc1;
    sample_.rr = msg->enc2;
    sample_.rf = msg->enc3;
  }

  void update() {
    WheelSample sample;
    {
      std::lock_guard<std::mutex> lock(sample_mutex_);
      sample = sample_;
    }
    if (!sample.valid) {
      return;
    }

    rclcpp::Time current_time = sample.stamp;
    if (last_time_.nanoseconds() == 0) {
      last_time_ = current_time;
      return;
    }
    double dt = (current_time - last_time_).seconds();
    if (dt <= 0.0) {
      return;
    }
    last_time_ = current_time;

    double v_lf = static_cast<double>(sample.lf) / 1000.0;
    double v_lr = static_cast<double>(sample.lr) / 1000.0;
    double v_rr = static_cast<double>(sample.rr) / 1000.0;
    double v_rf = static_cast<double>(sample.rf) / 1000.0;

    double k = (wheel_separation_width_ / 2.0) + (wheel_separation_length_ / 2.0);
    double linear_x = (-v_lf + v_rf - v_lr + v_rr) / 4.0;
    double linear_y = (v_lf + v_rf - v_lr - v_rr) / 4.0;
    double angular_z = -(v_lf + v_rf + v_lr + v_rr) / (4.0 * k);

    double delta_heading = angular_z * dt;
    double delta_x = (linear_x * std::cos(heading_) - linear_y * std::sin(heading_)) * dt;
    double delta_y = (linear_x * std::sin(heading_) + linear_y * std::cos(heading_)) * dt;

    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;

    tf2::Quaternion odom_quat;
    odom_quat.setRPY(0.0, 0.0, heading_);

    nav_msgs::msg::Odometry odom;
    {
      uint64_t tns = current_time.nanoseconds();
      odom.header.stamp.sec = static_cast<int32_t>(tns / 1000000000ULL);
      odom.header.stamp.nanosec = static_cast<uint32_t>(tns % 1000000000ULL);
    }
    odom.header.frame_id = frame_id_;
    odom.child_frame_id = child_frame_id_;
    odom.pose.pose.position.x = x_pos_;
    odom.pose.pose.position.y = y_pos_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = odom_quat.x();
    odom.pose.pose.orientation.y = odom_quat.y();
    odom.pose.pose.orientation.z = odom_quat.z();
    odom.pose.pose.orientation.w = odom_quat.w();
    odom.pose.covariance[0] = 0.001;
    odom.pose.covariance[7] = 0.001;
    odom.pose.covariance[35] = 0.001;

    odom.twist.twist.linear.x = linear_x;
    odom.twist.twist.linear.y = linear_y;
    odom.twist.twist.angular.z = angular_z;
    odom.twist.covariance[0] = 0.0001;
    odom.twist.covariance[7] = 0.0001;
    odom.twist.covariance[35] = 0.0001;
    odom_pub_->publish(odom);

    if (publish_tf_ && tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped odom_tf;
      {
        uint64_t tns = current_time.nanoseconds();
        odom_tf.header.stamp.sec = static_cast<int32_t>(tns / 1000000000ULL);
        odom_tf.header.stamp.nanosec = static_cast<uint32_t>(tns % 1000000000ULL);
      }
      odom_tf.header.frame_id = frame_id_;
      odom_tf.child_frame_id = child_frame_id_;
      odom_tf.transform.translation.x = x_pos_;
      odom_tf.transform.translation.y = y_pos_;
      odom_tf.transform.translation.z = 0.0;
      odom_tf.transform.rotation.x = odom_quat.x();
      odom_tf.transform.rotation.y = odom_quat.y();
      odom_tf.transform.rotation.z = odom_quat.z();
      odom_tf.transform.rotation.w = odom_quat.w();
      tf_broadcaster_->sendTransform(odom_tf);
    }
  }

  struct WheelSample {
    bool valid{false};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    int16_t lf{0};
    int16_t lr{0};
    int16_t rr{0};
    int16_t rf{0};
  };

  std::mutex sample_mutex_;
  WheelSample sample_;
  rclcpp::Subscription<nexus_base_ros::msg::Encoders>::SharedPtr wheel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr update_timer_;

  std::string frame_id_;
  std::string child_frame_id_;
  bool publish_tf_;
  double wheel_separation_width_;
  double wheel_separation_length_;
  double update_rate_hz_;

  double x_pos_;
  double y_pos_;
  double heading_;
  rclcpp::Time last_time_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NexusOdometry>());
  rclcpp::shutdown();
  return 0;
}

