#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "nexus_base_ros/msg/encoders.hpp"
#include "nexus_base_ros/srv/arming_enable.hpp"
#include "nexus_base_ros/srv/emergency_stop_enable.hpp"

namespace {
constexpr int kDefaultBaudrate = 115200; 
constexpr double kDefaultTxRateHz = 20.0; 
constexpr int kDefaultCmdTimeoutMs = 250;  
constexpr size_t kLineBufferSize = 256;   
}

class NexusSerialBridge : public rclcpp::Node {
public:
  NexusSerialBridge()
  : Node("nexus_serial_bridge"),
    io_(),
    serial_(io_),
    running_(true),
    armed_(true),
    has_cmd_(false) {
    port_ = declare_parameter<std::string>("serial_port", "/dev/ttyNEXUS");
    baudrate_ = declare_parameter<int>("serial_baud", kDefaultBaudrate);
    tx_rate_hz_ = declare_parameter<double>("tx_rate_hz", kDefaultTxRateHz);
    cmd_timeout_ms_ = declare_parameter<int>("cmd_timeout_ms", kDefaultCmdTimeoutMs);
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");

    wheel_vel_pub_ = create_publisher<nexus_base_ros::msg::Encoders>("wheel_vel", rclcpp::SensorDataQoS());
    wheel_enc_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>("wheel_enc", rclcpp::SensorDataQoS());

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel",
      rclcpp::SensorDataQoS(),
      std::bind(&NexusSerialBridge::cmdVelCallback, this, std::placeholders::_1));

    emergency_srv_ = create_service<nexus_base_ros::srv::EmergencyStopEnable>(
      "emergency_stop_enable",
      std::bind(&NexusSerialBridge::handleEmergencyStop, this, std::placeholders::_1, std::placeholders::_2));

    arming_srv_ = create_service<nexus_base_ros::srv::ArmingEnable>(
      "arming_enable",
      std::bind(&NexusSerialBridge::handleArming, this, std::placeholders::_1, std::placeholders::_2));

    openSerial();

    auto period = std::chrono::duration<double>(1.0 / std::max(1e-3, tx_rate_hz_));
    send_timer_ = create_wall_timer(period, std::bind(&NexusSerialBridge::sendLatestCmd, this));

    read_thread_ = std::thread(&NexusSerialBridge::readLoop, this);
  }

  ~NexusSerialBridge() override {
    running_.store(false);
    closeSerial();
    if (read_thread_.joinable()) {
      read_thread_.join();
    }
  }

private:
  void openSerial() {
    try {
      serial_.open(port_);
      serial_.set_option(boost::asio::serial_port_base::baud_rate(baudrate_));
      serial_.set_option(boost::asio::serial_port_base::character_size(8));
      serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
      serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
      serial_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
      RCLCPP_INFO(get_logger(), "Serial opened: %s @ %d", port_.c_str(), baudrate_);
    } catch (const std::exception &ex) {
      RCLCPP_ERROR(get_logger(), "Failed to open serial port %s: %s", port_.c_str(), ex.what());
    }
  }

  void closeSerial() {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_.is_open()) {
      boost::system::error_code ec;
      serial_.cancel(ec);
      serial_.close(ec);
    }
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    cmd_vx_mmps_ = msg->linear.x * 1000.0;
    cmd_vy_mmps_ = msg->linear.y * 1000.0;
    cmd_wz_mradps_ = msg->angular.z * 1000.0;
    last_cmd_time_ = now();
    has_cmd_ = true;
  }

  void handleEmergencyStop(
    const std::shared_ptr<nexus_base_ros::srv::EmergencyStopEnable::Request> req,
    std::shared_ptr<nexus_base_ros::srv::EmergencyStopEnable::Response> res) {
    if (req->enable) {
      armed_.store(false);
      writeLine("STOP\n");
    }
    res->success = true;
  }

  void handleArming(
    const std::shared_ptr<nexus_base_ros::srv::ArmingEnable::Request> req,
    std::shared_ptr<nexus_base_ros::srv::ArmingEnable::Response> res) {
    if (req->enable) {
      armed_.store(true);
    }
    res->success = true;
  }

  void sendLatestCmd() {
    if (!serial_.is_open()) {
      return;
    }

    bool send_stop = false;
    double vx = 0.0;
    double vy = 0.0;
    double wz = 0.0;

    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      if (!has_cmd_) {
        send_stop = true;
      } else {
        auto age_ms = (now() - last_cmd_time_).nanoseconds() / 1000000;
        if (age_ms > cmd_timeout_ms_) {
          send_stop = true;
        } else {
          vx = cmd_vx_mmps_;
          vy = cmd_vy_mmps_;
          wz = cmd_wz_mradps_;
        }
      }
    }

    if (!armed_.load()) {
      send_stop = true;
    }

    if (send_stop) {
      writeLine("STOP\n");
      return;
    }

    char line[kLineBufferSize];
    std::snprintf(line, sizeof(line), "TWIST %.0f %.0f %.0f\n", vx, vy, wz);
    writeLine(line);
  }

  void writeLine(const std::string &line) {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (!serial_.is_open()) {
      return;
    }
    boost::system::error_code ec;
    boost::asio::write(serial_, boost::asio::buffer(line), ec);
    if (ec) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Serial write failed: %s", ec.message().c_str());
    }
  }

  static int16_t clampToInt16(long value) {
    if (value > std::numeric_limits<int16_t>::max()) {
      return std::numeric_limits<int16_t>::max();
    }
    if (value < std::numeric_limits<int16_t>::min()) {
      return std::numeric_limits<int16_t>::min();
    }
    return static_cast<int16_t>(value);
  }

  void publishWheelVel(int v1, int v2, int v3, int v4) {
    nexus_base_ros::msg::Encoders msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.enc0 = clampToInt16(v3);
    msg.enc1 = clampToInt16(v4);
    msg.enc2 = clampToInt16(v1);
    msg.enc3 = clampToInt16(v2);
    wheel_vel_pub_->publish(msg);
  }

  void publishWheelEnc(long p1, long p2, long p3, long p4) {
    std_msgs::msg::Int32MultiArray msg;
    msg.data.resize(4);
    msg.data[0] = static_cast<int32_t>(p1);
    msg.data[1] = static_cast<int32_t>(p2);
    msg.data[2] = static_cast<int32_t>(p3);
    msg.data[3] = static_cast<int32_t>(p4);
    wheel_enc_pub_->publish(msg);
  }

  void readLoop() {
    boost::asio::streambuf buffer;
    while (running_.load()) {
      if (!serial_.is_open()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        continue;
      }

      boost::system::error_code ec;
      std::size_t n = boost::asio::read_until(serial_, buffer, '\n', ec);
      if (ec) {
        if (running_.load()) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Serial read failed: %s", ec.message().c_str());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }

      std::istream is(&buffer);
      std::string line;
      std::getline(is, line);
      if (line.empty()) {
        continue;
      }

      std::istringstream ss(line);
      std::string tag;
      ss >> tag;
      if (tag == "WHEEL") {
        int dt = 0;
        int v1 = 0, v2 = 0, v3 = 0, v4 = 0;
        if (ss >> dt >> v1 >> v2 >> v3 >> v4) {
          publishWheelVel(v1, v2, v3, v4);
        }
      } 
      
    }
  }

  boost::asio::io_context io_;
  boost::asio::serial_port serial_;
  std::thread read_thread_;
  std::mutex serial_mutex_;
  std::mutex cmd_mutex_;
  std::atomic<bool> running_;
  std::atomic<bool> armed_;
  bool has_cmd_;

  std::string port_;
  int baudrate_;
  double tx_rate_hz_;
  int cmd_timeout_ms_;
  std::string frame_id_;

  double cmd_vx_mmps_{0.0};
  double cmd_vy_mmps_{0.0};
  double cmd_wz_mradps_{0.0};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<nexus_base_ros::msg::Encoders>::SharedPtr wheel_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr wheel_enc_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Service<nexus_base_ros::srv::EmergencyStopEnable>::SharedPtr emergency_srv_;
  rclcpp::Service<nexus_base_ros::srv::ArmingEnable>::SharedPtr arming_srv_;
  rclcpp::TimerBase::SharedPtr send_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NexusSerialBridge>());
  rclcpp::shutdown();
  return 0;
}