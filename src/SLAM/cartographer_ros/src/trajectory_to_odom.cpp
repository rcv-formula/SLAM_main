#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

using std::placeholders::_1;

namespace {

double StampToSeconds(const builtin_interfaces::msg::Time& stamp)
{
    return static_cast<double>(stamp.sec) + 1e-9 * stamp.nanosec;
}

std::string DoubleToCsv(const double value)
{
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(9) << value;
    return stream.str();
}

double YawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
{
    return std::atan2(2. * (q.w * q.z + q.x * q.y),
                      1. - 2. * (q.y * q.y + q.z * q.z));
}

}  // namespace

class TrackedPoseToOdom : public rclcpp::Node
{
public:
    TrackedPoseToOdom()
        : Node("tracked_pose_to_odom"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        // /tracked_pose 구독자 생성
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/tracked_pose", 20, std::bind(&TrackedPoseToOdom::callback, this, _1));

        // /odom 퍼블리셔 생성
        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 20);

        const char* output_trace_csv_path =
            std::getenv("CARTOGRAPHER_ODOM_OUTPUT_TRACE_CSV_PATH");
        if (output_trace_csv_path != nullptr &&
            std::string(output_trace_csv_path) != "") {
            output_trace_csv_.open(output_trace_csv_path);
            if (output_trace_csv_.is_open()) {
                output_trace_csv_
                    << "now_sec,tracked_pose_stamp,odom_stamp,"
                    << "stamp_delta_sec,x,y,yaw,use_sim_time\n";
            }
        }

        // use_sim_time 파라미터 확인
        if (this->has_parameter("use_sim_time")) {
            this->get_parameter("use_sim_time", use_sim_time_);
        } else {
            use_sim_time_ = this->declare_parameter<bool>("use_sim_time", false);
        }

        RCLCPP_INFO(this->get_logger(), "TrackedPoseToOdom node initialized.");
    }

private:
    void callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped map_to_base_link;
        try {
            map_to_base_link =
                tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Could not transform map to base_link: %s", ex.what());
            return;
        }

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = msg->header.stamp;
        odom_msg.header.frame_id = map_to_base_link.header.frame_id.empty()
                                       ? "map"
                                       : map_to_base_link.header.frame_id;
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose.position.x = map_to_base_link.transform.translation.x;
        odom_msg.pose.pose.position.y = map_to_base_link.transform.translation.y;
        odom_msg.pose.pose.position.z = map_to_base_link.transform.translation.z;
        odom_msg.pose.pose.orientation = map_to_base_link.transform.rotation;
        odom_msg.pose.pose.position.z = 0.0;
        publisher_->publish(odom_msg);
        if (output_trace_csv_.is_open()) {
            const rclcpp::Time now = this->get_clock()->now();
            output_trace_csv_
                << DoubleToCsv(now.seconds()) << ','
                << DoubleToCsv(StampToSeconds(msg->header.stamp)) << ','
                << DoubleToCsv(StampToSeconds(odom_msg.header.stamp)) << ','
                << DoubleToCsv(StampToSeconds(odom_msg.header.stamp) -
                               StampToSeconds(msg->header.stamp)) << ','
                << DoubleToCsv(odom_msg.pose.pose.position.x) << ','
                << DoubleToCsv(odom_msg.pose.pose.position.y) << ','
                << DoubleToCsv(YawFromQuaternion(odom_msg.pose.pose.orientation))
                << ',' << (use_sim_time_ ? "1" : "0") << '\n';
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    std::ofstream output_trace_csv_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    bool use_sim_time_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrackedPoseToOdom>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
