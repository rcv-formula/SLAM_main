#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;

class TrackedPoseToOdom : public rclcpp::Node
{
public:
    TrackedPoseToOdom()
        : Node("tracked_pose_to_odom")
    {
        // /tracked_pose 구독자 생성
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/tracked_pose", 20, std::bind(&TrackedPoseToOdom::callback, this, _1));

        // /odom 퍼블리셔 생성
        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 20);

        // use_sim_time 파라미터 확인
        if (!this->get_parameter("use_sim_time", use_sim_time_)) {
            RCLCPP_INFO(this->get_logger(), "\033[33muse_sim_time NOT SET. Defaulting to false.\033[0m");
            use_sim_time_ = false;
        }

        RCLCPP_INFO(this->get_logger(), "TrackedPoseToOdom node initialized.");
    }

private:
    void callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        nav_msgs::msg::Odometry odom_msg;
        if (use_sim_time_) {
            odom_msg.header.stamp = this->get_clock()->now();
        } else {
            odom_msg.header.stamp = msg->header.stamp;
        }
        odom_msg.header.frame_id = msg->header.frame_id.empty() ? "map" : msg->header.frame_id;
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose = msg->pose;
        odom_msg.pose.pose.position.z = 0.0;
        publisher_->publish(odom_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;

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
